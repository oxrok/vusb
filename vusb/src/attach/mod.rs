// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Putting a served device on a real bus of the machine.

pub mod vhci;

use std::net::{SocketAddr, TcpStream};
use std::path::PathBuf;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crate::model::Speed;
use crate::transport::usbip;
use crate::{Error, Result};

/// Where an attached device ended up.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Placed {
    /// The controller's port.
    pub port: u8,
    /// The kernel's bus id, as under `/sys/bus/usb/devices`.
    pub busid: String,
    /// `bus.address`, when the device has one.
    pub bus_address: Option<String>,
    /// `/dev/bus/usb/BBB/DDD`, when the device has it.
    pub usbfs: Option<PathBuf>,
    /// The class drivers' nodes, sorted.
    pub nodes: Vec<PathBuf>,
}

/// The port a server holds, which a signal handler can give back too.
#[derive(Debug)]
pub struct PortGuard {
    port: Mutex<Option<u8>>,
    devid: u32,
    lost: AtomicBool,
}

impl PortGuard {
    fn new(devid: u32) -> PortGuard {
        PortGuard { port: Mutex::new(None), devid, lost: AtomicBool::new(false) }
    }

    /// Whether the port was found detached by someone else.
    pub fn lost(&self) -> bool {
        self.lost.load(Ordering::Relaxed)
    }

    /// Detaches the port, once. A port that no longer holds this
    /// server's `devid` is someone else's by now and is left alone.
    pub fn release(&self) {
        let Some(port) = self.port.lock().expect("port lock").take() else { return };
        match vhci::attached_ports(Some(self.devid)) {
            Ok(ps) if ps.iter().any(|p| p.port == port) => {
                if let Err(e) = vhci::detach(port) {
                    eprintln!("vusb: {e}");
                }
            }
            Ok(_) => {
                self.lost.store(true, Ordering::Relaxed);
                eprintln!(
                    "vusb: port {port} no longer holds devid {:#010x}: the device was detached \
                     by someone else during the run; the port is left alone",
                    self.devid
                );
            }
            Err(e) => eprintln!("vusb: port {port} left attached: {e}"),
        }
    }
}

/// A served device's place on `vhci-hcd`: the socket handed to the
/// controller, and what attaching it again takes.
#[derive(Debug)]
pub struct Attachment {
    addr: SocketAddr,
    busid: String,
    open: bool,
    socket: Option<TcpStream>,
    guard: std::sync::Arc<PortGuard>,
}

impl Attachment {
    /// For the device exported as `busid` by the server at `addr`, under
    /// `devid`; with `open`, its nodes are made world accessible, so the
    /// tools under test run without root.
    pub fn new(addr: SocketAddr, busid: &str, devid: u32, open: bool) -> Attachment {
        Attachment {
            addr,
            busid: busid.to_owned(),
            open,
            socket: None,
            guard: std::sync::Arc::new(PortGuard::new(devid)),
        }
    }

    /// What gives the port back, whatever else is going on.
    pub fn guard(&self) -> std::sync::Arc<PortGuard> {
        std::sync::Arc::clone(&self.guard)
    }

    /// Imports the device over a new connection and attaches it, on port
    /// `want` when that one is free (a driver finds a renumerated device
    /// by its port), and waits until the kernel has configured it.
    /// `configured` is called then, before the nodes are opened. A device
    /// that never configures is detached again, not left on the bus for
    /// the next run to trip over.
    ///
    /// # Errors
    ///
    /// All as [`Error::Attach`]: after them the device is off the bus.
    pub fn attach(
        &mut self,
        speed: Speed,
        want: Option<u8>,
        configured: impl FnOnce(),
    ) -> Result<Placed> {
        let (socket, _record) = usbip::import(self.addr, &self.busid)
            .map_err(|e| Error::Attach(format!("import: {e}")))?;
        let free = |p: u8| {
            vhci::ports().is_ok_and(|ps| {
                ps.iter().any(|s| s.port == p && s.hub == "hs" && s.status == vhci::PORT_FREE)
            })
        };
        let port = match want {
            Some(p) if free(p) => p,
            _ => vhci::free_port(speed)?,
        };
        vhci::attach(port, &socket, self.guard.devid, speed)?;
        // The controller took its own reference to the socket; ours is
        // kept until the device is detached.
        self.socket = Some(socket);
        *self.guard.port.lock().expect("port lock") = Some(port);
        let placed = Self::wait(port, self.open, configured);
        if placed.is_err() {
            self.guard.release();
        }
        placed
    }

    fn wait(port: u8, open: bool, configured: impl FnOnce()) -> Result<Placed> {
        let busid = vhci::busid(port, Duration::from_secs(10))?;
        let dir = vhci::wait_configured(&busid, Duration::from_secs(10))?;
        configured();
        let found = vhci::open_device(&dir, open, Duration::from_secs(5))?;
        Ok(Placed {
            port,
            busid,
            bus_address: found.bus_address,
            usbfs: found.usbfs,
            nodes: found.nodes,
        })
    }

    /// Takes the device off the bus: detaches its port, drops the
    /// connection and waits for the port to be free. Returns the port.
    ///
    /// # Errors
    ///
    /// When someone else detached the device, or the port stays in use.
    pub fn detach(&mut self) -> Result<Option<u8>> {
        let port = self.guard.port.lock().expect("port lock").take();
        if let Some(p) = port {
            let ours = vhci::attached_ports(Some(self.guard.devid))?.iter().any(|s| s.port == p);
            if !ours {
                self.guard.lost.store(true, Ordering::Relaxed);
                return Err(Error::Attach(
                    "the device was detached by someone else during the run".into(),
                ));
            }
            vhci::detach(p)?;
        }
        // The server sees the connection close only once our end goes.
        drop(self.socket.take());
        let start = Instant::now();
        while let Some(p) = port {
            if !vhci::attached_ports(None)?.iter().any(|s| s.port == p) {
                break;
            }
            if start.elapsed() > Duration::from_secs(5) {
                return Err(Error::Attach(format!("port {p} still in use 5 s after the detach")));
            }
            thread::sleep(Duration::from_millis(20));
        }
        Ok(port)
    }

    /// Drops our end of the connection, after the port was released.
    pub fn close(&mut self) {
        drop(self.socket.take());
    }
}
