// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Attaching a USB/IP socket to the Linux `vhci-hcd` virtual host
//! controller through its sysfs files, as the `usbip attach` tool
//! does: a free port is picked from `status`, and `port sockfd devid
//! speed` is written to `attach`; `detach` takes the port back. Needs
//! root, and the module (`modprobe vhci-hcd`).
//!
//! Once attached, [`open_device`] finds the nodes the device got (the
//! class drivers' `/dev/hidrawN`, `/dev/ttyACMN`, `/dev/usbtmcN`, and the
//! raw usbfs node `/dev/bus/usb/BBB/DDD` that libusb and libftdi open)
//! and makes them world accessible after udev is done with them. A
//! device that disconnects and comes back under new ids (EZ-USB
//! renumeration) gets new nodes; the caller calls [`open_device`]
//! again for its new bus id.

use std::fs;
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::model::Speed;
use crate::{Error, Result};

/// The controller's sysfs directory.
pub const SYSFS: &str = "/sys/devices/platform/vhci_hcd.0";

/// Whether the module is loaded.
pub fn available() -> bool {
    Path::new(SYSFS).join("attach").exists()
}

/// One line of `status`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PortStatus {
    /// `hs` or `ss`.
    pub hub: String,
    /// The port number, what `attach` and `detach` take.
    pub port: u8,
    /// `004` is free (`VDEV_ST_NULL`), `006` in use.
    pub status: u16,
    /// The `devid` the USB/IP server announced (`busnum << 16 | devnum`
    /// on the server's side), 0 when free.
    pub devid: u32,
    /// The bus id of the attached device, `0-0` when none.
    pub busid: String,
}

/// Parses the `status` file(s).
pub fn parse_status(text: &str) -> Vec<PortStatus> {
    text.lines()
        .filter_map(|l| {
            let f: Vec<&str> = l.split_whitespace().collect();
            if f.len() < 7 || f[0] == "hub" {
                return None;
            }
            Some(PortStatus {
                hub: f[0].to_owned(),
                port: f[1].parse().ok()?,
                status: f[2].parse().ok()?,
                devid: u32::from_str_radix(f[4], 16).ok()?,
                busid: f[6].to_owned(),
            })
        })
        .collect()
}

/// All ports: `status` and, with several controllers, `status.N`.
pub fn ports() -> Result<Vec<PortStatus>> {
    let dir = Path::new(SYSFS);
    let mut text = fs::read_to_string(dir.join("status"))
        .map_err(|e| Error::Attach(format!("{SYSFS}/status: {e} (is vhci-hcd loaded?)")))?;
    for n in 1.. {
        match fs::read_to_string(dir.join(format!("status.{n}"))) {
            Ok(t) => text.push_str(&t),
            Err(_) => break,
        }
    }
    Ok(parse_status(&text))
}

/// A free port of the hub that takes `speed`.
pub fn free_port(speed: Speed) -> Result<u8> {
    let hub = match speed {
        Speed::Low | Speed::Full | Speed::High => "hs",
    };
    ports()?
        .iter()
        .find(|p| p.hub == hub && p.status == PORT_FREE)
        .map(|p| p.port)
        .ok_or_else(|| Error::Attach(format!("no free {hub} port on {SYSFS}")))
}

/// Hands the connected socket to the controller.
#[cfg(unix)]
pub fn attach(port: u8, socket: &TcpStream, devid: u32, speed: Speed) -> Result<()> {
    use std::os::fd::AsRawFd;
    let line = format!(
        "{port} {} {devid} {}",
        socket.as_raw_fd(),
        crate::transport::usbip::speed_code(speed)
    );
    fs::write(Path::new(SYSFS).join("attach"), &line).map_err(|e| {
        Error::Attach(format!("attach '{line}': {e} (root needed; dmesg has the kernel's reason)"))
    })
}

/// There is no `vhci-hcd` here, and no file descriptor to hand it.
#[cfg(not(unix))]
pub fn attach(_port: u8, _socket: &TcpStream, _devid: u32, _speed: Speed) -> Result<()> {
    Err(Error::Attach("attaching a device needs Linux's vhci-hcd".into()))
}

/// Takes the port back; the device disappears from the bus.
pub fn detach(port: u8) -> Result<()> {
    fs::write(Path::new(SYSFS).join("detach"), format!("{port}"))
        .map_err(|e| Error::Attach(format!("detach port {port}: {e}")))
}

/// The status code of a free port.
pub const PORT_FREE: u16 = 4;

/// The ports in use whose device announced `devid`; with `None`, every
/// port in use.
pub fn attached_ports(devid: Option<u32>) -> Result<Vec<PortStatus>> {
    Ok(ports()?
        .into_iter()
        .filter(|p| p.status != PORT_FREE && devid.is_none_or(|d| p.devid == d))
        .collect())
}

/// The bus id the kernel gives the device on `port`; it appears in
/// `status` once the device has enumerated, so this waits up to
/// `timeout`.
pub fn busid(port: u8, timeout: Duration) -> Result<String> {
    let start = Instant::now();
    loop {
        if let Some(p) = ports()?.iter().find(|p| p.port == port && p.busid != "0-0") {
            return Ok(p.busid.clone());
        }
        if start.elapsed() > timeout {
            return Err(Error::Attach(format!(
                "port {port} has no enumerated device after {timeout:?}"
            )));
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Waits until the device at `busid` is configured; returns its sysfs
/// directory.
pub fn wait_configured(busid: &str, timeout: Duration) -> Result<PathBuf> {
    let dir = PathBuf::from("/sys/bus/usb/devices").join(busid);
    let start = Instant::now();
    loop {
        if let Ok(v) = fs::read_to_string(dir.join("bConfigurationValue"))
            && !v.trim().is_empty()
        {
            return Ok(dir);
        }
        if start.elapsed() > timeout {
            return Err(Error::Attach(format!(
                "{} not configured after {timeout:?}",
                dir.display()
            )));
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// The device nodes the kernel created for the device: `/dev/hidrawN`,
/// `/dev/ttyACMN`, `/dev/usbtmcN`. Waits up to `timeout` for the first
/// only: a device with several functions may be reported with those that
/// are there by then.
pub fn device_nodes(dir: &Path, timeout: Duration) -> Vec<PathBuf> {
    let start = Instant::now();
    loop {
        let mut nodes = Vec::new();
        collect_nodes(dir, 0, &mut nodes);
        nodes.sort();
        if !nodes.is_empty() || start.elapsed() > timeout {
            return nodes;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn collect_nodes(dir: &Path, depth: usize, out: &mut Vec<PathBuf>) {
    let Ok(entries) = fs::read_dir(dir) else { return };
    for entry in entries.flatten() {
        let path = entry.path();
        if !path.is_dir() || entry.file_type().is_ok_and(|t| t.is_symlink()) {
            continue;
        }
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if matches!(name.as_ref(), "hidraw" | "tty" | "usbmisc") {
            if let Ok(children) = fs::read_dir(&path) {
                for c in children.flatten() {
                    let child = c.file_name();
                    // usbhid's hiddev nodes live under /dev/usb.
                    let dev = if child.to_string_lossy().starts_with("hiddev") {
                        PathBuf::from("/dev/usb").join(&child)
                    } else {
                        PathBuf::from("/dev").join(&child)
                    };
                    if dev.exists() {
                        out.push(dev);
                    }
                }
            }
        } else if depth < 4 {
            collect_nodes(&path, depth + 1, out);
        }
    }
}

/// Makes the nodes readable and writable by everyone, so the tools
/// under test run without root.
#[cfg(unix)]
pub fn open_nodes(nodes: &[PathBuf]) -> Result<()> {
    use std::os::unix::fs::PermissionsExt;
    for n in nodes {
        fs::set_permissions(n, fs::Permissions::from_mode(0o666))
            .map_err(|e| Error::Attach(format!("chmod {}: {e}", n.display())))?;
    }
    Ok(())
}

/// There are no such nodes, nor their modes, here.
#[cfg(not(unix))]
pub fn open_nodes(_nodes: &[PathBuf]) -> Result<()> {
    Err(Error::Attach("opening device nodes needs Linux".into()))
}

/// Whether `node` is no longer readable and writable by everyone.
#[cfg(unix)]
fn lost_open_mode(node: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    fs::metadata(node).is_ok_and(|m| m.permissions().mode() & 0o666 != 0o666)
}

/// Never reached: [`open_nodes`] fails first here.
#[cfg(not(unix))]
fn lost_open_mode(_node: &Path) -> bool {
    false
}

/// The usbfs node of the device in sysfs directory `dir`,
/// `/dev/bus/usb/BBB/DDD` from its `busnum` and `devnum`, and the
/// `bus.address` pair `conn=` takes for it.
pub fn usbfs_node(dir: &Path) -> Option<(PathBuf, String)> {
    let read =
        |f: &str| -> Option<u16> { fs::read_to_string(dir.join(f)).ok()?.trim().parse().ok() };
    let (bus, dev) = (read("busnum")?, read("devnum")?);
    Some((PathBuf::from(format!("/dev/bus/usb/{bus:03}/{dev:03}")), format!("{bus}.{dev}")))
}

/// Waits for udev to finish the events queued so far (up to `timeout`),
/// so that the permissions it applies to new nodes are not applied
/// after ours. Without `udevadm`, nothing to wait for.
pub fn udev_settle(timeout: Duration) {
    let _ = std::process::Command::new("udevadm")
        .arg("settle")
        .arg(format!("--timeout={}", timeout.as_secs().max(1)))
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status();
}

/// Whether some interface of the device in `dir` is of a class whose
/// kernel driver creates a node (HID, CDC, USBTMC), so that waiting for
/// one is worth it.
fn expects_class_node(dir: &Path) -> bool {
    let Ok(entries) = fs::read_dir(dir) else { return false };
    entries.flatten().any(|e| {
        let read = |f: &str| fs::read_to_string(e.path().join(f)).unwrap_or_default();
        match read("bInterfaceClass").trim() {
            "03" | "02" | "0a" => true,
            "fe" => read("bInterfaceSubClass").trim() == "03",
            _ => false,
        }
    })
}

/// The nodes of an attached device.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DeviceNodes {
    /// `/dev/bus/usb/BBB/DDD`, when the device has one.
    pub usbfs: Option<PathBuf>,
    /// `bus.address`, as `conn=` takes it for libusb drivers.
    pub bus_address: Option<String>,
    /// The class drivers' nodes, sorted.
    pub nodes: Vec<PathBuf>,
}

impl DeviceNodes {
    /// All of them, usbfs first.
    pub fn all(&self) -> Vec<PathBuf> {
        self.usbfs.iter().cloned().chain(self.nodes.iter().cloned()).collect()
    }
}

/// Finds the nodes of the configured device in sysfs directory `dir`
/// and, with `open`, makes them readable and writable by everyone.
///
/// udev applies its own modes and ACLs to new nodes some time after the
/// kernel creates them; the queue is settled first, the modes set, and
/// checked again after a second settle (a late event resets them), up
/// to three rounds. A class node is waited for up to `timeout` when an
/// interface's class has one. Called once after the attach, and again
/// after the device reappears under a new address.
///
/// # Errors
///
/// When a node cannot be changed, or keeps losing its mode.
pub fn open_device(dir: &Path, open: bool, timeout: Duration) -> Result<DeviceNodes> {
    udev_settle(timeout);
    let wait = if expects_class_node(dir) { timeout } else { Duration::ZERO };
    let nodes = device_nodes(dir, wait);
    let (usbfs, bus_address) = match usbfs_node(dir) {
        Some((path, pair)) if path.exists() => (Some(path), Some(pair)),
        Some((_, pair)) => (None, Some(pair)),
        None => (None, None),
    };
    let found = DeviceNodes { usbfs, bus_address, nodes };
    if !open {
        return Ok(found);
    }
    let all = found.all();
    for round in 0..3 {
        open_nodes(&all)?;
        udev_settle(timeout);
        let lost: Vec<&PathBuf> = all.iter().filter(|n| lost_open_mode(n)).collect();
        if lost.is_empty() {
            return Ok(found);
        }
        if round == 2 {
            return Err(Error::Attach(format!(
                "{} lost mode 0666 again after udev settled",
                lost[0].display()
            )));
        }
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_status_table() {
        let text = "hub port sta spd dev      sockfd local_busid\n\
                    hs  0000 006 002 00010002 000004 1-1\n\
                    hs  0001 004 000 00000000 000000 0-0\n\
                    ss  0008 004 000 00000000 000000 0-0\n";
        let p = parse_status(text);
        assert_eq!(p.len(), 3);
        assert_eq!(
            p[0],
            PortStatus {
                hub: "hs".into(),
                port: 0,
                status: 6,
                devid: 0x0001_0002,
                busid: "1-1".into()
            }
        );
        assert_eq!(p[1].status, PORT_FREE);
        assert_eq!(p[1].devid, 0);
        assert_eq!(p[2].hub, "ss");
    }
}
