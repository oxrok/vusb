// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The gadget transport: the device on a USB device controller of this
//! machine, through Linux's Raw Gadget driver (`/dev/raw-gadget`,
//! `CONFIG_USB_RAW_GADGET`, Linux 5.7 on). Whatever host is at the other
//! end of the cable sees it: another machine and its operating system,
//! a phone, a boot loader.
//!
//! ```text
//! sudo modprobe raw_gadget
//! sudo vusb run cdc-echo --gadget auto
//! ```
//!
//! The controller (UDC) is the hardware of a device port: `dwc2` on a
//! Raspberry Pi Zero or 4, `musb` on a Beagle board, and the like on most
//! boards with an OTG port. A PC has none; there `dummy_hcd` is a
//! controller in software, wired to a host controller of the same
//! machine, which puts the device on the machine itself as `vhci-hcd`
//! does.
//!
//! Raw Gadget hands every control request to this process, descriptors
//! and `SET_CONFIGURATION` included (the controller only keeps
//! `SET_ADDRESS`), so the host talks to the model as over the other
//! transports, faults of the enumeration and all. The server is a codec
//! around the [`Bus`] as they are, and the transfers are in the log and
//! reach the observers the same way.
//!
//! # What a gadget turns around
//!
//! USB/IP and usbredir carry the host's transfers, so the bus sees each
//! one as the host submits it. A device controller shows less: a
//! device never learns that the host asked for data, only that data it
//! had ready was taken. So the server stands in for the host:
//!
//! - **IN is asked ahead.** For every IN endpoint the server keeps one
//!   transfer submitted to the bus; what the model answers waits in the
//!   controller until the host takes it, and only then is the model
//!   asked again. A model is asked one transfer before the host is; an
//!   armed `in=EP:N` counts those. A transfer asks for up to
//!   [`WINDOW`] bytes of a bulk endpoint, one packet of an interrupt
//!   endpoint.
//! - **What a model answers is what goes on the wire.** An answer of
//!   whole packets does not end the host's transfer, as on any device;
//!   an empty answer is a zero length packet.
//! - **OUT arrives packet by packet.** The server takes one packet,
//!   hands it to the bus as a transfer, and takes the next when the
//!   model took that one: a model that answers NAK holds the host back
//!   as on the wire.
//! - **A STALL is a halt.** An endpoint the model STALLs is halted; the
//!   host clears it. A model that STALLs an IN endpoint twice running
//!   keeps it so: it is wedged until the model answers otherwise. On an
//!   OUT endpoint the packet that the model refused was acknowledged
//!   already, and the halt meets the next one.
//! - **A control OUT with data gets no STALL** once the data is
//!   read, and the model needs the data to decide. The host sees such a
//!   request accepted; the log has a comment.
//! - **A control transfer is answered within a second**, with a STALL
//!   when the model still holds it: the driver takes no further request
//!   before.
//!
//! An unplug pulls the controller's connection down through its
//! `soft_connect` in sysfs, which is what a pulled cable does. A replug
//! and a renumeration then bind the controller anew, with the speed
//! the descriptors have by then.
//!
//! # Limits
//!
//! - **The controller has the endpoints it has.** Each endpoint of the
//!   configuration must find one of the controller's that fits its
//!   address, type and packet size; `SET_CONFIGURATION` gets a STALL
//!   otherwise, with a comment that names the endpoint.
//! - **Bus resets and disconnects** are reported by Linux 6.7 on.
//!   Before, the server takes the death of an endpoint's transfer for a
//!   reset; and it cannot let go of the controller while it waits for
//!   an event that never comes, so a replug pulls the connection up
//!   again instead of binding anew, which keeps the speed of the start
//!   and which `dummy_hcd` does not come back from.
//! - **Isochronous endpoints** are served a packet per transfer, at the
//!   controller's pace, and have not been tried on hardware:
//!   `dummy_hcd` has no isochronous transfers.
//! - **Root**, as `/dev/raw-gadget` and `soft_connect` are root's.

#[cfg(target_os = "linux")]
mod raw;

use std::collections::{HashMap, VecDeque};
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use crate::arm::Event as Armed;
use crate::bus::{Bus, Completion, Signal, Status, Step, Urb};
use crate::descriptor;
use crate::model::{
    BULK, ISOCHRONOUS, RECIPIENT_DEVICE, RECIPIENT_ENDPOINT, RECIPIENT_INTERFACE, STANDARD, Setup,
    Speed,
};
use crate::{Error, Result};

/// The poll period for held transfers.
pub const TICK: Duration = Duration::from_millis(20);

/// What one transfer asks of a bulk IN endpoint, in bytes: a page,
/// which every version of the driver takes in one request.
pub const WINDOW: usize = 4096;

/// How long the model may hold a control transfer before the host is
/// answered with a STALL.
const CONTROL_PATIENCE: Duration = Duration::from_secs(1);

/// How often an IN endpoint the model keeps stalled is offered again
/// when nothing else happens.
const STALL_RETRY: Duration = Duration::from_secs(1);

/// How long a replug waits for the driver to let go of the controller
/// before it pulls the connection up again instead.
const REBIND_PATIENCE: Duration = Duration::from_secs(1);

const CLEAR_FEATURE: u8 = 1;
const SET_FEATURE: u8 = 3;
const SET_CONFIGURATION: u8 = 9;
const SET_INTERFACE: u8 = 11;

/// `ECONNRESET` and `ESHUTDOWN` on Linux: how a transfer ends that the
/// controller dropped at a bus reset or a disconnect.
const GONE: [i32; 2] = [104, 108];

/// A USB device controller of this machine.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Udc {
    /// The controller's name, as in `/sys/class/udc`: `dummy_udc.0`,
    /// `fe980000.usb`.
    pub device: String,
    /// The name of its driver, which Raw Gadget asks for too:
    /// `dummy_udc`; for `dwc2` it is the controller's own name.
    pub driver: String,
}

impl Udc {
    const CLASS: &'static str = "/sys/class/udc";

    fn named(device: &str) -> Udc {
        let uevent = std::fs::read_to_string(format!("{}/{device}/uevent", Udc::CLASS));
        let driver = uevent
            .ok()
            .and_then(|u| {
                u.lines().find_map(|l| l.strip_prefix("USB_UDC_NAME=").map(str::to_owned))
            })
            .unwrap_or_else(|| device.to_owned());
        Udc { device: device.to_owned(), driver }
    }

    /// The controllers this machine has; none where it has no
    /// `/sys/class/udc`.
    pub fn list() -> Vec<Udc> {
        let mut names: Vec<String> = std::fs::read_dir(Udc::CLASS)
            .into_iter()
            .flatten()
            .filter_map(|entry| entry.ok()?.file_name().into_string().ok())
            .collect();
        names.sort();
        names.iter().map(|n| Udc::named(n)).collect()
    }

    /// The controller of this name, or with `auto` the only one there
    /// is.
    ///
    /// # Errors
    ///
    /// [`Error::Attach`] when the machine has no such controller, or
    /// with `auto` none or several.
    pub fn find(name: &str) -> Result<Udc> {
        let all = Udc::list();
        let names = || all.iter().map(|u| u.device.as_str()).collect::<Vec<_>>().join(", ");
        if all.is_empty() {
            return Err(Error::Attach(
                "this machine has no USB device controller: a board with a device port has \
                 one, and sudo modprobe dummy_hcd makes one in software"
                    .into(),
            ));
        }
        if name == "auto" {
            return match all.as_slice() {
                [only] => Ok(only.clone()),
                _ => Err(Error::Attach(format!("several controllers, name one: {}", names()))),
            };
        }
        all.iter()
            .find(|u| u.device == name)
            .cloned()
            .ok_or_else(|| Error::Attach(format!("no controller {name}; there is {}", names())))
    }
}

/// What the driver reports. Only the Linux driver (`raw`) constructs
/// these; elsewhere `open` refuses and no event is ever made.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
enum Event {
    /// Bound to the controller.
    Connect,
    /// A control request arrived on endpoint 0.
    Control(Setup),
    Suspend,
    Resume,
    /// A bus reset.
    Reset,
    /// The host is gone; some controllers say so for a reset too.
    Disconnect,
    /// One this code does not know.
    Other(u32),
}

/// The driver's side of a gadget: `/dev/raw-gadget` on Linux, a stand
/// in under test. Every method may be called from any thread; `event`,
/// `write` and `read` block until the host does its part.
trait Raw: Send + Sync {
    /// Waits for the next event.
    fn event(&self) -> io::Result<Event>;
    /// The data stage of the control IN request pending; `zero` ends it
    /// with a zero length packet when it is whole packets.
    fn ep0_write(&self, data: &[u8], zero: bool) -> io::Result<usize>;
    /// The data stage of the control OUT request pending, or with
    /// length 0 its acknowledgement.
    fn ep0_read(&self, length: usize) -> io::Result<Vec<u8>>;
    /// STALLs the control request pending.
    fn ep0_stall(&self) -> io::Result<()>;
    /// Tells the controller the device is configured.
    fn configure(&self) -> io::Result<()>;
    /// The current the configuration draws, in units of 2 mA.
    fn vbus_draw(&self, units: u32) -> io::Result<()>;
    /// Enables the endpoint of this descriptor; its handle.
    fn enable(&self, descriptor: &[u8]) -> io::Result<u32>;
    fn disable(&self, handle: u32) -> io::Result<()>;
    /// Sends on an IN endpoint; returns once the host took it.
    fn write(&self, handle: u32, data: &[u8]) -> io::Result<usize>;
    /// Receives on an OUT endpoint, up to `length` bytes or a short
    /// packet.
    fn read(&self, handle: u32, length: usize) -> io::Result<Vec<u8>>;
    /// Halts an endpoint until the host clears it.
    fn halt(&self, handle: u32) -> io::Result<()>;
    /// Halts an endpoint so that the host cannot clear it.
    fn wedge(&self, handle: u32) -> io::Result<()>;
    fn clear_halt(&self, handle: u32) -> io::Result<()>;
    /// Connects the device to the bus or takes it off, as a cable does.
    fn pull_up(&self, on: bool) -> io::Result<()>;
}

/// Where the driver was before a rebind that failed: nothing answers.
struct Unbound;

impl Unbound {
    fn error<T>() -> io::Result<T> {
        Err(io::Error::other("the gadget is not bound to the controller"))
    }
}

impl Raw for Unbound {
    fn event(&self) -> io::Result<Event> {
        Unbound::error()
    }
    fn ep0_write(&self, _: &[u8], _: bool) -> io::Result<usize> {
        Unbound::error()
    }
    fn ep0_read(&self, _: usize) -> io::Result<Vec<u8>> {
        Unbound::error()
    }
    fn ep0_stall(&self) -> io::Result<()> {
        Unbound::error()
    }
    fn configure(&self) -> io::Result<()> {
        Unbound::error()
    }
    fn vbus_draw(&self, _: u32) -> io::Result<()> {
        Unbound::error()
    }
    fn enable(&self, _: &[u8]) -> io::Result<u32> {
        Unbound::error()
    }
    fn disable(&self, _: u32) -> io::Result<()> {
        Unbound::error()
    }
    fn write(&self, _: u32, _: &[u8]) -> io::Result<usize> {
        Unbound::error()
    }
    fn read(&self, _: u32, _: usize) -> io::Result<Vec<u8>> {
        Unbound::error()
    }
    fn halt(&self, _: u32) -> io::Result<()> {
        Unbound::error()
    }
    fn wedge(&self, _: u32) -> io::Result<()> {
        Unbound::error()
    }
    fn clear_halt(&self, _: u32) -> io::Result<()> {
        Unbound::error()
    }
    fn pull_up(&self, _: bool) -> io::Result<()> {
        Unbound::error()
    }
}

/// What a thread that calls the driver holds of it. The driver comes
/// first: it is let go of before the count of such threads says so.
struct Grip {
    raw: Arc<dyn Raw>,
    _counted: Arc<()>,
}

/// Binds a controller anew, at this speed.
type Rebind = Box<dyn Fn(Speed) -> Result<Arc<dyn Raw>> + Send>;

#[cfg(target_os = "linux")]
fn open(udc: &Udc, speed: Speed) -> Result<Arc<dyn Raw>> {
    match raw::Device::open(udc, speed) {
        Ok(device) => Ok(Arc::new(device)),
        Err(e) => Err(Error::Attach(e.to_string())),
    }
}

#[cfg(not(target_os = "linux"))]
fn open(_: &Udc, _: Speed) -> Result<Arc<dyn Raw>> {
    Err(Error::Attach("a gadget needs Linux and its raw_gadget driver".into()))
}

/// Something the driver of the run asks the server to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Command {
    /// The device leaves the bus.
    Disconnect,
    /// The device comes (back): the bus reset, the connection up.
    Connect,
}

/// What the server tells about the host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Link {
    /// A host configured the device.
    Configured,
    /// The host reset the device or is gone, or the device left.
    Gone,
}

/// Drives a running server from elsewhere.
#[derive(Clone, Debug)]
pub struct Handle {
    commands: mpsc::Sender<Command>,
    configured: Arc<AtomicBool>,
}

impl Handle {
    /// The device leaves the bus, as a pulled cable does, until
    /// [`Handle::connect`]. The bus itself is the caller's to mark
    /// ([`Bus::unplug`]).
    pub fn disconnect(&self) {
        let _ = self.commands.send(Command::Disconnect);
    }

    /// The device comes back: the bus is reset (the new attach) and the
    /// host finds the device with the descriptors it has now.
    pub fn connect(&self) {
        let _ = self.commands.send(Command::Connect);
    }

    /// Whether a host has the device configured.
    pub fn configured(&self) -> bool {
        self.configured.load(Ordering::Relaxed)
    }
}

/// The server: one device on one controller. The bus is shared with
/// whoever drives the run.
pub struct Server {
    bus: Arc<Mutex<Bus>>,
    raw: Arc<dyn Raw>,
    commands: mpsc::Receiver<Command>,
    handle: Handle,
    rebind: Option<Rebind>,
    signals: Option<Box<dyn Fn(Signal) + Send>>,
    links: Option<Box<dyn Fn(Link) + Send>>,
}

impl std::fmt::Debug for Server {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Server").field("handle", &self.handle).finish_non_exhaustive()
    }
}

impl Server {
    /// Binds the device of `bus` to the controller and puts it on the
    /// bus: from here on a host finds it, and [`Server::run`] answers.
    ///
    /// # Errors
    ///
    /// [`Error::Attach`] when `/dev/raw-gadget` cannot be opened (the
    /// module, root) or the controller cannot be bound (it is not
    /// there, or has a gadget already).
    pub fn open(bus: Arc<Mutex<Bus>>, udc: &Udc) -> Result<Server> {
        let speed = bus.lock().expect("bus lock").device().descriptors().speed;
        let raw = open(udc, speed)?;
        let udc = udc.clone();
        let rebind: Rebind = Box::new(move |speed| open(&udc, speed));
        Ok(Server { rebind: Some(rebind), ..Server::with(bus, raw) })
    }

    fn with(bus: Arc<Mutex<Bus>>, raw: Arc<dyn Raw>) -> Server {
        let (tx, rx) = mpsc::channel();
        let handle = Handle { commands: tx, configured: Arc::new(AtomicBool::new(false)) };
        Server { bus, raw, commands: rx, handle, rebind: None, signals: None, links: None }
    }

    /// Drives the server from elsewhere.
    pub fn handle(&self) -> Handle {
        self.handle.clone()
    }

    /// What is called with each [`Signal`] of the bus, after the server
    /// acted on it: a renumeration (the device left the bus and came
    /// back), an armed event that fired. It runs on the server's thread
    /// and must not block.
    #[must_use]
    pub fn on_signal(mut self, f: impl Fn(Signal) + Send + 'static) -> Server {
        self.signals = Some(Box::new(f));
        self
    }

    /// What is called when a host has configured the device, and when
    /// it is no longer so.
    #[must_use]
    pub fn on_link(mut self, f: impl Fn(Link) + Send + 'static) -> Server {
        self.links = Some(Box::new(f));
        self
    }

    /// Serves the host until `stop` is set; the device then leaves the
    /// bus.
    ///
    /// # Errors
    ///
    /// When the driver stops reporting events.
    pub fn run(self, stop: &AtomicBool) -> Result<()> {
        let (tx, rx) = mpsc::channel();
        let mut pump = Pump {
            bus: self.bus,
            raw: self.raw,
            rebind: self.rebind,
            threads: Arc::default(),
            binding: 0,
            unbinding: Arc::default(),
            commands: self.commands,
            configured: self.handle.configured,
            signals: self.signals,
            links: self.links,
            messages: tx,
            endpoints: HashMap::new(),
            closing: Vec::new(),
            pending: HashMap::new(),
            todo: VecDeque::new(),
            alternates: HashMap::new(),
            next: 1,
            ids: 0,
            present: true,
            fresh: true,
            retried: Instant::now(),
        };
        pump.listen();
        let result = pump.serve(&rx, stop);
        pump.leave();
        result
    }
}

/// What reaches the server's thread.
enum Message {
    /// An event of the driver, with the binding it is of.
    Event(u64, io::Result<Event>),
    /// An IN endpoint's worker is back from the driver.
    Wrote { id: u64, result: io::Result<usize> },
    /// An OUT endpoint's worker is back with a packet.
    Read { id: u64, result: io::Result<Vec<u8>> },
}

/// What a worker does next.
enum Work {
    Write(Vec<u8>),
    Read(usize),
}

/// A transfer the bus is working on.
#[derive(Clone, Copy, Debug)]
enum Pending {
    /// The control request the driver waits on. `taken` says its data
    /// stage was read and so acknowledged already.
    Control { setup: Setup, taken: bool, since: Instant },
    /// A transfer of the endpoint at this address.
    Endpoint(u8),
}

/// What to do once the transfer at hand is dealt with.
#[derive(Clone, Copy, Debug)]
enum Todo {
    /// Asks the model for the next transfer of an IN endpoint.
    Offer(u8),
    /// Takes the next packet of an OUT endpoint.
    Take(u8),
}

/// An endpoint the controller has enabled.
struct Endpoint {
    /// Which enabling of the address this is: a worker's messages name
    /// it, so one from before a reset is told apart.
    id: u64,
    descriptor: Vec<u8>,
    handle: u32,
    isochronous: bool,
    /// What one transfer asks for (IN) or one packet holds (OUT).
    length: usize,
    /// To its worker, which ends when this is dropped.
    work: mpsc::Sender<Work>,
    /// The worker is in the driver, waiting for the host.
    busy: bool,
    /// The transfer the bus has of it.
    seqnum: Option<u32>,
    /// STALLs the model answered running.
    stalls: u32,
    wedged: bool,
}

impl Endpoint {
    fn address(&self) -> u8 {
        self.descriptor[2]
    }

    fn is_in(&self) -> bool {
        self.address() & 0x80 != 0
    }
}

/// The endpoint descriptors of a configuration, each with its interface
/// and alternate setting.
fn endpoints_of(config: &[u8]) -> Vec<(u8, u8, Vec<u8>)> {
    let mut found = Vec::new();
    let mut interface = (0, 0);
    for record in descriptor::records(config) {
        match record[1] {
            descriptor::INTERFACE if record.len() >= 9 => interface = (record[2], record[3]),
            descriptor::ENDPOINT if record.len() >= 7 => {
                found.push((interface.0, interface.1, record.to_vec()));
            }
            _ => {}
        }
    }
    found
}

fn gone(e: &io::Error) -> bool {
    e.raw_os_error().is_some_and(|code| GONE.contains(&code))
}

/// The server at work, on its thread.
struct Pump {
    bus: Arc<Mutex<Bus>>,
    raw: Arc<dyn Raw>,
    rebind: Option<Rebind>,
    /// Counts the threads that call the driver, with this one.
    threads: Arc<()>,
    /// Which binding to the controller this is: the events of one
    /// before are told apart.
    binding: u64,
    /// Tells the thread that waits for events to end at the next one,
    /// so that the driver can be closed.
    unbinding: Arc<AtomicBool>,
    commands: mpsc::Receiver<Command>,
    configured: Arc<AtomicBool>,
    signals: Option<Box<dyn Fn(Signal) + Send>>,
    links: Option<Box<dyn Fn(Link) + Send>>,
    /// For the workers.
    messages: mpsc::Sender<Message>,
    /// By address.
    endpoints: HashMap<u8, Endpoint>,
    /// Endpoints given up whose worker is still in the driver: id and
    /// handle. They are disabled once it is back.
    closing: Vec<(u64, u32)>,
    pending: HashMap<u32, Pending>,
    todo: VecDeque<Todo>,
    /// The alternate setting of each interface that left 0.
    alternates: HashMap<u8, u8>,
    next: u32,
    ids: u64,
    /// The connection to the bus is up.
    present: bool,
    /// The bus was reset and nothing was transferred since.
    fresh: bool,
    /// When the stalled endpoints were last offered again.
    retried: Instant,
}

impl Pump {
    fn bus(&self) -> MutexGuard<'_, Bus> {
        self.bus.lock().expect("bus lock")
    }

    fn comment(&self, message: &str) {
        self.bus().comment(message);
    }

    /// A step of the driver that failed and that the run survives.
    fn tried(&self, what: &str, result: io::Result<impl Sized>) {
        if let Err(e) = result {
            self.comment(&format!("gadget: {what}: {e}"));
        }
    }

    fn seqnum(&mut self) -> u32 {
        let n = self.next;
        self.next = self.next.wrapping_add(1).max(1);
        n
    }

    fn link(&self, link: Link) {
        let configured = link == Link::Configured;
        if self.configured.swap(configured, Ordering::Relaxed) == configured {
            return;
        }
        if let Some(f) = &self.links {
            f(link);
        }
    }

    /// The driver's events block with no way to give up: they get a
    /// thread of their own, which ends at the first event after it was
    /// told to, or after the server ended.
    fn grip(&self) -> Grip {
        Grip { raw: Arc::clone(&self.raw), _counted: Arc::clone(&self.threads) }
    }

    fn listen(&mut self) {
        self.binding += 1;
        self.unbinding = Arc::default();
        let (binding, unbinding) = (self.binding, Arc::clone(&self.unbinding));
        let (grip, messages) = (self.grip(), self.messages.clone());
        thread::spawn(move || {
            let grip = grip;
            loop {
                let event = grip.raw.event();
                let failed = event.is_err();
                if unbinding.load(Ordering::Relaxed)
                    || messages.send(Message::Event(binding, event)).is_err()
                    || failed
                {
                    return;
                }
            }
        });
    }

    fn serve(&mut self, messages: &mpsc::Receiver<Message>, stop: &AtomicBool) -> Result<()> {
        while !stop.load(Ordering::Relaxed) {
            match messages.recv_timeout(TICK) {
                Ok(message) => self.message(message)?,
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return Ok(()),
            }
            while let Ok(command) = self.commands.try_recv() {
                match command {
                    Command::Disconnect => self.unplug(),
                    Command::Connect => self.replug(),
                }
            }
            let polled = self.bus().poll();
            self.deliver(polled);
            self.patience();
            self.work();
        }
        Ok(())
    }

    /// Does what the transfers so far left to do, and what that leaves.
    fn work(&mut self) {
        while let Some(todo) = self.todo.pop_front() {
            match todo {
                Todo::Offer(address) => self.offer(address),
                Todo::Take(address) => self.take(address),
            }
        }
    }

    fn message(&mut self, message: Message) -> Result<()> {
        match message {
            Message::Event(binding, _) if binding != self.binding => {}
            Message::Event(_, Ok(event)) => self.event(event),
            Message::Event(_, Err(e)) => {
                return Err(Error::Io(io::Error::new(e.kind(), format!("raw-gadget events: {e}"))));
            }
            Message::Wrote { id, result } => {
                if let Some(address) = self.back(id, result.as_ref().err()) {
                    self.todo.push_back(Todo::Offer(address));
                }
            }
            Message::Read { id, result } => {
                let failed = result.as_ref().err();
                if let (Some(address), Ok(data)) = (self.back(id, failed), &result) {
                    self.received(address, data.clone());
                }
            }
        }
        Ok(())
    }

    /// A worker is back from the driver: the address of its endpoint,
    /// when that is still in use and the transfer went through.
    fn back(&mut self, id: u64, failed: Option<&io::Error>) -> Option<u8> {
        if let Some(i) = self.closing.iter().position(|(closing, _)| *closing == id) {
            let (_, handle) = self.closing.swap_remove(i);
            self.tried("disabling an endpoint", self.raw.disable(handle));
            return None;
        }
        let endpoint = self.endpoints.values_mut().find(|e| e.id == id)?;
        endpoint.busy = false;
        let address = endpoint.address();
        match failed {
            None => Some(address),
            // The controller dropped the transfer: on a kernel that
            // reports no reset, this is how one shows.
            Some(e) if gone(e) => {
                self.reset("an endpoint's transfer was dropped");
                None
            }
            Some(e) => {
                self.comment(&format!("gadget: endpoint {address:#04x}: {e}; it is closed"));
                if let Some(dead) = self.endpoints.remove(&address) {
                    self.close(dead);
                }
                None
            }
        }
    }

    fn event(&mut self, event: Event) {
        match event {
            Event::Connect => self.comment("gadget: bound to the controller"),
            Event::Control(setup) => self.control(setup),
            Event::Suspend => self.comment("gadget: suspended"),
            Event::Resume => self.comment("gadget: resumed"),
            Event::Reset => self.reset("bus reset"),
            Event::Disconnect => self.reset("the host is gone"),
            Event::Other(kind) => self.comment(&format!("gadget: event {kind}")),
        }
    }

    /// The host reset the bus, or left: the endpoints are gone and the
    /// device starts over. Not while the device is off the bus by its
    /// own doing, and once for a reset reported twice.
    fn reset(&mut self, why: &str) {
        if !self.present || self.fresh {
            return;
        }
        self.comment(&format!("gadget: {why}"));
        self.forget();
        self.bus().reset();
        self.fresh = true;
        self.link(Link::Gone);
    }

    /// Gives up every endpoint and every transfer in flight.
    fn forget(&mut self) {
        let endpoints: Vec<Endpoint> = self.endpoints.drain().map(|(_, e)| e).collect();
        for endpoint in endpoints {
            self.close(endpoint);
        }
        let control = self.pending.values().any(|p| matches!(p, Pending::Control { .. }));
        if control {
            // The driver takes no request while one waits.
            let _ = self.raw.ep0_stall();
        }
        self.pending.clear();
        self.todo.clear();
        self.alternates.clear();
    }

    /// Gives an endpoint up: its worker ends, and the controller's
    /// endpoint is disabled once nothing of it is in the driver.
    fn close(&mut self, endpoint: Endpoint) {
        let Endpoint { id, handle, busy, work, .. } = endpoint;
        drop(work);
        if busy {
            self.closing.push((id, handle));
        } else {
            self.tried("disabling an endpoint", self.raw.disable(handle));
        }
    }

    /// The device leaves the bus, as a pulled cable does.
    fn unplug(&mut self) {
        if !self.present {
            return;
        }
        self.present = false;
        // The disconnect is the event that ends the wait for events,
        // where the kernel reports it.
        self.unbinding.store(self.rebind.is_some(), Ordering::Relaxed);
        self.tried("taking the device off the bus", self.raw.pull_up(false));
        self.forget();
        self.link(Link::Gone);
    }

    /// Whether no thread but this one holds the driver, so that
    /// dropping it here closes it; waited for.
    fn alone(&self) -> bool {
        let end = Instant::now() + REBIND_PATIENCE;
        while Arc::strong_count(&self.threads) > 1 && Instant::now() < end {
            thread::sleep(Duration::from_millis(5));
        }
        Arc::strong_count(&self.threads) == 1
    }

    /// The device comes back: the new attach. The controller is bound
    /// anew when the driver can be closed first; a connection that is
    /// only pulled down and up again leaves some controllers deaf
    /// (`dummy_hcd`), so that is for when it cannot.
    fn replug(&mut self) {
        if self.present {
            return;
        }
        self.bus().reset();
        self.fresh = true;
        let speed = self.bus().device().descriptors().speed;
        let alone = self.rebind.is_some() && {
            let alone = self.alone();
            // Whoever still waits for an event is to stay.
            self.unbinding.store(false, Ordering::Relaxed);
            alone || Arc::strong_count(&self.threads) == 1
        };
        if !alone {
            self.present = true;
            self.tried("putting the device on the bus", self.raw.pull_up(true));
            return;
        }
        // The controller takes one gadget: the old one goes first.
        self.raw = Arc::new(Unbound);
        self.closing.clear();
        let bound = self.rebind.as_ref().map(|rebind| rebind(speed));
        match bound {
            Some(Ok(raw)) => {
                self.raw = raw;
                self.present = true;
                self.listen();
            }
            Some(Err(e)) => self.comment(&format!("gadget: binding the controller again: {e}")),
            None => {}
        }
    }

    /// The end of the run.
    fn leave(&mut self) {
        self.unbinding.store(true, Ordering::Relaxed);
        if self.present {
            let _ = self.raw.pull_up(false);
        }
        self.forget();
        self.link(Link::Gone);
    }

    fn control(&mut self, setup: Setup) {
        self.fresh = false;
        let with_data = !setup.is_in() && setup.length > 0;
        let data = if with_data {
            match self.raw.ep0_read(usize::from(setup.length)) {
                Ok(data) => data,
                Err(e) => {
                    self.comment(&format!("gadget: the data of a control request: {e}"));
                    return;
                }
            }
        } else {
            Vec::new()
        };
        let seqnum = self.seqnum();
        self.pending
            .insert(seqnum, Pending::Control { setup, taken: with_data, since: Instant::now() });
        let step = self.bus().submit(Urb::control(seqnum, setup, data));
        self.deliver(step);
    }

    /// Answers the host for a control request the model held too long:
    /// the driver takes no other before.
    fn patience(&mut self) {
        let late = self.pending.iter().find_map(|(seqnum, p)| match p {
            Pending::Control { taken, since, .. } if since.elapsed() >= CONTROL_PATIENCE => {
                Some((*seqnum, *taken))
            }
            _ => None,
        });
        let Some((seqnum, taken)) = late else { return };
        self.pending.remove(&seqnum);
        self.bus().unlink(seqnum);
        self.comment("gadget: a control transfer the model held for a second is STALLed");
        if !taken {
            self.tried("stalling a control request", self.raw.ep0_stall());
        }
        if self.retried.elapsed() >= STALL_RETRY {
            self.wake_stalled();
        }
    }

    fn deliver(&mut self, step: Step) {
        for completion in step.completions {
            self.completed(completion);
        }
        for signal in step.signals {
            self.signal(signal);
        }
        if self.retried.elapsed() >= STALL_RETRY {
            self.wake_stalled();
        }
    }

    fn signal(&mut self, signal: Signal) {
        match &signal {
            Signal::Fired(Armed::Unplug) => self.unplug(),
            Signal::Renumeration(_) => {
                self.unplug();
                self.replug();
            }
            _ => {}
        }
        if let Some(f) = &self.signals {
            f(signal);
        }
    }

    fn completed(&mut self, completion: Completion) {
        match self.pending.remove(&completion.seqnum) {
            Some(Pending::Control { setup, taken, .. }) => self.answer(setup, taken, &completion),
            Some(Pending::Endpoint(address)) => {
                let Some(endpoint) = self.endpoints.get_mut(&address) else { return };
                if endpoint.seqnum != Some(completion.seqnum) {
                    return;
                }
                endpoint.seqnum = None;
                if endpoint.is_in() {
                    self.answered(address, completion);
                } else {
                    self.taken(address, completion.status);
                }
            }
            None => {}
        }
    }

    /// Answers the host for a control request the bus completed.
    fn answer(&mut self, setup: Setup, taken: bool, completion: &Completion) {
        if completion.status != Status::Ok {
            if taken {
                self.comment(&format!(
                    "gadget: control request {:#04x} is STALLed after its data, which a gadget \
                     has acknowledged already: the host saw it accepted",
                    setup.request
                ));
            } else {
                self.tried("stalling a control request", self.raw.ep0_stall());
            }
            self.wake_stalled();
            return;
        }
        if let Err(e) = self.apply(setup) {
            self.comment(&format!(
                "gadget: request {:#04x} cannot be carried out: {e}",
                setup.request
            ));
            if !taken {
                self.tried("stalling a control request", self.raw.ep0_stall());
            }
            return;
        }
        if setup.is_in() {
            // Shorter than asked for and whole packets: a zero length
            // packet says it is all.
            let short = completion.data.len() < usize::from(setup.length);
            self.tried("answering a control request", self.raw.ep0_write(&completion.data, short));
        } else if !taken {
            self.tried("acknowledging a control request", self.raw.ep0_read(0));
        }
        if setup.kind() == STANDARD
            && setup.recipient() == RECIPIENT_DEVICE
            && setup.request == SET_CONFIGURATION
        {
            self.link(if setup.value == 0 { Link::Gone } else { Link::Configured });
        }
        self.wake_stalled();
    }

    /// What a standard request the device took changes on the
    /// controller: the endpoints of a configuration or an alternate
    /// setting, a halt.
    fn apply(&mut self, setup: Setup) -> io::Result<()> {
        if setup.kind() != STANDARD || setup.is_in() {
            return Ok(());
        }
        match (setup.recipient(), setup.request) {
            (RECIPIENT_DEVICE, SET_CONFIGURATION) => {
                self.alternates.clear();
                self.endpoints()?;
                let power = {
                    let bus = self.bus();
                    let device = bus.device();
                    device.descriptors().config_by_value(device.configuration()).map(|c| c[8])
                };
                if let Some(power) = power {
                    self.tried("the bus current", self.raw.vbus_draw(u32::from(power)));
                    self.tried("the configured state", self.raw.configure());
                }
            }
            (RECIPIENT_INTERFACE, SET_INTERFACE) => {
                self.alternates.insert(setup.index as u8, setup.value as u8);
                self.endpoints()?;
            }
            // Most controllers answer these themselves; one that hands
            // them up has the halt set or cleared here.
            (RECIPIENT_ENDPOINT, CLEAR_FEATURE | SET_FEATURE) if setup.value == 0 => {
                if let Some(endpoint) = self.endpoints.get_mut(&(setup.index as u8)) {
                    let handle = endpoint.handle;
                    endpoint.wedged = false;
                    let _ = if setup.request == SET_FEATURE {
                        self.raw.halt(handle)
                    } else {
                        self.raw.clear_halt(handle)
                    };
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Enables the endpoints of the configuration and alternate
    /// settings in force, and gives up the others.
    fn endpoints(&mut self) -> io::Result<()> {
        let wanted: Vec<Vec<u8>> = {
            let bus = self.bus();
            let device = bus.device();
            let config = device.descriptors().config_by_value(device.configuration());
            endpoints_of(config.unwrap_or_default())
                .into_iter()
                .filter(|(interface, alternate, _)| {
                    self.alternates.get(interface).copied().unwrap_or(0) == *alternate
                })
                .map(|(_, _, descriptor)| descriptor)
                .collect()
        };
        let stale: Vec<u8> = self
            .endpoints
            .iter()
            .filter(|(_, e)| !wanted.contains(&e.descriptor))
            .map(|(address, _)| *address)
            .collect();
        for address in stale {
            if let Some(endpoint) = self.endpoints.remove(&address) {
                self.close(endpoint);
            }
        }
        for descriptor in wanted {
            if !self.endpoints.contains_key(&descriptor[2]) {
                self.enable(descriptor)?;
            }
        }
        Ok(())
    }

    fn enable(&mut self, descriptor: Vec<u8>) -> io::Result<()> {
        let address = descriptor[2];
        let handle = self.raw.enable(&descriptor).map_err(|e| {
            io::Error::new(
                e.kind(),
                format!("endpoint {address:#04x} finds no endpoint of the controller: {e}"),
            )
        })?;
        let kind = descriptor[3] & 3;
        let packet = u16::from_le_bytes([descriptor[4], descriptor[5]]);
        // The low 11 bits; a high speed isochronous endpoint may send
        // up to three of them in a microframe.
        let packet = usize::from(packet & 0x07ff) * (1 + usize::from((packet >> 11) & 3));
        let is_in = address & 0x80 != 0;
        self.ids += 1;
        let id = self.ids;
        let (work, jobs) = mpsc::channel();
        let (grip, messages) = (self.grip(), self.messages.clone());
        thread::spawn(move || {
            let grip = grip;
            for job in jobs {
                let (message, failed) = match job {
                    Work::Write(data) => {
                        let result = grip.raw.write(handle, &data);
                        let failed = result.is_err();
                        (Message::Wrote { id, result }, failed)
                    }
                    Work::Read(length) => {
                        let result = grip.raw.read(handle, length);
                        let failed = result.is_err();
                        (Message::Read { id, result }, failed)
                    }
                };
                // An endpoint whose transfer failed is given up.
                if messages.send(message).is_err() || failed {
                    return;
                }
            }
        });
        self.endpoints.insert(
            address,
            Endpoint {
                id,
                descriptor,
                handle,
                isochronous: kind == ISOCHRONOUS,
                length: if is_in && kind == BULK { WINDOW.max(packet) } else { packet },
                work,
                busy: false,
                seqnum: None,
                stalls: 0,
                wedged: false,
            },
        );
        self.todo.push_back(if is_in { Todo::Offer(address) } else { Todo::Take(address) });
        Ok(())
    }

    /// Asks the model for the next transfer of an IN endpoint, unless
    /// one is asked for or on its way to the host.
    fn offer(&mut self, address: u8) {
        let seqnum = self.seqnum();
        let Some(endpoint) = self.endpoints.get_mut(&address) else { return };
        if endpoint.busy || endpoint.seqnum.is_some() {
            return;
        }
        endpoint.seqnum = Some(seqnum);
        let number = address & 0x0f;
        let urb = if endpoint.isochronous {
            Urb::iso_read(seqnum, number, vec![endpoint.length])
        } else {
            Urb::read(seqnum, number, endpoint.length)
        };
        self.pending.insert(seqnum, Pending::Endpoint(address));
        // The controller keeps the time of an isochronous stream: the
        // host takes a packet per service interval.
        let step = self.bus().submit_timed(urb);
        let held = !step.completions.iter().any(|c| c.seqnum == seqnum);
        if held {
            // The model stalls no longer: it has nothing to send.
            self.unwedge(address);
        }
        self.deliver(step);
    }

    fn unwedge(&mut self, address: u8) {
        let Some(endpoint) = self.endpoints.get_mut(&address) else { return };
        endpoint.stalls = 0;
        if std::mem::take(&mut endpoint.wedged) {
            let handle = endpoint.handle;
            self.tried("clearing a halt", self.raw.clear_halt(handle));
        }
    }

    /// The model answered an IN transfer.
    fn answered(&mut self, address: u8, completion: Completion) {
        if completion.status == Status::Stall {
            let Some(endpoint) = self.endpoints.get_mut(&address) else { return };
            endpoint.stalls += 1;
            let (handle, again) = (endpoint.handle, endpoint.stalls > 1);
            if again {
                // The model keeps it stalled: the host cannot clear it.
                endpoint.wedged = true;
                self.tried("wedging an endpoint", self.raw.wedge(handle));
            } else {
                self.tried("halting an endpoint", self.raw.halt(handle));
                // What the model has once the host cleared the halt.
                self.todo.push_back(Todo::Offer(address));
            }
            return;
        }
        self.unwedge(address);
        let Some(endpoint) = self.endpoints.get_mut(&address) else { return };
        endpoint.busy = true;
        // A worker that is gone says so itself.
        let _ = endpoint.work.send(Work::Write(completion.data));
    }

    /// The endpoints the model keeps stalled are offered again: what
    /// the host did since may have changed its mind.
    fn wake_stalled(&mut self) {
        self.retried = Instant::now();
        let stalled = self.endpoints.values().filter(|e| e.stalls > 1 && e.seqnum.is_none());
        let stalled: Vec<Todo> = stalled.map(|e| Todo::Offer(e.address())).collect();
        self.todo.extend(stalled);
    }

    /// Takes the next packet of an OUT endpoint.
    fn take(&mut self, address: u8) {
        let Some(endpoint) = self.endpoints.get_mut(&address) else { return };
        if endpoint.busy || endpoint.seqnum.is_some() {
            return;
        }
        endpoint.busy = true;
        let _ = endpoint.work.send(Work::Read(endpoint.length));
    }

    /// A packet arrived on an OUT endpoint.
    fn received(&mut self, address: u8, data: Vec<u8>) {
        self.fresh = false;
        let seqnum = self.seqnum();
        let Some(endpoint) = self.endpoints.get_mut(&address) else { return };
        endpoint.seqnum = Some(seqnum);
        let number = address & 0x0f;
        let urb = if endpoint.isochronous {
            Urb::iso_write(seqnum, number, &[&data])
        } else {
            Urb::write(seqnum, number, data)
        };
        self.pending.insert(seqnum, Pending::Endpoint(address));
        let step = self.bus().submit_timed(urb);
        self.deliver(step);
    }

    /// The model took, or refused, a packet of an OUT endpoint.
    fn taken(&mut self, address: u8, status: Status) {
        if status == Status::Stall
            && let Some(endpoint) = self.endpoints.get(&address)
        {
            self.tried("halting an endpoint", self.raw.halt(endpoint.handle));
        }
        self.todo.push_back(Todo::Take(address));
        self.wake_stalled();
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Condvar;

    use super::*;
    use crate::arm::Arm;
    use crate::device::Device;
    use crate::examples::{cdc_echo, msc_disk, relay};
    use crate::model::DeviceModel;
    use crate::observer::{Recorder, SharedObserver};

    const PATIENCE: Duration = Duration::from_secs(5);

    /// What the gadget did to the driver, the endpoints' data aside.
    #[derive(Clone, Debug, PartialEq, Eq)]
    enum Did {
        Ep0Write(Vec<u8>, bool),
        Ep0Ack,
        Ep0Stall,
        Configure,
        Enable(u8),
        Disable(u8),
        Halt(u8),
        Wedge(u8),
        ClearHalt(u8),
        PullUp(bool),
    }

    /// Packets between the gadget and the host of a test, one endpoint.
    #[derive(Default)]
    struct Pipe {
        state: Mutex<(VecDeque<Vec<u8>>, bool)>,
        changed: Condvar,
    }

    impl Pipe {
        fn push(&self, packet: Vec<u8>) {
            self.state.lock().unwrap().0.push_back(packet);
            self.changed.notify_all();
        }

        /// Closed, as the controller drops what is queued at a reset.
        fn close(&self) {
            self.state.lock().unwrap().1 = true;
            self.changed.notify_all();
        }

        fn shut_down() -> io::Error {
            io::Error::from_raw_os_error(GONE[1])
        }

        /// The next packet, waited for.
        fn pop(&self) -> io::Result<Vec<u8>> {
            let mut state = self.state.lock().unwrap();
            loop {
                if state.1 {
                    return Err(Pipe::shut_down());
                }
                if let Some(packet) = state.0.pop_front() {
                    self.changed.notify_all();
                    return Ok(packet);
                }
                state = self.changed.wait(state).unwrap();
            }
        }

        /// Waits until what was pushed was taken.
        fn drained(&self) -> io::Result<()> {
            let mut state = self.state.lock().unwrap();
            loop {
                if state.1 {
                    return Err(Pipe::shut_down());
                }
                if state.0.is_empty() {
                    return Ok(());
                }
                state = self.changed.wait(state).unwrap();
            }
        }

        /// The next packet, for the host of a test: none within its
        /// patience is a failure.
        fn expect(&self) -> Vec<u8> {
            let mut state = self.state.lock().unwrap();
            let end = Instant::now() + PATIENCE;
            loop {
                if let Some(packet) = state.0.pop_front() {
                    self.changed.notify_all();
                    return packet;
                }
                let left = end.saturating_duration_since(Instant::now());
                assert!(!left.is_zero(), "the gadget sent nothing");
                state = self.changed.wait_timeout(state, left).unwrap().0;
            }
        }
    }

    /// The driver and the host behind it, for a test.
    struct Fake {
        events: Mutex<mpsc::Receiver<Event>>,
        /// For the disconnect the driver reports itself.
        report: Mutex<mpsc::Sender<Event>>,
        did: Mutex<mpsc::Sender<Did>>,
        /// The data stages of control OUT requests to come.
        ep0: Mutex<VecDeque<Vec<u8>>>,
        /// Enabled endpoints, by handle: address and packets.
        pipes: Mutex<HashMap<u32, (u8, Arc<Pipe>)>>,
        handles: Mutex<u32>,
    }

    impl Fake {
        fn did(&self, did: Did) {
            let _ = self.did.lock().unwrap().send(did);
        }

        fn pipe(&self, handle: u32) -> io::Result<(u8, Arc<Pipe>)> {
            let pipes = self.pipes.lock().unwrap();
            pipes.get(&handle).cloned().ok_or_else(|| io::Error::other("no such endpoint"))
        }

        /// The endpoint at `address`, for the host.
        fn endpoint(&self, address: u8) -> Arc<Pipe> {
            let pipes = self.pipes.lock().unwrap();
            let pipe = pipes.values().find(|(a, _)| *a == address);
            Arc::clone(&pipe.unwrap_or_else(|| panic!("endpoint {address:#04x} is not enabled")).1)
        }

        /// What a reset or a disconnect does to the transfers queued.
        fn drop_transfers(&self) {
            for (_, pipe) in self.pipes.lock().unwrap().values() {
                pipe.close();
            }
        }
    }

    impl Raw for Fake {
        fn event(&self) -> io::Result<Event> {
            self.events.lock().unwrap().recv().map_err(|_| io::Error::other("no more events"))
        }

        fn ep0_write(&self, data: &[u8], zero: bool) -> io::Result<usize> {
            self.did(Did::Ep0Write(data.to_vec(), zero));
            Ok(data.len())
        }

        fn ep0_read(&self, length: usize) -> io::Result<Vec<u8>> {
            if length == 0 {
                self.did(Did::Ep0Ack);
                return Ok(Vec::new());
            }
            Ok(self.ep0.lock().unwrap().pop_front().expect("the data of a control request"))
        }

        fn ep0_stall(&self) -> io::Result<()> {
            self.did(Did::Ep0Stall);
            Ok(())
        }

        fn configure(&self) -> io::Result<()> {
            self.did(Did::Configure);
            Ok(())
        }

        fn vbus_draw(&self, _: u32) -> io::Result<()> {
            Ok(())
        }

        fn enable(&self, descriptor: &[u8]) -> io::Result<u32> {
            let mut handles = self.handles.lock().unwrap();
            *handles += 1;
            self.pipes.lock().unwrap().insert(*handles, (descriptor[2], Arc::default()));
            self.did(Did::Enable(descriptor[2]));
            Ok(*handles)
        }

        fn disable(&self, handle: u32) -> io::Result<()> {
            let gone = self.pipes.lock().unwrap().remove(&handle);
            let (address, _) = gone.ok_or_else(|| io::Error::other("not enabled"))?;
            self.did(Did::Disable(address));
            Ok(())
        }

        fn write(&self, handle: u32, data: &[u8]) -> io::Result<usize> {
            let (_, pipe) = self.pipe(handle)?;
            pipe.push(data.to_vec());
            pipe.drained()?;
            Ok(data.len())
        }

        fn read(&self, handle: u32, _: usize) -> io::Result<Vec<u8>> {
            self.pipe(handle)?.1.pop()
        }

        fn halt(&self, handle: u32) -> io::Result<()> {
            self.did(Did::Halt(self.pipe(handle)?.0));
            Ok(())
        }

        fn wedge(&self, handle: u32) -> io::Result<()> {
            self.did(Did::Wedge(self.pipe(handle)?.0));
            Ok(())
        }

        fn clear_halt(&self, handle: u32) -> io::Result<()> {
            self.did(Did::ClearHalt(self.pipe(handle)?.0));
            Ok(())
        }

        fn pull_up(&self, on: bool) -> io::Result<()> {
            if !on {
                self.drop_transfers();
                let _ = self.report.lock().unwrap().send(Event::Disconnect);
            }
            self.did(Did::PullUp(on));
            Ok(())
        }
    }

    /// A driver, and the host's ends of it.
    fn fake() -> (Arc<Fake>, mpsc::Sender<Event>, mpsc::Receiver<Did>) {
        let (events, event_rx) = mpsc::channel();
        let (did_tx, did) = mpsc::channel();
        let fake = Arc::new(Fake {
            events: Mutex::new(event_rx),
            report: Mutex::new(events.clone()),
            did: Mutex::new(did_tx),
            ep0: Mutex::default(),
            pipes: Mutex::default(),
            handles: Mutex::new(0),
        });
        (fake, events, did)
    }

    /// A driver the controller was bound anew with: the speed asked for,
    /// and the host's ends of it.
    type Bound = (Speed, Arc<Fake>, mpsc::Sender<Event>, mpsc::Receiver<Did>);

    /// A gadget served to a host that is the test.
    struct Served {
        fake: Arc<Fake>,
        events: mpsc::Sender<Event>,
        did: mpsc::Receiver<Did>,
        links: mpsc::Receiver<Link>,
        signals: mpsc::Receiver<Signal>,
        handle: Handle,
        bus: Arc<Mutex<Bus>>,
        seen: Arc<Mutex<Recorder>>,
        stop: Arc<AtomicBool>,
        thread: Option<thread::JoinHandle<Result<()>>>,
        /// The drivers the controller was bound anew with, when the
        /// test lets it be, each with the speed asked for.
        rebound: mpsc::Receiver<Bound>,
    }

    fn serve(model: Box<dyn DeviceModel>) -> Served {
        serve_with(model, false)
    }

    /// Serves `model`; with `rebinds` the controller can be bound anew,
    /// as the real one can.
    fn serve_with(model: Box<dyn DeviceModel>, rebinds: bool) -> Served {
        let seen = Arc::new(Mutex::new(Recorder::default()));
        let shared: SharedObserver = seen.clone();
        let bus = Arc::new(Mutex::new(Bus::new(Device::new(model)).with_observer(shared)));
        let (fake, events, did) = fake();
        let (link_tx, links) = mpsc::channel();
        let (signal_tx, signals) = mpsc::channel();
        let (rebound_tx, rebound) = mpsc::channel();
        let raw: Arc<dyn Raw> = fake.clone();
        let mut server = Server::with(Arc::clone(&bus), raw);
        if rebinds {
            server.rebind = Some(Box::new(move |speed| {
                let (fake, events, did) = self::fake();
                events.send(Event::Connect).unwrap();
                let raw: Arc<dyn Raw> = fake.clone();
                let _ = rebound_tx.send((speed, fake, events, did));
                Ok(raw)
            }));
        }
        let server = server
            .on_link(move |link| {
                let _ = link_tx.send(link);
            })
            .on_signal(move |signal| {
                let _ = signal_tx.send(signal);
            });
        let handle = server.handle();
        let stop = Arc::new(AtomicBool::new(false));
        let stopped = Arc::clone(&stop);
        let thread = Some(thread::spawn(move || server.run(&stopped)));
        events.send(Event::Connect).unwrap();
        Served { fake, events, did, links, signals, handle, bus, seen, stop, thread, rebound }
    }

    impl Served {
        fn did(&self) -> Did {
            self.did.recv_timeout(PATIENCE).expect("the gadget did nothing")
        }

        fn control(&self, request_type: u8, request: u8, value: u16, index: u16, length: u16) {
            let setup = Setup { request_type, request, value, index, length };
            self.events.send(Event::Control(setup)).unwrap();
        }

        /// A control IN request: the data the host gets.
        fn control_in(&self, request_type: u8, request: u8, value: u16, length: u16) -> Vec<u8> {
            self.control(request_type, request, value, 0, length);
            match self.did() {
                Did::Ep0Write(data, _) => data,
                other => panic!("{other:?}"),
            }
        }

        /// The first requests of a host, up to the configuration: what
        /// the gadget did for `SET_CONFIGURATION`.
        fn configure(&self) -> Vec<Did> {
            assert_eq!(self.control_in(0x80, 6, 0x0100, 18).len(), 18);
            self.control(0x00, SET_CONFIGURATION, 1, 0, 0);
            let mut did = Vec::new();
            loop {
                did.push(self.did());
                if did.last() == Some(&Did::Ep0Ack) {
                    break;
                }
            }
            assert_eq!(self.links.recv_timeout(PATIENCE), Ok(Link::Configured));
            did
        }

        fn violations(&self) -> Vec<String> {
            self.seen.lock().unwrap().violations.clone()
        }

        fn lines(&self) -> Vec<String> {
            self.seen
                .lock()
                .unwrap()
                .transfers
                .iter()
                .map(crate::observer::Transfer::line)
                .collect()
        }

        fn finish(mut self) {
            self.stop.store(true, Ordering::Relaxed);
            self.thread.take().unwrap().join().unwrap().unwrap();
        }
    }

    #[test]
    fn a_host_enumerates_and_configures_the_device() {
        let served = serve(cdc_echo::build(&[]).unwrap());
        // The device descriptor, cut to what the host asks for.
        assert_eq!(served.control_in(0x80, 6, 0x0100, 8).len(), 8);
        served.control(0x80, 6, 0x0100, 0, 64);
        let Did::Ep0Write(device, zero) = served.did() else { panic!() };
        assert_eq!(device.len(), 18);
        assert!(zero, "shorter than asked for: a zero length packet may end it");
        // A descriptor the device lacks, and a request it lacks.
        served.control(0x80, 6, 0x0f00, 0, 5);
        assert_eq!(served.did(), Did::Ep0Stall);
        served.control(0xc0, 0x77, 0, 0, 4);
        assert_eq!(served.did(), Did::Ep0Stall);

        assert_eq!(
            served.configure(),
            [Did::Enable(0x81), Did::Enable(0x82), Did::Enable(0x02), Did::Configure, Did::Ep0Ack]
        );
        assert!(served.handle.configured());
        assert_eq!(served.bus.lock().unwrap().device().configuration(), 1);

        // A control OUT with data: read first, then handed to the model.
        served.fake.ep0.lock().unwrap().push_back(vec![0x00, 0xc2, 0x01, 0x00, 0, 0, 8]);
        served.control(0x21, 0x20, 0, 0, 7);
        assert_eq!(served.control_in(0xa1, 0x21, 0, 7), [0x00, 0xc2, 0x01, 0x00, 0, 0, 8]);
        assert_eq!(served.violations().len(), 1, "{:?}", served.violations());
        served.finish();
    }

    #[test]
    fn data_goes_through_packet_by_packet() {
        let served = serve(cdc_echo::build(&[]).unwrap());
        served.configure();
        let (out, back) = (served.fake.endpoint(0x02), served.fake.endpoint(0x82));
        out.push(b"hello".to_vec());
        assert_eq!(back.expect(), b"hello");
        // Two packets of one write, and the echo of both.
        out.push(vec![0x55; 64]);
        out.push(b"!".to_vec());
        let mut echo = back.expect();
        while echo.len() < 65 {
            echo.extend_from_slice(&back.expect());
        }
        assert_eq!(echo[..64], [0x55; 64]);
        assert_eq!(echo[64], b'!');
        assert_eq!(served.violations(), Vec::<String>::new());
        let lines = served.lines();
        assert!(lines.contains(&"bulk 0x02 5 68656c6c6f -> ok".to_owned()), "{lines:?}");
        served.finish();
    }

    #[test]
    fn a_reset_gives_the_endpoints_up_and_the_host_starts_over() {
        let served = serve(cdc_echo::build(&[]).unwrap());
        served.configure();
        served.fake.endpoint(0x02).push(b"lost".to_vec());
        assert_eq!(served.fake.endpoint(0x82).expect(), b"lost");
        // The controller drops what is queued, then reports the reset.
        served.fake.drop_transfers();
        served.events.send(Event::Reset).unwrap();
        assert_eq!(served.links.recv_timeout(PATIENCE), Ok(Link::Gone));
        let mut disabled = vec![served.did(), served.did(), served.did()];
        disabled.sort_by_key(|d| format!("{d:?}"));
        assert_eq!(disabled, [Did::Disable(0x81), Did::Disable(0x82), Did::Disable(0x02)]);
        assert_eq!(served.bus.lock().unwrap().device().configuration(), 0);

        let did = served.configure();
        assert_eq!(did.iter().filter(|d| matches!(d, Did::Enable(_))).count(), 3);
        served.fake.endpoint(0x02).push(b"again".to_vec());
        assert_eq!(served.fake.endpoint(0x82).expect(), b"again");
        served.finish();
    }

    #[test]
    fn an_unplug_takes_the_device_off_the_bus_and_a_replug_puts_it_back() {
        let served = serve(relay::build(&[]).unwrap());
        served.configure();
        // Armed: at the second control read from here.
        served.bus.lock().unwrap().arm(Arm::parse("unplug in=0:2").unwrap());
        assert_eq!(served.control_in(0x80, 6, 0x0100, 18).len(), 18);
        served.control(0x80, 6, 0x0100, 0, 18);
        // The transfer that fired it is still answered; then the cable.
        let mut did = vec![served.did(), served.did()];
        did.retain(|d| !matches!(d, Did::Ep0Write(..)));
        assert_eq!(did, [Did::PullUp(false)]);
        assert_eq!(served.signals.recv_timeout(PATIENCE), Ok(Signal::Fired(Armed::Unplug)));
        assert_eq!(served.links.recv_timeout(PATIENCE), Ok(Link::Gone));
        assert!(served.bus.lock().unwrap().unplugged());
        assert_eq!(served.did(), Did::Disable(0x81));

        served.handle.connect();
        assert_eq!(served.did(), Did::PullUp(true));
        assert!(!served.bus.lock().unwrap().unplugged());
        served.configure();

        served.handle.disconnect();
        assert_eq!(served.did(), Did::PullUp(false));
        served.finish();
    }

    #[test]
    fn a_replug_binds_the_controller_anew() {
        let mut served = serve_with(cdc_echo::build(&[]).unwrap(), true);
        served.configure();
        served.handle.disconnect();
        assert_eq!(served.did(), Did::PullUp(false));
        assert_eq!(served.links.recv_timeout(PATIENCE), Ok(Link::Gone));

        // The old driver is closed, and a new one has the controller.
        served.handle.connect();
        let (speed, fake, events, did) = served.rebound.recv_timeout(PATIENCE).unwrap();
        assert_eq!(speed, Speed::Full);
        let old = std::mem::replace(&mut served.fake, fake);
        let end = Instant::now() + PATIENCE;
        while Arc::strong_count(&old) > 1 {
            assert!(Instant::now() < end, "the old driver is still held");
            thread::sleep(Duration::from_millis(5));
        }
        served.events = events;
        served.did = did;
        assert_eq!(served.configure().len(), 5);
        served.fake.endpoint(0x02).push(b"back".to_vec());
        assert_eq!(served.fake.endpoint(0x82).expect(), b"back");
        assert!(!served.bus.lock().unwrap().unplugged());
        served.finish();
    }

    /// One mass storage command, as a host does it through the gadget:
    /// what it read, and the status.
    fn command(
        served: &Served,
        tag: u32,
        block: &[u8],
        length: u32,
        data: Option<&[u8]>,
    ) -> (Vec<u8>, u8) {
        let (out, back) = (served.fake.endpoint(0x02), served.fake.endpoint(0x81));
        let mut cbw = b"USBC".to_vec();
        cbw.extend_from_slice(&tag.to_le_bytes());
        cbw.extend_from_slice(&length.to_le_bytes());
        cbw.extend_from_slice(&[if data.is_some() { 0 } else { 0x80 }, 0, block.len() as u8]);
        cbw.extend_from_slice(block);
        cbw.resize(31, 0);
        out.push(cbw);
        let mut read = Vec::new();
        match data {
            Some(data) => data.chunks(512).for_each(|packet| out.push(packet.to_vec())),
            None => {
                while read.len() < length as usize {
                    read.extend_from_slice(&back.expect());
                }
            }
        }
        let status = back.expect();
        assert_eq!(status[..4], *b"USBS");
        assert_eq!(status[4..8], tag.to_le_bytes());
        (read, status[12])
    }

    #[test]
    fn a_disk_is_read_and_written_through_a_gadget() {
        let served = serve(msc_disk::build(&[("size".to_owned(), "1".to_owned())]).unwrap());
        assert_eq!(served.configure()[..2], [Did::Enable(0x81), Did::Enable(0x02)]);
        let (inquiry, status) = command(&served, 1, &[0x12, 0, 0, 0, 36, 0], 36, None);
        assert_eq!((status, &inquiry[8..12]), (0, &b"vusb"[..]));

        // Twenty blocks: more than one transfer asks of the model.
        let data: Vec<u8> = (0..20 * 512).map(|i| (i / 512 + i % 7) as u8).collect();
        let write = [0x2a, 0, 0, 0, 0, 3, 0, 0, 20, 0];
        assert_eq!(command(&served, 2, &write, data.len() as u32, Some(&data)).1, 0);
        let read = [0x28, 0, 0, 0, 0, 3, 0, 0, 20, 0];
        let (back, status) = command(&served, 3, &read, data.len() as u32, None);
        assert_eq!(status, 0);
        assert_eq!(back, data);
        assert_eq!(served.violations(), Vec::<String>::new());
        served.finish();
    }

    #[test]
    fn a_model_that_keeps_an_endpoint_stalled_has_it_wedged() {
        let served = serve(msc_disk::build(&[]).unwrap());
        served.configure();
        // Not a command block wrapper: the disk stalls both endpoints
        // until the reset recovery.
        served.fake.endpoint(0x02).push(b"nonsense".to_vec());
        let mut did = vec![served.did(), served.did(), served.did()];
        did.sort_by_key(|d| format!("{d:?}"));
        assert_eq!(did, [Did::Halt(0x81), Did::Halt(0x02), Did::Wedge(0x81)]);

        // The reset recovery, and the disk answers again.
        served.control(0x21, 0xff, 0, 0, 0);
        assert_eq!(served.did(), Did::Ep0Ack);
        assert_eq!(served.did(), Did::ClearHalt(0x81));
        assert_eq!(command(&served, 9, &[0, 0, 0, 0, 0, 0], 0, None).1, 0);
        served.finish();
    }

    #[test]
    fn endpoints_are_told_from_their_interfaces() {
        let model = crate::examples::uac_mic::build(&[]).unwrap();
        let config = model.descriptors().configs[0].clone();
        let found = endpoints_of(&config);
        assert_eq!(found.len(), 1);
        // The microphone's stream is in an alternate setting of its own.
        assert!(found.iter().all(|(_, alternate, d)| *alternate == 1 && d[3] & 3 == ISOCHRONOUS));
        let disk = msc_disk::build(&[]).unwrap();
        let found = endpoints_of(&disk.descriptors().configs[0]);
        let addresses: Vec<u8> = found.iter().map(|(_, _, d)| d[2]).collect();
        assert_eq!(addresses, [0x81, 0x02]);
    }
}
