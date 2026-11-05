// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The usbredir server: the `usb-host` side of the USB redirection
//! protocol, version 0.7, as QEMU's `usb-redir` device (the
//! `usb-guest`) speaks it. One connection carries one device into a
//! virtual machine, on any platform QEMU runs on, with no kernel module
//! and no root.
//!
//! ```text
//! vusb run cdc-echo --usbredir /tmp/echo.sock
//! qemu-system-x86_64 ... -device qemu-xhci \
//!     -chardev socket,id=u,path=/tmp/echo.sock -device usb-redir,chardev=u
//! ```
//!
//! Implemented from the protocol document alone
//! (`usb-redirection-protocol.md` in the usbredir repository); no code
//! of usbredir or QEMU is used. Little endian, packed structs, a
//! header (type, length, id) on every packet: 12 bytes, or 16 once
//! both sides have said they take 64 bit ids. Three things set it
//! apart from USB/IP:
//!
//! - The device announces itself: `ep_info` and `interface_info`, then
//!   `device_connect`; `device_disconnect` takes it away, and the
//!   `device_disconnect_ack` capability lets a new device follow on the
//!   same connection, which is what a renumeration and a replug need.
//! - Configuration is a message: the guest sends `set_configuration`
//!   and `set_alt_setting` rather than the control transfers, and the
//!   endpoint and interface info is sent again before each answer.
//! - Interrupt IN is push: the guest says `start_interrupt_receiving`
//!   once, and the server polls the endpoint and sends each packet with
//!   its own id.
//!
//! The server is a codec around the [`Bus`], as the USB/IP one is: what
//! a host controller does (holding a transfer the device NAKs, keeping
//! each endpoint's order, cutting a babble) is the bus's, so a device
//! behaves the same on both. Held transfers are offered again after
//! every packet and every [`TICK`].
//!
//! Capabilities offered: `connect_device_version`,
//! `ep_info_max_packet_size`, `device_disconnect_ack`, `64bits_ids` and
//! `32bits_bulk_length`; QEMU's XHCI controller takes a device only
//! with the last three. Not offered: bulk streams, buffered bulk
//! receiving, filters.
//!
//! An isochronous stream is the one place the clock comes in: the guest
//! says `start_iso_stream` once, and for an IN endpoint the server then
//! offers the device one packet per service interval of the endpoint,
//! by its own clock, and sends each on. The model still sees one call
//! per interval and keeps no time itself. So a stream is in real time
//! here whatever [`DeviceModel::realtime`](crate::model::DeviceModel::realtime)
//! says: the server submits each packet when its interval comes
//! ([`Bus::submit_timed`]).

use std::collections::HashMap;
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use crate::arm::Event;
use crate::bus::{Bus, Completion, Signal, Status, Step, Unlink, Urb};
use crate::descriptor;
use crate::model::{CONTROL, Descriptors, Endpoint, INTERRUPT, ISOCHRONOUS, Setup, Speed};
use crate::transport::socket::{Accepted, Address, ListenOptions, Listener, Reader, Writer};
use crate::{Error, Result};

/// The poll period for held transfers.
pub const TICK: Duration = Duration::from_millis(20);

/// What our `hello` says.
const VERSION: &str = concat!("vusb ", env!("CARGO_PKG_VERSION"));

// Control packet types, in the protocol's order.
const HELLO: u32 = 0;
const DEVICE_CONNECT: u32 = 1;
const DEVICE_DISCONNECT: u32 = 2;
const RESET: u32 = 3;
const INTERFACE_INFO: u32 = 4;
const EP_INFO: u32 = 5;
const SET_CONFIGURATION: u32 = 6;
const GET_CONFIGURATION: u32 = 7;
const CONFIGURATION_STATUS: u32 = 8;
const SET_ALT_SETTING: u32 = 9;
const GET_ALT_SETTING: u32 = 10;
const ALT_SETTING_STATUS: u32 = 11;
const START_ISO_STREAM: u32 = 12;
const STOP_ISO_STREAM: u32 = 13;
const ISO_STREAM_STATUS: u32 = 14;
const START_INTERRUPT_RECEIVING: u32 = 15;
const STOP_INTERRUPT_RECEIVING: u32 = 16;
const INTERRUPT_RECEIVING_STATUS: u32 = 17;
const ALLOC_BULK_STREAMS: u32 = 18;
const FREE_BULK_STREAMS: u32 = 19;
const BULK_STREAMS_STATUS: u32 = 20;
const CANCEL_DATA_PACKET: u32 = 21;
const FILTER_REJECT: u32 = 22;
const FILTER_FILTER: u32 = 23;
const DEVICE_DISCONNECT_ACK: u32 = 24;
const START_BULK_RECEIVING: u32 = 25;
const STOP_BULK_RECEIVING: u32 = 26;
const BULK_RECEIVING_STATUS: u32 = 27;
// Data packet types.
const CONTROL_PACKET: u32 = 100;
const BULK_PACKET: u32 = 101;
const ISO_PACKET: u32 = 102;
const INTERRUPT_PACKET: u32 = 103;

// Status codes.
const SUCCESS: u8 = 0;
const CANCELLED: u8 = 1;
const INVAL: u8 = 2;
const IOERROR: u8 = 3;
const STALL: u8 = 4;
const BABBLE: u8 = 6;

// Capability bits.
const CAP_BULK_STREAMS: u32 = 0;
const CAP_CONNECT_DEVICE_VERSION: u32 = 1;
const CAP_DEVICE_DISCONNECT_ACK: u32 = 3;
const CAP_EP_INFO_MAX_PACKET_SIZE: u32 = 4;
const CAP_64BITS_IDS: u32 = 5;
const CAP_32BITS_BULK_LENGTH: u32 = 6;

/// The capabilities this server offers.
const OURS: u32 = 1 << CAP_CONNECT_DEVICE_VERSION
    | 1 << CAP_DEVICE_DISCONNECT_ACK
    | 1 << CAP_EP_INFO_MAX_PACKET_SIZE
    | 1 << CAP_64BITS_IDS
    | 1 << CAP_32BITS_BULK_LENGTH;

/// The header before the id, which is 4 or 8 bytes.
const HEADER: usize = 8;
const HELLO_VERSION: usize = 64;
/// Endpoint type of a slot `ep_info` has nothing for.
const TYPE_INVALID: u8 = 255;
/// The longest packet taken from the guest.
const MAX_PACKET: u32 = 1 << 24;

fn status_of(status: Status) -> u8 {
    match status {
        Status::Ok => SUCCESS,
        Status::Stall => STALL,
        Status::Babble => BABBLE,
        Status::Unlinked => CANCELLED,
        Status::Missed => IOERROR,
    }
}

/// The speed code of `device_connect`.
pub fn speed_code(speed: Speed) -> u8 {
    match speed {
        Speed::Low => 0,
        Speed::Full => 1,
        Speed::High => 2,
    }
}

/// A packet as it goes over the wire.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet {
    /// The type code.
    pub kind: u32,
    /// The id, which a reply echoes; 32 bits on the wire unless both
    /// sides take 64.
    pub id: u64,
    /// The type's header and data, as one.
    pub payload: Vec<u8>,
}

impl Packet {
    fn new(kind: u32, id: u64, payload: Vec<u8>) -> Packet {
        Packet { kind, id, payload }
    }

    /// Writes it, the id in 64 bits when `wide`.
    fn write_to(&self, out: &mut dyn Write, wide: bool) -> io::Result<()> {
        let mut b = Vec::with_capacity(HEADER + 8 + self.payload.len());
        b.extend_from_slice(&self.kind.to_le_bytes());
        b.extend_from_slice(&(self.payload.len() as u32).to_le_bytes());
        if wide {
            b.extend_from_slice(&self.id.to_le_bytes());
        } else {
            b.extend_from_slice(&(self.id as u32).to_le_bytes());
        }
        b.extend_from_slice(&self.payload);
        out.write_all(&b)?;
        out.flush()
    }

    /// Reads one, its id in 64 bits when `wide`.
    fn read_from(input: &mut dyn Read, wide: bool) -> io::Result<Packet> {
        let mut h = [0u8; HEADER];
        input.read_exact(&mut h)?;
        let kind = u32::from_le_bytes([h[0], h[1], h[2], h[3]]);
        let length = u32::from_le_bytes([h[4], h[5], h[6], h[7]]);
        let id = if wide {
            let mut id = [0u8; 8];
            input.read_exact(&mut id)?;
            u64::from_le_bytes(id)
        } else {
            let mut id = [0u8; 4];
            input.read_exact(&mut id)?;
            u64::from(u32::from_le_bytes(id))
        };
        if length > MAX_PACKET {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("packet of {length} bytes, more than a device moves"),
            ));
        }
        let mut payload = vec![0u8; length as usize];
        input.read_exact(&mut payload)?;
        Ok(Packet { kind, id, payload })
    }
}

/// Whether the ids after the hellos are 64 bits wide: both sides must
/// have offered it. `theirs` is the peer's hello.
fn wide_after(ours: u32, theirs: &Packet) -> bool {
    theirs.kind == HELLO
        && parse_hello(&theirs.payload)
            .is_some_and(|(_, caps)| ours & caps & (1 << CAP_64BITS_IDS) != 0)
}

fn le16(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

fn le32(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

/// Our `hello`.
fn hello() -> Packet {
    let mut p = VERSION.as_bytes()[..VERSION.len().min(HELLO_VERSION - 1)].to_vec();
    p.resize(HELLO_VERSION, 0);
    p.extend_from_slice(&OURS.to_le_bytes());
    Packet::new(HELLO, 0, p)
}

/// The peer's version string and capabilities from its `hello`.
fn parse_hello(payload: &[u8]) -> Option<(String, u32)> {
    let version = payload.get(..HELLO_VERSION)?;
    let end = version.iter().position(|&b| b == 0).unwrap_or(HELLO_VERSION);
    let version = String::from_utf8_lossy(&version[..end]).into_owned();
    let caps = payload.get(HELLO_VERSION..HELLO_VERSION + 4).map_or(0, |c| le32(c, 0));
    Some((version, caps))
}

/// The capabilities both sides have.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Caps(u32);

impl Caps {
    fn has(self, bit: u32) -> bool {
        self.0 & (1 << bit) != 0
    }
}

/// What the device looks like to the guest: the endpoints and
/// interfaces of its configuration, and the device itself.
fn ep_info(desc: &Descriptors, configuration: u8, caps: Caps) -> Packet {
    let mut kind = [TYPE_INVALID; 32];
    let mut interval = [0u8; 32];
    let mut interface = [0u8; 32];
    let mut max_packet = [0u16; 32];
    for slot in [0, 16] {
        kind[slot] = CONTROL;
        max_packet[slot] = u16::from(desc.device[7]);
    }
    let config =
        desc.config_by_value(configuration).or_else(|| desc.configs.first().map(Vec::as_slice));
    if let Some(config) = config {
        let mut current = 0u8;
        for r in descriptor::records(config) {
            match r[1] {
                descriptor::INTERFACE if r.len() >= 9 => current = r[2],
                descriptor::ENDPOINT if r.len() >= 7 => {
                    let address = r[2];
                    let slot =
                        usize::from(address & 0x0f) + if address & 0x80 != 0 { 16 } else { 0 };
                    kind[slot] = r[3] & 3;
                    interval[slot] = r[6];
                    interface[slot] = current;
                    max_packet[slot] = le16(r, 4);
                }
                _ => {}
            }
        }
    }
    let mut p = Vec::with_capacity(32 * 4 + 64 + 128);
    p.extend_from_slice(&kind);
    p.extend_from_slice(&interval);
    p.extend_from_slice(&interface);
    if caps.has(CAP_EP_INFO_MAX_PACKET_SIZE) {
        for m in max_packet {
            p.extend_from_slice(&m.to_le_bytes());
        }
    }
    if caps.has(CAP_BULK_STREAMS) {
        p.extend_from_slice(&[0u8; 128]);
    }
    Packet::new(EP_INFO, 0, p)
}

fn interface_info(desc: &Descriptors, configuration: u8) -> Packet {
    let config =
        desc.config_by_value(configuration).or_else(|| desc.configs.first().map(Vec::as_slice));
    let interfaces: Vec<[u8; 5]> = config
        .map(|c| Descriptors::interfaces(c).into_iter().filter(|i| i[1] == 0).collect())
        .unwrap_or_default();
    let mut p = Vec::with_capacity(4 + 128);
    p.extend_from_slice(&(interfaces.len().min(32) as u32).to_le_bytes());
    for field in 0..4 {
        let mut column = [0u8; 32];
        for (slot, i) in interfaces.iter().take(32).enumerate() {
            column[slot] = [i[0], i[2], i[3], i[4]][field];
        }
        p.extend_from_slice(&column);
    }
    Packet::new(INTERFACE_INFO, 0, p)
}

fn device_connect(desc: &Descriptors, caps: Caps) -> Packet {
    let class = desc.device_class();
    let mut p = vec![speed_code(desc.speed), class[0], class[1], class[2]];
    p.extend_from_slice(&desc.vendor_id().to_le_bytes());
    p.extend_from_slice(&desc.product_id().to_le_bytes());
    if caps.has(CAP_CONNECT_DEVICE_VERSION) {
        p.extend_from_slice(&desc.bcd_device().to_le_bytes());
    }
    Packet::new(DEVICE_CONNECT, 0, p)
}

/// A data packet the bus is working on, by our sequence number.
#[derive(Clone, Debug)]
enum Pending {
    /// A control transfer: the header to echo.
    Control { id: u64, header: [u8; 10] },
    /// A bulk transfer: the guest's id, endpoint address, stream id.
    Bulk { id: u64, endpoint: u8, stream: u32 },
    /// An interrupt OUT transfer.
    InterruptOut { id: u64, endpoint: u8 },
    /// Our own read of an interrupt IN endpoint being received.
    Receive { endpoint: u8 },
}

/// An isochronous IN stream: the server offers the device a packet per
/// service interval and sends it on.
#[derive(Clone, Copy, Debug)]
struct IsoStream {
    /// The id of the next packet.
    next: u64,
    started: Instant,
    /// Packets offered since the start.
    offered: u64,
    /// The service interval.
    period: Duration,
    /// `wMaxPacketSize`: what one packet holds.
    max: usize,
}

impl IsoStream {
    /// Packets offered at once at most, after a stall of the loop: a
    /// stream that fell behind does not flood the guest to catch up.
    const BURST: u32 = 64;

    /// A stream on this endpoint of a device of this speed.
    fn new(speed: Speed, endpoint: Endpoint) -> IsoStream {
        IsoStream {
            next: 0,
            started: Instant::now(),
            offered: 0,
            period: endpoint.iso_interval(speed),
            // The low 11 bits; the rest say how many per microframe.
            max: usize::from(endpoint.max_packet & 0x07ff),
        }
    }

    /// How many packets are due now; they count as offered.
    fn due(&mut self) -> u32 {
        let elapsed = self.started.elapsed().as_micros() / self.period.as_micros().max(1);
        let behind = u64::try_from(elapsed).unwrap_or(u64::MAX).saturating_sub(self.offered);
        // What a long stall left behind is dropped, not sent late.
        self.offered += behind;
        u32::try_from(behind).unwrap_or(u32::MAX).min(Self::BURST)
    }
}

/// Something the driver of the run asks the server to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Command {
    /// The device leaves: `device_disconnect`.
    Disconnect,
    /// The device comes (back): a reset, then it is announced.
    Connect,
}

/// What the server tells about the connection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Link {
    /// A guest connected and the device was announced to it.
    Connected,
    /// The guest went away.
    Gone,
}

/// Drives a running server from elsewhere.
#[derive(Clone, Debug)]
pub struct Handle {
    commands: mpsc::Sender<Command>,
    connected: Arc<AtomicBool>,
}

impl Handle {
    /// The device leaves the bus, as a pulled cable does, until
    /// [`Handle::connect`]. The bus itself is the caller's to mark
    /// ([`Bus::unplug`]).
    pub fn disconnect(&self) {
        let _ = self.commands.send(Command::Disconnect);
    }

    /// The device comes back: the bus is reset (the new attach) and the
    /// device announced with the descriptors it has now.
    pub fn connect(&self) {
        let _ = self.commands.send(Command::Connect);
    }

    /// Whether a guest is connected and has the device.
    pub fn connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }
}

/// The server: one device, one guest at a time. The bus is shared with
/// whoever drives the run.
pub struct Server {
    bus: Arc<Mutex<Bus>>,
    listener: Listener,
    commands: mpsc::Receiver<Command>,
    handle: Handle,
    signals: Option<Box<dyn Fn(Signal) + Send>>,
    links: Option<Box<dyn Fn(Link) + Send>>,
}

impl std::fmt::Debug for Server {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Server").field("listener", &self.listener).finish_non_exhaustive()
    }
}

impl Server {
    /// Binds where the guest connects: a Unix socket path or `host:port`,
    /// what QEMU's `-chardev socket` takes.
    ///
    /// # Errors
    ///
    /// When binding fails, see [`Listener::bind`].
    pub fn bind(
        bus: Arc<Mutex<Bus>>,
        address: &Address,
        options: &ListenOptions,
    ) -> Result<Server> {
        let listener = Listener::bind(address, options)?;
        let (tx, rx) = mpsc::channel();
        let handle = Handle { commands: tx, connected: Arc::new(AtomicBool::new(false)) };
        Ok(Server { bus, listener, commands: rx, handle, signals: None, links: None })
    }

    /// Where it listens; a TCP port 0 is resolved.
    pub fn address(&self) -> Result<Address> {
        Ok(self.listener.address()?)
    }

    /// Drives the server from elsewhere.
    pub fn handle(&self) -> Handle {
        self.handle.clone()
    }

    /// What is called with each [`Signal`] of the bus, after the server
    /// acted on it: a renumeration (the device was disconnected and
    /// announced again), an armed event that fired. It runs on the
    /// server's thread and must not block.
    #[must_use]
    pub fn on_signal(mut self, f: impl Fn(Signal) + Send + 'static) -> Server {
        self.signals = Some(Box::new(f));
        self
    }

    /// What is called when a guest gets the device, and when it goes.
    #[must_use]
    pub fn on_link(mut self, f: impl Fn(Link) + Send + 'static) -> Server {
        self.links = Some(Box::new(f));
        self
    }

    fn bus(&self) -> MutexGuard<'_, Bus> {
        self.bus.lock().expect("bus lock")
    }

    fn comment(&self, message: &str) {
        self.bus().comment(message);
    }

    fn link(&self, link: Link) {
        self.handle.connected.store(link == Link::Connected, Ordering::Relaxed);
        if let Some(f) = &self.links {
            f(link);
        }
    }

    /// Accepts guests until `stop` is set, one at a time.
    ///
    /// # Errors
    ///
    /// When accepting fails.
    pub fn run(mut self, stop: &AtomicBool) -> Result<()> {
        while !stop.load(Ordering::Relaxed) {
            match self.listener.accept() {
                Ok(Some(accepted)) => {
                    self.comment("usbredir guest connected");
                    match self.guest(accepted, stop) {
                        Ok(()) => self.comment("usbredir guest gone"),
                        Err(e) => self.comment(&format!("usbredir guest gone: {e}")),
                    }
                    self.link(Link::Gone);
                }
                Ok(None) => thread::sleep(TICK),
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }

    /// One guest, from its hello to the end of the connection.
    fn guest(&mut self, (reader, writer): Accepted, stop: &AtomicBool) -> Result<()> {
        let (tx, rx) = mpsc::channel();
        let bus = Arc::clone(&self.bus);
        thread::spawn(move || {
            let mut reader: Reader = reader;
            // The hello has a 32 bit id whatever follows.
            let mut wide = false;
            let mut first = true;
            loop {
                match Packet::read_from(&mut reader, wide) {
                    Ok(p) => {
                        if std::mem::take(&mut first) {
                            wide = wide_after(OURS, &p);
                        }
                        if tx.send(p).is_err() {
                            return;
                        }
                    }
                    Err(e) => {
                        if e.kind() != io::ErrorKind::UnexpectedEof {
                            bus.lock().expect("bus lock").comment(&format!("reader stopped: {e}"));
                        }
                        return;
                    }
                }
            }
        });
        let mut conn = Connection {
            server: self,
            out: writer,
            caps: Caps::default(),
            peer: None,
            pending: HashMap::new(),
            next: 1,
            receiving: HashMap::new(),
            streams: HashMap::new(),
            present: false,
            awaiting_ack: None,
        };
        hello().write_to(&mut conn.out, false)?;
        loop {
            if stop.load(Ordering::Relaxed) {
                return Ok(());
            }
            match rx.recv_timeout(TICK) {
                Ok(packet) => conn.packet(packet)?,
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return Ok(()),
            }
            while let Ok(command) = conn.server.commands.try_recv() {
                conn.command(command)?;
            }
            let polled = conn.server.bus().poll();
            conn.deliver(polled)?;
            conn.pump()?;
        }
    }
}

/// One guest's connection.
struct Connection<'a> {
    server: &'a mut Server,
    out: Writer,
    caps: Caps,
    peer: Option<String>,
    pending: HashMap<u32, Pending>,
    next: u32,
    /// Interrupt IN endpoints being received, by address: the id of
    /// the next packet.
    receiving: HashMap<u8, u64>,
    /// Isochronous IN streams, by endpoint address.
    streams: HashMap<u8, IsoStream>,
    /// The guest has the device.
    present: bool,
    /// A disconnect was sent and its ack is awaited; whether the device
    /// is then announced again.
    awaiting_ack: Option<bool>,
}

impl Connection<'_> {
    fn send(&mut self, packet: &Packet) -> io::Result<()> {
        packet.write_to(&mut self.out, self.caps.has(CAP_64BITS_IDS))
    }

    fn seq(&mut self) -> u32 {
        let n = self.next;
        self.next = self.next.wrapping_add(1).max(1);
        n
    }

    /// Announces the device: `ep_info`, `interface_info`, `device_connect`.
    fn announce(&mut self) -> io::Result<()> {
        let (ep, iface, dev) = {
            let bus = self.server.bus();
            let device = bus.device();
            let desc = device.descriptors();
            let configuration = device.configuration();
            (
                ep_info(desc, configuration, self.caps),
                interface_info(desc, configuration),
                device_connect(desc, self.caps),
            )
        };
        // The link is marked before the guest hears of it: whoever
        // sees the device announced finds the handle saying so. A send
        // that fails ends the connection, which marks it gone.
        self.present = true;
        self.server.link(Link::Connected);
        self.send(&ep)?;
        self.send(&iface)?;
        self.send(&dev)
    }

    /// `ep_info` and `interface_info` again, after a configuration or
    /// alternate setting changed.
    fn info(&mut self) -> io::Result<()> {
        let (ep, iface) = {
            let bus = self.server.bus();
            let device = bus.device();
            let desc = device.descriptors();
            let configuration = device.configuration();
            (ep_info(desc, configuration, self.caps), interface_info(desc, configuration))
        };
        self.send(&ep)?;
        self.send(&iface)
    }

    /// The device leaves the guest; `again` announces it anew once the
    /// guest acknowledged (or at once, without the capability).
    fn disconnect(&mut self, again: bool) -> io::Result<()> {
        if !self.present {
            if again {
                return self.reconnect();
            }
            return Ok(());
        }
        self.present = false;
        self.pending.clear();
        self.receiving.clear();
        self.streams.clear();
        self.server.link(Link::Gone);
        self.send(&Packet::new(DEVICE_DISCONNECT, 0, Vec::new()))?;
        if self.caps.has(CAP_DEVICE_DISCONNECT_ACK) {
            self.awaiting_ack = Some(again);
            Ok(())
        } else if again {
            self.reconnect()
        } else {
            Ok(())
        }
    }

    /// The new attach: the bus reset, the device announced.
    fn reconnect(&mut self) -> io::Result<()> {
        self.server.bus().reset();
        self.pending.clear();
        self.receiving.clear();
        self.streams.clear();
        self.announce()
    }

    fn command(&mut self, command: Command) -> io::Result<()> {
        match command {
            Command::Disconnect => self.disconnect(false),
            Command::Connect if self.present => Ok(()),
            Command::Connect if self.awaiting_ack.is_some() => {
                self.awaiting_ack = Some(true);
                Ok(())
            }
            Command::Connect => self.reconnect(),
        }
    }

    #[expect(
        clippy::too_many_lines,
        reason = "one dispatch over the packet types, each next to its reply"
    )]
    fn packet(&mut self, packet: Packet) -> Result<()> {
        if self.peer.is_none() {
            let Some((version, caps)) =
                (packet.kind == HELLO).then(|| parse_hello(&packet.payload)).flatten()
            else {
                return Err(Error::Protocol(format!(
                    "the guest sent packet type {} before its hello",
                    packet.kind
                )));
            };
            self.caps = Caps(caps & OURS);
            self.server.comment(&format!("usbredir guest: {version}, capabilities {caps:#x}"));
            self.peer = Some(version);
            if !self.server.bus().unplugged() {
                self.announce()?;
            }
            return Ok(());
        }
        let Packet { kind, id, payload: p } = packet;
        match kind {
            HELLO => self.server.comment("a second hello from the guest, ignored"),
            RESET => {
                self.server.comment("usbredir reset");
                self.server.bus().reset();
                self.pending.clear();
                self.receiving.clear();
                self.streams.clear();
            }
            SET_CONFIGURATION => {
                let value = p.first().copied().unwrap_or(0);
                let setup = Setup {
                    request_type: 0x00,
                    request: 9,
                    value: u16::from(value),
                    index: 0,
                    length: 0,
                };
                let status = status_of(self.server.bus().control(setup, &[]).status);
                if status == SUCCESS {
                    self.info()?;
                }
                let configuration = self.server.bus().device().configuration();
                self.send(&Packet::new(CONFIGURATION_STATUS, id, vec![status, configuration]))?;
            }
            GET_CONFIGURATION => {
                let configuration = self.server.bus().device().configuration();
                self.send(&Packet::new(CONFIGURATION_STATUS, id, vec![SUCCESS, configuration]))?;
            }
            SET_ALT_SETTING => {
                let (interface, alt) =
                    (p.first().copied().unwrap_or(0), p.get(1).copied().unwrap_or(0));
                let setup = Setup {
                    request_type: 0x01,
                    request: 11,
                    value: u16::from(alt),
                    index: u16::from(interface),
                    length: 0,
                };
                let status = status_of(self.server.bus().control(setup, &[]).status);
                if status == SUCCESS {
                    self.info()?;
                }
                self.send(&Packet::new(ALT_SETTING_STATUS, id, vec![status, interface, alt]))?;
            }
            GET_ALT_SETTING => {
                let interface = p.first().copied().unwrap_or(0);
                let setup = Setup {
                    request_type: 0x81,
                    request: 10,
                    value: 0,
                    index: u16::from(interface),
                    length: 1,
                };
                let c = self.server.bus().control(setup, &[]);
                let alt = c.data.first().copied().unwrap_or(0);
                let status = status_of(c.status);
                self.send(&Packet::new(ALT_SETTING_STATUS, id, vec![status, interface, alt]))?;
            }
            START_ISO_STREAM => {
                let endpoint = p.first().copied().unwrap_or(0);
                let found = self.server.bus().device().endpoint(endpoint);
                let speed = self.server.bus().device().descriptors().speed;
                let status = match found {
                    Some(e) if e.transfer_type() == ISOCHRONOUS => {
                        if endpoint & 0x80 != 0 {
                            self.streams.insert(endpoint, IsoStream::new(speed, e));
                        }
                        SUCCESS
                    }
                    _ => INVAL,
                };
                self.send(&Packet::new(ISO_STREAM_STATUS, id, vec![status, endpoint]))?;
            }
            STOP_ISO_STREAM => {
                let endpoint = p.first().copied().unwrap_or(0);
                self.streams.remove(&endpoint);
                self.send(&Packet::new(ISO_STREAM_STATUS, id, vec![SUCCESS, endpoint]))?;
            }
            START_INTERRUPT_RECEIVING => {
                let endpoint = p.first().copied().unwrap_or(0);
                let interrupt_in = endpoint & 0x80 != 0
                    && self
                        .server
                        .bus()
                        .device()
                        .endpoint(endpoint)
                        .is_some_and(|e| e.transfer_type() == INTERRUPT);
                let status = if interrupt_in {
                    self.receiving.entry(endpoint).or_insert(0);
                    self.receive(endpoint)?;
                    SUCCESS
                } else {
                    INVAL
                };
                self.send(&Packet::new(INTERRUPT_RECEIVING_STATUS, id, vec![status, endpoint]))?;
            }
            STOP_INTERRUPT_RECEIVING => {
                let endpoint = p.first().copied().unwrap_or(0);
                self.receiving.remove(&endpoint);
                let ours: Vec<u32> = self
                    .pending
                    .iter()
                    .filter(
                        |(_, v)| matches!(v, Pending::Receive { endpoint: e } if *e == endpoint),
                    )
                    .map(|(k, _)| *k)
                    .collect();
                for seq in ours {
                    self.pending.remove(&seq);
                    // What the device held for it goes nowhere: the
                    // guest stopped listening.
                    let _ = self.server.bus().unlink(seq);
                }
                self.send(&Packet::new(INTERRUPT_RECEIVING_STATUS, id, vec![SUCCESS, endpoint]))?;
            }
            ALLOC_BULK_STREAMS | FREE_BULK_STREAMS => {
                let mut r = p.get(..8).map_or_else(|| vec![0; 8], <[u8]>::to_vec);
                r.push(INVAL);
                self.send(&Packet::new(BULK_STREAMS_STATUS, id, r))?;
            }
            START_BULK_RECEIVING | STOP_BULK_RECEIVING => {
                let stream = p.get(..4).map_or(0, |s| le32(s, 0));
                let endpoint = p.get(8).copied().unwrap_or(0);
                let mut r = stream.to_le_bytes().to_vec();
                r.extend_from_slice(&[endpoint, INVAL]);
                self.send(&Packet::new(BULK_RECEIVING_STATUS, id, r))?;
            }
            CANCEL_DATA_PACKET => self.cancel(id)?,
            DEVICE_DISCONNECT_ACK => {
                if let Some(again) = self.awaiting_ack.take()
                    && again
                {
                    self.reconnect()?;
                }
            }
            FILTER_REJECT => self.server.comment("the guest rejected the device with a filter"),
            FILTER_FILTER => {}
            CONTROL_PACKET => {
                if p.len() < 10 {
                    return Err(Error::Protocol("a control packet shorter than its header".into()));
                }
                let header: [u8; 10] = p[..10].try_into().expect("ten bytes");
                let setup = Setup {
                    request_type: header[2],
                    request: header[1],
                    value: le16(&header, 4),
                    index: le16(&header, 6),
                    length: le16(&header, 8),
                };
                let data = if setup.is_in() { Vec::new() } else { p[10..].to_vec() };
                let seq = self.seq();
                self.pending.insert(seq, Pending::Control { id, header });
                let step = self.server.bus().submit(Urb::control(seq, setup, data));
                self.deliver(step)?;
            }
            BULK_PACKET => {
                let long = self.caps.has(CAP_32BITS_BULK_LENGTH);
                let head = if long { 10 } else { 8 };
                if p.len() < head {
                    return Err(Error::Protocol("a bulk packet shorter than its header".into()));
                }
                let endpoint = p[0];
                let mut length = usize::from(le16(&p, 2));
                let stream = le32(&p, 4);
                if long {
                    length |= usize::from(le16(&p, 8)) << 16;
                }
                let seq = self.seq();
                self.pending.insert(seq, Pending::Bulk { id, endpoint, stream });
                let urb = if endpoint & 0x80 != 0 {
                    Urb::read(seq, endpoint & 0x0f, length)
                } else {
                    Urb::write(seq, endpoint & 0x0f, p[head..].to_vec())
                };
                let step = self.server.bus().submit(urb);
                self.deliver(step)?;
            }
            INTERRUPT_PACKET => {
                if p.len() < 4 {
                    return Err(Error::Protocol(
                        "an interrupt packet shorter than its header".into(),
                    ));
                }
                let endpoint = p[0];
                if endpoint & 0x80 != 0 {
                    // Interrupt IN is received, not requested.
                    self.send(&Packet::new(INTERRUPT_PACKET, id, vec![endpoint, INVAL, 0, 0]))?;
                } else {
                    let seq = self.seq();
                    self.pending.insert(seq, Pending::InterruptOut { id, endpoint });
                    let step =
                        self.server.bus().submit(Urb::write(seq, endpoint & 0x0f, p[4..].to_vec()));
                    self.deliver(step)?;
                }
            }
            ISO_PACKET => {
                // An OUT stream's packet: one service interval's data.
                // Nothing is answered.
                if p.len() < 4 {
                    return Err(Error::Protocol("an iso packet shorter than its header".into()));
                }
                let endpoint = p[0];
                if endpoint & 0x80 == 0 {
                    let seq = self.seq();
                    let urb = Urb::iso_write(seq, endpoint & 0x0f, &[&p[4..]]);
                    // The guest kept its time.
                    let step = self.server.bus().submit_timed(urb);
                    self.deliver(step)?;
                }
            }
            other => self.server.comment(&format!("usbredir packet type {other} ignored")),
        }
        Ok(())
    }

    /// Offers the device the packets that are due on every isochronous
    /// IN stream, and sends each to the guest.
    fn pump(&mut self) -> io::Result<()> {
        let due: Vec<(u8, u32, usize)> =
            self.streams.iter_mut().map(|(ep, s)| (*ep, s.due(), s.max)).collect();
        for (endpoint, count, max) in due {
            for _ in 0..count {
                let seq = self.seq();
                let urb = Urb::iso_read(seq, endpoint & 0x0f, vec![max]);
                let step = self.server.bus().submit_timed(urb);
                let sent = step.completions.iter().find(|c| c.seqnum == seq).cloned();
                if let Some(c) = sent
                    && let (Some(packet), Some(stream)) =
                        (c.iso.first(), self.streams.get_mut(&endpoint))
                {
                    let id = stream.next;
                    stream.next = stream.next.wrapping_add(1);
                    let mut payload = vec![endpoint, status_of(packet.status)];
                    payload.extend_from_slice(&(c.data.len() as u16).to_le_bytes());
                    payload.extend_from_slice(&c.data);
                    self.send(&Packet::new(ISO_PACKET, id, payload))?;
                }
                self.deliver(step)?;
                if !self.present {
                    return Ok(());
                }
            }
        }
        Ok(())
    }

    /// Submits our next read of an interrupt IN endpoint being received.
    fn receive(&mut self, endpoint: u8) -> io::Result<()> {
        let max =
            self.server.bus().device().endpoint(endpoint).map_or(64, |e| usize::from(e.max_packet));
        let seq = self.seq();
        self.pending.insert(seq, Pending::Receive { endpoint });
        let step = self.server.bus().submit(Urb::read(seq, endpoint & 0x0f, max));
        self.deliver(step)
    }

    /// The guest cancels its packet `id`: it gets it back, cancelled, or
    /// with what the device held for it.
    fn cancel(&mut self, id: u64) -> io::Result<()> {
        let ours = self.pending.iter().find_map(|(seq, p)| match p {
            Pending::Control { id: i, .. }
            | Pending::Bulk { id: i, .. }
            | Pending::InterruptOut { id: i, .. }
                if *i == id =>
            {
                Some(*seq)
            }
            _ => None,
        });
        let Some(seq) = ours else {
            self.server.comment(&format!("cancel of packet {id}, which completed already"));
            return Ok(());
        };
        let outcome = self.server.bus().unlink(seq);
        match outcome {
            Unlink::NotHeld => Ok(()),
            Unlink::Cancelled => {
                let c = Completion {
                    seqnum: seq,
                    status: Status::Unlinked,
                    actual: 0,
                    data: Vec::new(),
                    iso: Vec::new(),
                };
                self.complete(&c)
            }
            Unlink::Completed(c) => self.complete(&c),
        }
    }

    /// Answers the completions of a step and acts on its signals.
    fn deliver(&mut self, step: Step) -> io::Result<()> {
        for c in &step.completions {
            self.complete(c)?;
        }
        for signal in step.signals {
            match &signal {
                Signal::Renumeration(_) => self.disconnect(true)?,
                Signal::Fired(Event::Unplug) => self.disconnect(false)?,
                Signal::Fired(_) => {}
            }
            if let Some(f) = &self.server.signals {
                f(signal);
            }
        }
        Ok(())
    }

    /// One completed transfer back to the guest.
    fn complete(&mut self, c: &Completion) -> io::Result<()> {
        let Some(pending) = self.pending.remove(&c.seqnum) else {
            // Dropped by a reset or a disconnect; the guest knows.
            return Ok(());
        };
        let status = status_of(c.status);
        match pending {
            Pending::Control { id, mut header } => {
                header[3] = status;
                let is_in = header[2] & 0x80 != 0;
                let length = if is_in { c.data.len() } else { c.actual };
                header[8..10].copy_from_slice(&(length as u16).to_le_bytes());
                let mut payload = header.to_vec();
                if is_in {
                    payload.extend_from_slice(&c.data);
                }
                self.send(&Packet::new(CONTROL_PACKET, id, payload))
            }
            Pending::Bulk { id, endpoint, stream } => {
                let is_in = endpoint & 0x80 != 0;
                let length = if is_in { c.data.len() } else { c.actual };
                let mut payload = vec![endpoint, status];
                payload.extend_from_slice(&(length as u16).to_le_bytes());
                payload.extend_from_slice(&stream.to_le_bytes());
                if self.caps.has(CAP_32BITS_BULK_LENGTH) {
                    payload.extend_from_slice(&((length >> 16) as u16).to_le_bytes());
                }
                if is_in {
                    payload.extend_from_slice(&c.data);
                }
                self.send(&Packet::new(BULK_PACKET, id, payload))
            }
            Pending::InterruptOut { id, endpoint } => {
                let mut payload = vec![endpoint, status];
                payload.extend_from_slice(&(c.actual as u16).to_le_bytes());
                self.send(&Packet::new(INTERRUPT_PACKET, id, payload))
            }
            Pending::Receive { endpoint } => {
                let Some(next) = self.receiving.get_mut(&endpoint) else { return Ok(()) };
                let id = *next;
                *next = next.wrapping_add(1);
                match c.status {
                    Status::Stall => {
                        // The stream stopped for a reason other than a
                        // stop: the guest learns it by a STALL status.
                        self.receiving.remove(&endpoint);
                        self.send(&Packet::new(
                            INTERRUPT_RECEIVING_STATUS,
                            0,
                            vec![STALL, endpoint],
                        ))
                    }
                    // Missed is an isochronous packet's status only.
                    Status::Unlinked | Status::Missed => Ok(()),
                    Status::Ok | Status::Babble => {
                        let mut payload = vec![endpoint, status];
                        payload.extend_from_slice(&(c.data.len() as u16).to_le_bytes());
                        payload.extend_from_slice(&c.data);
                        self.send(&Packet::new(INTERRUPT_PACKET, id, payload))?;
                        self.receive(endpoint)
                    }
                }
            }
        }
    }
}

/// A usb-guest, for tests and tools: what QEMU does, in the small.
pub struct Guest {
    reader: Reader,
    writer: Writer,
    /// The capabilities both sides have.
    pub caps: u32,
    /// Ids are 64 bits on the wire: both sides offered it.
    wide: bool,
    /// The host's version string.
    pub host: String,
    /// Packets taken that were not the reply asked for: `ep_info`,
    /// `interface_info`, `device_connect`, interrupt packets.
    pub unsolicited: Vec<Packet>,
}

impl std::fmt::Debug for Guest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Guest")
            .field("caps", &self.caps)
            .field("host", &self.host)
            .finish_non_exhaustive()
    }
}

impl Guest {
    /// Connects, exchanges hellos with these capability bits.
    ///
    /// # Errors
    ///
    /// When nothing listens, or the host's first packet is not a hello.
    pub fn connect(address: &Address, caps: u32) -> Result<Guest> {
        let (reader, writer): Accepted = match address {
            Address::Tcp(a) => {
                let s = std::net::TcpStream::connect(a)?;
                s.set_nodelay(true)?;
                (Box::new(s.try_clone()?), Box::new(s))
            }
            #[cfg(unix)]
            Address::Unix(p) => {
                let s = std::os::unix::net::UnixStream::connect(p)?;
                (Box::new(s.try_clone()?), Box::new(s))
            }
            #[cfg(not(unix))]
            Address::Unix(p) => {
                return Err(Error::Invalid(format!("{}: Unix sockets need Unix", p.display())));
            }
        };
        let mut guest = Guest {
            reader,
            writer,
            caps: 0,
            wide: false,
            host: String::new(),
            unsolicited: Vec::new(),
        };
        // Both hellos have 32 bit ids; the capabilities apply after.
        let mut p = b"vusb test guest".to_vec();
        p.resize(HELLO_VERSION, 0);
        p.extend_from_slice(&caps.to_le_bytes());
        guest.send(&Packet::new(HELLO, 0, p))?;
        let hello = guest.read()?;
        let Some((host, theirs)) =
            (hello.kind == HELLO).then(|| parse_hello(&hello.payload)).flatten()
        else {
            return Err(Error::Protocol("the host's first packet is not a hello".into()));
        };
        guest.caps = caps & theirs;
        guest.host = host;
        guest.wide = guest.caps & (1 << CAP_64BITS_IDS) != 0;
        Ok(guest)
    }

    /// Sends a packet.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn send(&mut self, packet: &Packet) -> Result<()> {
        Ok(packet.write_to(&mut self.writer, self.wide)?)
    }

    /// Reads the next packet, whatever it is.
    ///
    /// # Errors
    ///
    /// When the connection fails or ends.
    pub fn read(&mut self) -> Result<Packet> {
        Ok(Packet::read_from(&mut self.reader, self.wide)?)
    }

    /// Reads until the reply to `id` of type `kind`; what came before is
    /// kept in [`Guest::unsolicited`].
    ///
    /// # Errors
    ///
    /// When the connection fails or ends.
    pub fn reply(&mut self, kind: u32, id: u64) -> Result<Packet> {
        loop {
            let p = self.read()?;
            if p.kind == kind && p.id == id {
                return Ok(p);
            }
            self.unsolicited.push(p);
        }
    }

    /// Waits for the device: `ep_info`, `interface_info` and
    /// `device_connect`, in that order; returns the connect packet's
    /// payload, and leaves the two info packets in [`Guest::unsolicited`].
    ///
    /// # Errors
    ///
    /// When they come in another order, or the connection ends.
    pub fn device(&mut self) -> Result<Vec<u8>> {
        let mut want = [EP_INFO, INTERFACE_INFO, DEVICE_CONNECT].into_iter();
        let mut expected = want.next();
        loop {
            let p = self.read()?;
            if Some(p.kind) == expected {
                expected = want.next();
                if expected.is_none() {
                    return Ok(p.payload);
                }
                // The info packets stay readable after the connect.
                self.unsolicited.push(p);
            } else if [EP_INFO, INTERFACE_INFO, DEVICE_CONNECT].contains(&p.kind) {
                return Err(Error::Protocol(format!(
                    "packet type {} where {expected:?} was due",
                    p.kind
                )));
            } else {
                self.unsolicited.push(p);
            }
        }
    }

    /// A control transfer: the status and the data.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn control(&mut self, id: u64, setup: Setup, data: &[u8]) -> Result<(u8, Vec<u8>)> {
        let mut p =
            vec![if setup.is_in() { 0x80 } else { 0 }, setup.request, setup.request_type, 0];
        p.extend_from_slice(&setup.value.to_le_bytes());
        p.extend_from_slice(&setup.index.to_le_bytes());
        p.extend_from_slice(&setup.length.to_le_bytes());
        if !setup.is_in() {
            p.extend_from_slice(data);
        }
        self.send(&Packet::new(CONTROL_PACKET, id, p))?;
        let r = self.reply(CONTROL_PACKET, id)?;
        Ok((r.payload[3], r.payload[10..].to_vec()))
    }

    /// A bulk transfer on endpoint address `endpoint`: `length` bytes to
    /// read, or `data` to write. The status and the data.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn bulk(
        &mut self,
        id: u64,
        endpoint: u8,
        length: usize,
        data: &[u8],
    ) -> Result<(u8, Vec<u8>)> {
        let long = self.caps & (1 << CAP_32BITS_BULK_LENGTH) != 0;
        let mut p = vec![endpoint, 0];
        p.extend_from_slice(&(length as u16).to_le_bytes());
        p.extend_from_slice(&0u32.to_le_bytes());
        if long {
            p.extend_from_slice(&((length >> 16) as u16).to_le_bytes());
        }
        if endpoint & 0x80 == 0 {
            p.extend_from_slice(data);
        }
        self.send(&Packet::new(BULK_PACKET, id, p))?;
        let r = self.reply(BULK_PACKET, id)?;
        let head = if long { 10 } else { 8 };
        Ok((r.payload[1], r.payload[head..].to_vec()))
    }

    /// `set_configuration`: the status and the configuration, after the
    /// info packets that precede it.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn set_configuration(&mut self, id: u64, configuration: u8) -> Result<(u8, u8)> {
        self.send(&Packet::new(SET_CONFIGURATION, id, vec![configuration]))?;
        let r = self.reply(CONFIGURATION_STATUS, id)?;
        Ok((r.payload[0], r.payload[1]))
    }

    /// `start_interrupt_receiving`: the status.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn start_interrupt(&mut self, id: u64, endpoint: u8) -> Result<u8> {
        self.send(&Packet::new(START_INTERRUPT_RECEIVING, id, vec![endpoint]))?;
        Ok(self.reply(INTERRUPT_RECEIVING_STATUS, id)?.payload[0])
    }

    /// `stop_interrupt_receiving`: the status.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn stop_interrupt(&mut self, id: u64, endpoint: u8) -> Result<u8> {
        self.send(&Packet::new(STOP_INTERRUPT_RECEIVING, id, vec![endpoint]))?;
        Ok(self.reply(INTERRUPT_RECEIVING_STATUS, id)?.payload[0])
    }

    /// `start_iso_stream` on endpoint address `endpoint`: the status.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn start_iso(&mut self, id: u64, endpoint: u8) -> Result<u8> {
        self.send(&Packet::new(START_ISO_STREAM, id, vec![endpoint, 8, 4]))?;
        Ok(self.reply(ISO_STREAM_STATUS, id)?.payload[0])
    }

    /// `stop_iso_stream`: the status.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn stop_iso(&mut self, id: u64, endpoint: u8) -> Result<u8> {
        self.send(&Packet::new(STOP_ISO_STREAM, id, vec![endpoint]))?;
        Ok(self.reply(ISO_STREAM_STATUS, id)?.payload[0])
    }

    /// One packet of an isochronous OUT stream. Nothing comes back.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn iso_out(&mut self, id: u64, endpoint: u8, data: &[u8]) -> Result<()> {
        let mut p = vec![endpoint, 0];
        p.extend_from_slice(&(data.len() as u16).to_le_bytes());
        p.extend_from_slice(data);
        self.send(&Packet::new(ISO_PACKET, id, p))
    }

    /// Cancels data packet `id`; the packet comes back through
    /// [`Guest::reply`].
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn cancel(&mut self, id: u64) -> Result<()> {
        self.send(&Packet::new(CANCEL_DATA_PACKET, id, Vec::new()))
    }

    /// Acknowledges a disconnect.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn disconnect_ack(&mut self) -> Result<()> {
        self.send(&Packet::new(DEVICE_DISCONNECT_ACK, 0, Vec::new()))
    }
}

/// The packet types a test tells apart.
pub mod kinds {
    /// `usb_redir_device_disconnect`.
    pub const DEVICE_DISCONNECT: u32 = super::DEVICE_DISCONNECT;
    /// `usb_redir_device_connect`.
    pub const DEVICE_CONNECT: u32 = super::DEVICE_CONNECT;
    /// `usb_redir_ep_info`.
    pub const EP_INFO: u32 = super::EP_INFO;
    /// `usb_redir_interface_info`.
    pub const INTERFACE_INFO: u32 = super::INTERFACE_INFO;
    /// `usb_redir_interrupt_packet`.
    pub const INTERRUPT_PACKET: u32 = super::INTERRUPT_PACKET;
    /// `usb_redir_interrupt_receiving_status`.
    pub const INTERRUPT_RECEIVING_STATUS: u32 = super::INTERRUPT_RECEIVING_STATUS;
    /// `usb_redir_control_packet`.
    pub const CONTROL_PACKET: u32 = super::CONTROL_PACKET;
    /// `usb_redir_bulk_packet`.
    pub const BULK_PACKET: u32 = super::BULK_PACKET;
    /// `usb_redir_reset`.
    pub const RESET: u32 = super::RESET;
    /// `usb_redir_iso_stream_status`.
    pub const ISO_STREAM_STATUS: u32 = super::ISO_STREAM_STATUS;
    /// `usb_redir_iso_packet`.
    pub const ISO_PACKET: u32 = super::ISO_PACKET;
    /// `usb_redir_start_iso_stream`.
    pub const START_ISO_STREAM: u32 = super::START_ISO_STREAM;
}

/// The status codes a test tells apart.
pub mod statuses {
    /// `usb_redir_success`.
    pub const SUCCESS: u8 = super::SUCCESS;
    /// `usb_redir_cancelled`.
    pub const CANCELLED: u8 = super::CANCELLED;
    /// `usb_redir_inval`.
    pub const INVAL: u8 = super::INVAL;
    /// `usb_redir_ioerror`.
    pub const IOERROR: u8 = super::IOERROR;
    /// `usb_redir_stall`.
    pub const STALL: u8 = super::STALL;
    /// `usb_redir_babble`.
    pub const BABBLE: u8 = super::BABBLE;
}

/// The capability bits a guest may offer.
pub mod caps {
    /// `usb_redir_cap_connect_device_version`.
    pub const CONNECT_DEVICE_VERSION: u32 = 1 << super::CAP_CONNECT_DEVICE_VERSION;
    /// `usb_redir_cap_device_disconnect_ack`.
    pub const DEVICE_DISCONNECT_ACK: u32 = 1 << super::CAP_DEVICE_DISCONNECT_ACK;
    /// `usb_redir_cap_ep_info_max_packet_size`.
    pub const EP_INFO_MAX_PACKET_SIZE: u32 = 1 << super::CAP_EP_INFO_MAX_PACKET_SIZE;
    /// `usb_redir_cap_32bits_bulk_length`.
    pub const BULK_LENGTH_32: u32 = 1 << super::CAP_32BITS_BULK_LENGTH;
    /// `usb_redir_cap_64bits_ids`.
    pub const IDS_64: u32 = 1 << super::CAP_64BITS_IDS;
    /// All a QEMU offers that this server takes.
    pub const QEMU: u32 = CONNECT_DEVICE_VERSION
        | DEVICE_DISCONNECT_ACK
        | EP_INFO_MAX_PACKET_SIZE
        | BULK_LENGTH_32
        | IDS_64;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;
    use crate::examples::relay::UsbRelay;
    use crate::model::DeviceModel;
    use crate::observer::{Recorder, SharedObserver};

    fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index, length }
    }

    struct Served {
        bus: Arc<Mutex<Bus>>,
        rec: Arc<Mutex<Recorder>>,
        handle: Handle,
        address: Address,
        stop: Arc<AtomicBool>,
        thread: thread::JoinHandle<Result<()>>,
    }

    fn serve(model: Box<dyn DeviceModel>) -> Served {
        let rec = Arc::new(Mutex::new(Recorder::default()));
        let shared: SharedObserver = rec.clone();
        let bus = Arc::new(Mutex::new(Bus::new(Device::new(model)).with_observer(shared)));
        let server = Server::bind(
            Arc::clone(&bus),
            &Address::Tcp("127.0.0.1:0".into()),
            &ListenOptions::default(),
        )
        .unwrap();
        let address = server.address().unwrap();
        let handle = server.handle();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let thread = thread::spawn(move || server.run(&stop2));
        Served { bus, rec, handle, address, stop, thread }
    }

    impl Served {
        fn end(self) -> Arc<Mutex<Recorder>> {
            self.stop.store(true, Ordering::Relaxed);
            self.thread.join().unwrap().unwrap();
            self.rec
        }
    }

    #[test]
    fn announces_the_device_and_answers_transfers() {
        let served = serve(Box::new(UsbRelay::new(2)));
        let mut g = Guest::connect(&served.address, caps::QEMU).unwrap();
        assert_eq!(g.caps, caps::QEMU, "all five taken");
        assert!(g.host.starts_with("vusb "));
        let connect = g.device().unwrap();
        // speed full, class 0, 16c0:05df, bcdDevice 1.00
        assert_eq!(connect, [1, 0, 0, 0, 0xc0, 0x16, 0xdf, 0x05, 0x00, 0x01]);
        let ep = &g.unsolicited[0];
        assert_eq!((ep.kind, ep.payload.len()), (EP_INFO, 32 * 3 + 64));
        assert_eq!(ep.payload[0], CONTROL, "endpoint 0 out");
        assert_eq!(ep.payload[16], CONTROL, "endpoint 0 in");
        assert_eq!(ep.payload[17], INTERRUPT, "0x81");
        assert_eq!(ep.payload[32 + 17], 2, "its interval");
        assert_eq!(le16(&ep.payload, 96 + 17 * 2), 64, "its packet size");
        assert_eq!(ep.payload[1], TYPE_INVALID, "no 0x01");
        let iface = &g.unsolicited[1];
        assert_eq!(iface.kind, INTERFACE_INFO);
        assert_eq!(le32(&iface.payload, 0), 1);
        assert_eq!(iface.payload[4 + 32], 3, "HID");

        // Enumeration through control packets, then set_configuration as
        // a message, which brings the info again before its status.
        let (status, d) = g.control(1, setup(0x80, 6, 0x0100, 0, 18), &[]).unwrap();
        assert_eq!((status, d.len()), (SUCCESS, 18));
        // An id that needs its 64 bits comes back whole.
        let big = 0x1_0000_0007;
        let (status, d) = g.control(big, setup(0x80, 6, 0x0100, 0, 18), &[]).unwrap();
        assert_eq!((status, d.len()), (SUCCESS, 18));
        let (status, d) = g.control(2, setup(0x80, 6, 0x0600, 0, 10), &[]).unwrap();
        assert_eq!((status, d.len()), (STALL, 0), "no qualifier at full speed");
        g.unsolicited.clear();
        assert_eq!(g.set_configuration(3, 1).unwrap(), (SUCCESS, 1));
        let kinds: Vec<u32> = g.unsolicited.iter().map(|p| p.kind).collect();
        assert_eq!(kinds, [EP_INFO, INTERFACE_INFO]);
        assert_eq!(g.set_configuration(4, 7).unwrap(), (STALL, 1), "no such configuration");
        // A feature report through the HID interface, written and read.
        let (status, _) =
            g.control(5, setup(0x21, 9, 0x0300, 0, 8), &[0xff, 2, 0, 0, 0, 0, 0, 0]).unwrap();
        assert_eq!(status, SUCCESS);
        let (status, d) = g.control(6, setup(0xa1, 1, 0x0300, 0, 8), &[]).unwrap();
        assert_eq!((status, d.as_slice()), (SUCCESS, &b"ABCDE\0\0\x02"[..]));
        drop(g);
        let rec = served.end();
        let rec = rec.lock().unwrap();
        assert_eq!(rec.violations, ["SET_CONFIGURATION 7: no such configuration"]);
        assert!(rec.notices.iter().any(|n| n.starts_with("usbredir guest: vusb test guest")));
    }

    #[test]
    fn interrupt_in_is_received_and_cancel_answers() {
        use crate::shells::hid::HidReports;
        /// A relay board whose input report is a counter, ready every
        /// other poll.
        struct Ticking(UsbRelay, u8);
        impl DeviceModel for Ticking {
            fn name(&self) -> &'static str {
                "ticking"
            }
            fn descriptors(&self) -> &Descriptors {
                self.0.descriptors()
            }
            fn control(
                &mut self,
                s: Setup,
                d: &[u8],
                v: &mut crate::model::Violations,
            ) -> crate::model::Reply {
                self.0.control(s, d, v)
            }
            fn out(
                &mut self,
                ep: u8,
                d: &[u8],
                v: &mut crate::model::Violations,
            ) -> crate::model::Reply {
                self.0.out(ep, d, v)
            }
            fn in_transfer(
                &mut self,
                _: u8,
                _: usize,
                _: &mut crate::model::Violations,
            ) -> crate::model::Reply {
                self.1 = self.1.wrapping_add(1);
                if self.1.is_multiple_of(2) {
                    crate::model::Reply::Data(vec![self.1])
                } else {
                    crate::model::Reply::Nak
                }
            }
            fn reset(&mut self) {
                self.0.reset();
            }
        }
        let _ = <crate::examples::relay::Board as HidReports>::get_report;
        let served = serve(Box::new(Ticking(UsbRelay::new(2), 0)));
        let mut g = Guest::connect(&served.address, caps::QEMU).unwrap();
        g.device().unwrap();
        assert_eq!(g.set_configuration(1, 1).unwrap(), (SUCCESS, 1));
        assert_eq!(g.start_interrupt(2, 0x01).unwrap(), INVAL, "not an IN endpoint");
        assert_eq!(g.start_interrupt(3, 0x81).unwrap(), SUCCESS);
        // Packets come with their own ids from 0, data 2, 4, 6.
        let mut got = Vec::new();
        while got.len() < 3 {
            let p = g.read().unwrap();
            if p.kind == INTERRUPT_PACKET {
                assert_eq!((p.payload[0], p.payload[1]), (0x81, SUCCESS));
                got.push((p.id, p.payload[4]));
            }
        }
        assert_eq!(got, [(0, 2), (1, 4), (2, 6)]);
        assert_eq!(g.stop_interrupt(4, 0x81).unwrap(), SUCCESS);
        // A bulk read on an endpoint the device lacks: STALL, flagged.
        let (status, _) = g.bulk(5, 0x82, 64, &[]).unwrap();
        assert_eq!(status, STALL);
        // A control transfer the device holds is cancelled.
        let served_bus = Arc::clone(&served.bus);
        drop(served_bus);
        g.cancel(99).unwrap();
        assert_eq!(g.start_iso(6, 0x81).unwrap(), INVAL, "not an isochronous endpoint");
        drop(g);
        let rec = served.end();
        assert_eq!(rec.lock().unwrap().violations.len(), 1, "the missing endpoint");
    }

    #[test]
    fn isochronous_streams_run_at_the_endpoint_interval() {
        use crate::descriptor::{self, DeviceDescriptor};
        use crate::model::{Reply, Violations};
        /// An isochronous IN on 0x83 that sends its interval's number,
        /// nothing in every third; an isochronous OUT on 0x04.
        struct Stream(Descriptors, u8, Arc<Mutex<Vec<Vec<u8>>>>);
        impl DeviceModel for Stream {
            fn name(&self) -> &'static str {
                "stream"
            }
            fn descriptors(&self) -> &Descriptors {
                &self.0
            }
            fn control(&mut self, _: Setup, _: &[u8], _: &mut Violations) -> Reply {
                Reply::ack()
            }
            fn out(&mut self, _: u8, _: &[u8], _: &mut Violations) -> Reply {
                Reply::ack()
            }
            fn in_transfer(&mut self, _: u8, _: usize, _: &mut Violations) -> Reply {
                Reply::Nak
            }
            fn iso_in(&mut self, _: u8, _: usize, _: &mut Violations) -> Option<Vec<u8>> {
                self.1 = self.1.wrapping_add(1);
                (!self.1.is_multiple_of(3)).then(|| vec![self.1; 2])
            }
            fn iso_out(&mut self, _: u8, data: &[u8], _: &mut Violations) {
                self.2.lock().unwrap().push(data.to_vec());
            }
            // The server times the stream itself, so a device that
            // keeps real time is not made to wait twice.
            fn realtime(&self) -> bool {
                true
            }
            fn reset(&mut self) {}
        }
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 2, [0xff, 0, 0], 0));
        body.extend_from_slice(&descriptor::endpoint(0x83, 1, 8, 1));
        body.extend_from_slice(&descriptor::endpoint(0x04, 1, 8, 1));
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: 0x1234,
            product: 0x5678,
            device: 0x0100,
            strings: [0, 0, 0],
            configurations: 1,
        };
        let got = Arc::new(Mutex::new(Vec::new()));
        let descriptors = Descriptors {
            speed: Speed::Full,
            device: device.bytes(),
            configs: vec![descriptor::configuration(1, 0, 0x80, 50, &body)],
            strings: Vec::new(),
        };
        let served = serve(Box::new(Stream(descriptors, 0, Arc::clone(&got))));
        let mut g = Guest::connect(&served.address, caps::QEMU).unwrap();
        g.device().unwrap();
        assert_eq!(g.set_configuration(1, 1).unwrap(), (SUCCESS, 1));
        assert_eq!(g.unsolicited.iter().find(|p| p.kind == EP_INFO).unwrap().payload[16 + 3], 1);
        assert_eq!(g.start_iso(2, 0x83).unwrap(), SUCCESS);
        // One packet per millisecond, each with its own id from 0; the
        // third interval's is an error, as the device sent nothing.
        let mut packets = Vec::new();
        while packets.len() < 6 {
            let p = g.read().unwrap();
            if p.kind == ISO_PACKET {
                packets.push((p.id, p.payload[1], p.payload[4..].to_vec()));
            }
        }
        assert_eq!(packets[0], (0, SUCCESS, vec![1, 1]));
        assert_eq!(packets[1], (1, SUCCESS, vec![2, 2]));
        assert_eq!(packets[2], (2, IOERROR, vec![]));
        assert_eq!(packets[3], (3, SUCCESS, vec![4, 4]));
        assert_eq!(g.stop_iso(3, 0x83).unwrap(), SUCCESS);
        // OUT: packets as the guest sends them, nothing answered.
        assert_eq!(g.start_iso(4, 0x04).unwrap(), SUCCESS);
        g.iso_out(0, 0x04, &[7, 7]).unwrap();
        g.iso_out(1, 0x04, &[8]).unwrap();
        // A control transfer after them: they were taken in order.
        let (status, _) = g.control(5, setup(0x80, 0, 0, 0, 2), &[]).unwrap();
        assert_eq!(status, SUCCESS);
        assert_eq!(*got.lock().unwrap(), [vec![7, 7], vec![8]]);
        drop(g);
        let rec = served.end();
        assert_eq!(rec.lock().unwrap().violations, Vec::<String>::new());
    }

    #[test]
    fn unplug_disconnects_and_a_renumeration_reconnects_after_the_ack() {
        use crate::arm::{Arm, When};
        let served = serve(Box::new(UsbRelay::new(2)));
        let mut g = Guest::connect(&served.address, caps::QEMU).unwrap();
        g.device().unwrap();
        served.bus.lock().unwrap().arm(Arm::new(Event::Unplug, When::In { ep: 0, n: 1 }));
        let (status, _) = g.control(1, setup(0x80, 6, 0x0100, 0, 18), &[]).unwrap();
        assert_eq!(status, SUCCESS);
        let p = g.read().unwrap();
        assert_eq!(p.kind, DEVICE_DISCONNECT);
        assert!(!served.handle.connected());
        g.disconnect_ack().unwrap();
        // Nothing comes: an unplugged device stays away until a replug.
        served.handle.connect();
        let connect = g.device().unwrap();
        assert_eq!(&connect[4..8], &[0xc0, 0x16, 0xdf, 0x05]);
        assert!(served.handle.connected());
        // A device that renumerates: the relay board becomes a 4 relay
        // one with new strings, by a fresh model and a connect.
        served.bus.lock().unwrap().replace_model(Box::new(UsbRelay::new(4)));
        served.handle.disconnect();
        assert_eq!(g.read().unwrap().kind, DEVICE_DISCONNECT);
        served.handle.connect();
        // Before the ack the device is not announced.
        g.disconnect_ack().unwrap();
        g.device().unwrap();
        let (_, d) = g.control(2, setup(0x80, 6, 0x0302, 0x0409, 255), &[]).unwrap();
        let name: Vec<u16> = d[2..].chunks(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        assert_eq!(String::from_utf16_lossy(&name), "USBRelay4");
        drop(g);
        served.end();
    }

    #[test]
    fn without_the_capabilities_the_short_forms_are_used() {
        let served = serve(Box::new(UsbRelay::new(2)));
        let mut g = Guest::connect(&served.address, 0).unwrap();
        assert_eq!(g.caps, 0);
        let connect = g.device().unwrap();
        assert_eq!(connect.len(), 8, "no bcdDevice");
        assert_eq!(g.unsolicited[0].payload.len(), 32 * 3, "no max_packet_size");
        g.unsolicited.clear();
        assert_eq!(g.set_configuration(1, 1).unwrap(), (SUCCESS, 1));
        let (status, d) = g.control(2, setup(0xa1, 1, 0x0300, 0, 8), &[]).unwrap();
        assert_eq!((status, d.len()), (SUCCESS, 8));
        // A disconnect without the ack capability reconnects at once.
        served.bus.lock().unwrap().replace_model(Box::new(UsbRelay::new(3)));
        served.handle.disconnect();
        served.handle.connect();
        assert_eq!(g.read().unwrap().kind, DEVICE_DISCONNECT);
        g.device().unwrap();
        drop(g);
        served.end();
    }
}
