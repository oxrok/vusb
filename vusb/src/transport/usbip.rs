// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The USB/IP server: protocol version 1.1.1 as the Linux `vhci-hcd`
//! client speaks it. One TCP connection carries one device:
//! `OP_REQ_DEVLIST` lists it, `OP_REQ_IMPORT` claims it, then the
//! client submits URBs with `CMD_SUBMIT`, which the server answers
//! with `RET_SUBMIT`, and cancels them with `CMD_UNLINK`, answered
//! with `RET_UNLINK`. All header fields are big endian; a setup packet
//! is carried as its eight bytes.
//!
//! The server is a codec: it turns PDUs into [`Bus`] submissions and
//! unlinks, and the bus's completions into PDUs. What a host controller
//! does (holding a transfer answered NAK and offering it again after every
//! other transfer and every 20 ms, keeping each endpoint's order,
//! cutting a babble) is the bus's. So is the time of an isochronous
//! stream: the URBs of a device that keeps real time
//! ([`crate::model::DeviceModel::realtime`]) are answered when the bus
//! says they are due, and the others at once.
//!
//! When the model asks to renumerate
//! ([`crate::model::DeviceModel::take_renumeration`]), the server tells
//! whoever drives the run through [`Server::on_signal`]; that side
//! detaches the device and imports it again, and the new import resets
//! the bus, which takes over the model's new descriptors.
//!
//! Every transfer, violation and notice goes to the bus's observers.

use std::collections::HashMap;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use crate::bus::{Bus, Completion, Signal, Status, Step, Unlink, Urb};
use crate::model::{Descriptors, Setup, Speed};
use crate::{Error, Result};

/// The protocol version.
pub const VERSION: u16 = 0x0111;

/// The speed code USB/IP and `vhci-hcd` use.
pub fn speed_code(speed: Speed) -> u32 {
    match speed {
        Speed::Low => 1,
        Speed::Full => 2,
        Speed::High => 3,
    }
}

const OP_REQ_DEVLIST: u16 = 0x8005;
const OP_REP_DEVLIST: u16 = 0x0005;
const OP_REQ_IMPORT: u16 = 0x8003;
const OP_REP_IMPORT: u16 = 0x0003;

const CMD_SUBMIT: u32 = 1;
const CMD_UNLINK: u32 = 2;
const RET_SUBMIT: u32 = 3;
const RET_UNLINK: u32 = 4;

/// Direction of a URB.
pub const DIR_OUT: u32 = 0;
/// Device to host.
pub const DIR_IN: u32 = 1;

const EPIPE: i32 = -32;
/// A device sent more than the URB holds: babble, as the Linux HCDs
/// report it.
const EOVERFLOW: i32 = -75;
const ECONNRESET: i32 = -104;
/// An isochronous packet the device did not send.
const EXDEV: i32 = -18;

const HEADER: usize = 48;
/// One `usbip_iso_packet_descriptor`.
const ISO_DESCRIPTOR: usize = 16;
/// More packets than any host puts in an isochronous URB.
const MAX_ISO_PACKETS: i32 = 1024;
const DEVICE_RECORD: usize = 312;

/// The poll period for held transfers.
const TICK: Duration = Duration::from_millis(20);

fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

fn cstr(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

fn fixed(s: &str, len: usize) -> Vec<u8> {
    let mut b = s.as_bytes()[..s.len().min(len - 1)].to_vec();
    b.resize(len, 0);
    b
}

/// The `usbip_usb_device` record of a device: 312 bytes.
pub fn device_record(
    desc: &Descriptors,
    path: &str,
    busid: &str,
    busnum: u32,
    devnum: u32,
) -> Vec<u8> {
    let config = desc.configs.first().map(Vec::as_slice).unwrap_or_default();
    let class = desc.device_class();
    let mut b = fixed(path, 256);
    b.extend(fixed(busid, 32));
    b.extend_from_slice(&busnum.to_be_bytes());
    b.extend_from_slice(&devnum.to_be_bytes());
    b.extend_from_slice(&speed_code(desc.speed).to_be_bytes());
    b.extend_from_slice(&desc.vendor_id().to_be_bytes());
    b.extend_from_slice(&desc.product_id().to_be_bytes());
    b.extend_from_slice(&desc.bcd_device().to_be_bytes());
    b.extend_from_slice(&class);
    b.push(config.get(5).copied().unwrap_or(1));
    b.push(desc.configs.len() as u8);
    b.push(config.get(4).copied().unwrap_or(0));
    debug_assert_eq!(b.len(), DEVICE_RECORD);
    b
}

fn op_header(code: u16, status: u32) -> Vec<u8> {
    let mut b = VERSION.to_be_bytes().to_vec();
    b.extend_from_slice(&code.to_be_bytes());
    b.extend_from_slice(&status.to_be_bytes());
    b
}

/// A `CMD_SUBMIT` as it came off the wire.
#[derive(Clone, Debug)]
struct Submit {
    seqnum: u32,
    devid: u32,
    direction: u32,
    ep: u8,
    length: usize,
    /// The packets of an isochronous URB: offset into the buffer and
    /// length, as its descriptors give them.
    iso: Vec<(u32, u32)>,
    setup: [u8; 8],
    data: Vec<u8>,
}

impl Submit {
    /// The bus's view of it.
    fn urb(self) -> Urb {
        let setup = (self.ep == 0).then(|| Setup::from_bytes(&self.setup));
        let is_in = setup.map_or(self.direction == DIR_IN, Setup::is_in);
        let iso: Vec<usize> = self.iso.iter().map(|&(_, length)| length as usize).collect();
        // The bus takes an OUT's packets one after the other; the
        // descriptors say where each is in the buffer.
        let data = if iso.is_empty() || is_in {
            self.data
        } else {
            let mut packed = Vec::with_capacity(self.data.len());
            for &(offset, length) in &self.iso {
                let (from, to) = (offset as usize, offset as usize + length as usize);
                let chunk = self.data.get(from..to).unwrap_or_default();
                packed.extend_from_slice(chunk);
                packed.resize(packed.len() + (length as usize - chunk.len()), 0);
            }
            packed
        };
        let length = if iso.is_empty() { self.length } else { iso.iter().sum() };
        Urb { seqnum: self.seqnum, ep: self.ep, is_in, setup, length, data, iso }
    }
}

enum Pdu {
    Submit(Submit),
    Unlink { seqnum: u32, target: u32 },
}

/// Reads PDUs off the socket until it closes or a PDU is malformed.
fn read_pdus(mut stream: TcpStream, tx: &mpsc::Sender<Pdu>) -> io::Result<()> {
    loop {
        let mut h = [0u8; HEADER];
        stream.read_exact(&mut h)?;
        let command = be32(&h[0..4]);
        let seqnum = be32(&h[4..8]);
        let pdu = match command {
            CMD_SUBMIT => {
                let direction = be32(&h[12..16]);
                let length = be32(&h[24..28]) as i32;
                let packets = be32(&h[32..36]) as i32;
                let length = usize::try_from(length).unwrap_or(0);
                let mut data = Vec::new();
                if direction == DIR_OUT && length > 0 {
                    data.resize(length, 0);
                    stream.read_exact(&mut data)?;
                }
                // Isochronous descriptors follow: offset, length, actual
                // length and status of each packet.
                let mut iso = Vec::new();
                if packets > 0 {
                    if packets > MAX_ISO_PACKETS {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidData,
                            format!("isochronous URB of {packets} packets"),
                        ));
                    }
                    let mut raw = vec![0u8; ISO_DESCRIPTOR * packets as usize];
                    stream.read_exact(&mut raw)?;
                    iso = raw
                        .as_chunks::<ISO_DESCRIPTOR>()
                        .0
                        .iter()
                        .map(|d| (be32(&d[0..4]), be32(&d[4..8])))
                        .collect();
                }
                let mut setup = [0u8; 8];
                setup.copy_from_slice(&h[40..48]);
                Pdu::Submit(Submit {
                    seqnum,
                    devid: be32(&h[8..12]),
                    direction,
                    ep: be32(&h[16..20]) as u8,
                    length,
                    iso,
                    setup,
                    data,
                })
            }
            CMD_UNLINK => Pdu::Unlink { seqnum, target: be32(&h[20..24]) },
            other => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("unknown USB/IP command {other}"),
                ));
            }
        };
        if tx.send(pdu).is_err() {
            return Ok(());
        }
    }
}

/// The `RET_SUBMIT` status of a completion.
fn errno(status: Status) -> i32 {
    match status {
        Status::Ok => 0,
        Status::Stall => EPIPE,
        Status::Babble => EOVERFLOW,
        Status::Unlinked => ECONNRESET,
        Status::Missed => EXDEV,
    }
}

fn write_ret_submit(
    out: &mut TcpStream,
    seqnum: u32,
    status: i32,
    actual: usize,
    data: &[u8],
) -> io::Result<()> {
    let mut b = Vec::with_capacity(HEADER + data.len());
    b.extend_from_slice(&RET_SUBMIT.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&[0u8; 12]);
    b.extend_from_slice(&status.to_be_bytes());
    b.extend_from_slice(&(actual as i32).to_be_bytes());
    b.extend_from_slice(&[0u8; 20]);
    b.extend_from_slice(data);
    out.write_all(&b)
}

fn write_completion(out: &mut TcpStream, c: &Completion) -> io::Result<()> {
    write_ret_submit(out, c.seqnum, errno(c.status), c.actual, &c.data)
}

/// The `RET_SUBMIT` of an isochronous URB: the packets' data one after
/// the other (an IN), then a descriptor per packet with the offset and
/// length the host gave it, what moved, and its status. `asked` is the
/// (offset, length) of each packet as submitted.
fn write_ret_submit_iso(
    out: &mut TcpStream,
    c: &Completion,
    asked: &[(u32, u32)],
) -> io::Result<()> {
    let errors = c.iso.iter().filter(|p| p.status != Status::Ok).count();
    let mut b = Vec::with_capacity(HEADER + c.data.len() + ISO_DESCRIPTOR * c.iso.len());
    b.extend_from_slice(&RET_SUBMIT.to_be_bytes());
    b.extend_from_slice(&c.seqnum.to_be_bytes());
    b.extend_from_slice(&[0u8; 12]);
    b.extend_from_slice(&0i32.to_be_bytes());
    b.extend_from_slice(&(c.actual as i32).to_be_bytes());
    b.extend_from_slice(&0i32.to_be_bytes());
    b.extend_from_slice(&(c.iso.len() as i32).to_be_bytes());
    b.extend_from_slice(&(errors as i32).to_be_bytes());
    b.extend_from_slice(&[0u8; 8]);
    b.extend_from_slice(&c.data);
    for (packet, &(offset, length)) in c.iso.iter().zip(asked) {
        b.extend_from_slice(&offset.to_be_bytes());
        b.extend_from_slice(&length.to_be_bytes());
        b.extend_from_slice(&(packet.actual as u32).to_be_bytes());
        b.extend_from_slice(&errno(packet.status).to_be_bytes());
    }
    out.write_all(&b)
}

fn write_ret_unlink(out: &mut TcpStream, seqnum: u32, status: i32) -> io::Result<()> {
    let mut b = Vec::with_capacity(HEADER);
    b.extend_from_slice(&RET_UNLINK.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&[0u8; 12]);
    b.extend_from_slice(&status.to_be_bytes());
    b.extend_from_slice(&[0u8; 24]);
    out.write_all(&b)
}

/// The server: one device, one connection at a time. The bus is shared
/// with whoever drives the run, so its model can be replaced between
/// runs ([`Bus::replace_model`]) and events armed ([`Bus::arm`]).
pub struct Server {
    bus: Arc<Mutex<Bus>>,
    busid: String,
    busnum: u32,
    devnum: u32,
    signals: Option<Box<dyn Fn(Signal) + Send>>,
}

impl std::fmt::Debug for Server {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Server")
            .field("busid", &self.busid)
            .field("devid", &self.devid())
            .finish_non_exhaustive()
    }
}

impl Server {
    /// A server for the device on `bus`, exported under `busid`;
    /// `busnum` and `devnum` are what the client is told, and form the
    /// `devid` of every URB (`busnum << 16 | devnum`).
    pub fn new(bus: Arc<Mutex<Bus>>, busid: &str, busnum: u32, devnum: u32) -> Server {
        Server { bus, busid: busid.to_owned(), busnum, devnum, signals: None }
    }

    /// What is called with each [`Signal`] of the bus, after the
    /// transfer that caused it was answered and the observers got their
    /// notice: a renumeration the model asks for, an armed event that
    /// fired. It runs on the server's thread and must not block. Without
    /// it, a device that asks to renumerate stays as it is.
    #[must_use]
    pub fn on_signal(mut self, f: impl Fn(Signal) + Send + 'static) -> Server {
        self.signals = Some(Box::new(f));
        self
    }

    fn bus(&self) -> MutexGuard<'_, Bus> {
        self.bus.lock().expect("bus lock")
    }

    fn comment(&self, message: &str) {
        self.bus().comment(message);
    }

    /// The `devid` the client must use.
    pub fn devid(&self) -> u32 {
        (self.busnum << 16) | self.devnum
    }

    /// The device's `usbip_usb_device` record; its path is
    /// `/vusb/NAME`.
    pub fn record(&self) -> Vec<u8> {
        let bus = self.bus();
        let device = bus.device();
        let path = format!("/vusb/{}", device.name());
        device_record(device.descriptors(), &path, &self.busid, self.busnum, self.devnum)
    }

    /// Accepts connections until `stop` is set.
    pub fn run(mut self, listener: &TcpListener, stop: &AtomicBool) -> Result<()> {
        listener.set_nonblocking(true)?;
        while !stop.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, peer)) => {
                    stream.set_nonblocking(false)?;
                    stream.set_nodelay(true)?;
                    self.comment(&format!("connection from {peer}"));
                    match self.connection(stream, stop) {
                        Ok(()) => self.comment("connection closed"),
                        Err(e) => self.comment(&format!("connection ended: {e}")),
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => thread::sleep(TICK),
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }

    fn connection(&mut self, mut stream: TcpStream, stop: &AtomicBool) -> io::Result<()> {
        let mut head = [0u8; 8];
        stream.read_exact(&mut head)?;
        let code = u16::from_be_bytes([head[2], head[3]]);
        match code {
            OP_REQ_DEVLIST => {
                let mut b = op_header(OP_REP_DEVLIST, 0);
                b.extend_from_slice(&1u32.to_be_bytes());
                b.extend(self.record());
                let bus = self.bus();
                if let Some(config) = bus.device().descriptors().configs.first() {
                    for i in Descriptors::interfaces(config) {
                        if i[1] == 0 {
                            b.extend_from_slice(&[i[2], i[3], i[4], 0]);
                        }
                    }
                }
                stream.write_all(&b)
            }
            OP_REQ_IMPORT => {
                let mut busid = [0u8; 32];
                stream.read_exact(&mut busid)?;
                let want = cstr(&busid);
                if want != self.busid {
                    self.comment(&format!(
                        "import of {want} refused, the device is {}",
                        self.busid
                    ));
                    return stream.write_all(&op_header(OP_REP_IMPORT, 1));
                }
                let mut b = op_header(OP_REP_IMPORT, 0);
                b.extend(self.record());
                stream.write_all(&b)?;
                self.comment(&format!("{want} imported"));
                self.bus().reset();
                self.urbs(stream, stop)
            }
            other => {
                self.comment(&format!("unknown operation {other:#06x}"));
                stream.write_all(&op_header(other & 0x7fff, 1))
            }
        }
    }

    fn urbs(&mut self, stream: TcpStream, stop: &AtomicBool) -> io::Result<()> {
        let reader = stream.try_clone()?;
        let (tx, rx) = mpsc::channel();
        let bus = Arc::clone(&self.bus);
        thread::spawn(move || {
            if let Err(e) = read_pdus(reader, &tx) {
                bus.lock().expect("bus lock").comment(&format!("reader stopped: {e}"));
            }
        });
        let mut out = stream;
        // The packets of the isochronous URBs not answered yet, as the
        // host laid them out: its answer gives them back.
        let mut asked: HashMap<u32, Vec<(u32, u32)>> = HashMap::new();
        loop {
            if stop.load(Ordering::Relaxed) {
                return Ok(());
            }
            // An isochronous URB of a device in real time is answered
            // when its time comes, not a tick later.
            let wait = self
                .bus()
                .next_due()
                .map_or(TICK, |due| due.saturating_duration_since(Instant::now()).min(TICK));
            match rx.recv_timeout(wait) {
                Ok(Pdu::Submit(submit)) => {
                    if submit.devid != self.devid() {
                        self.bus().violation(&format!(
                            "URB for devid {:#010x}, the device is {:#010x}",
                            submit.devid,
                            self.devid()
                        ));
                    }
                    if !submit.iso.is_empty() {
                        asked.insert(submit.seqnum, submit.iso.clone());
                    }
                    let produced = self.bus().submit(submit.urb());
                    self.deliver(&mut out, produced, &mut asked)?;
                }
                Ok(Pdu::Unlink { seqnum, target }) => {
                    asked.remove(&target);
                    let status = match self.bus().unlink(target) {
                        Unlink::NotHeld => 0,
                        Unlink::Cancelled => ECONNRESET,
                        Unlink::Completed(c) => {
                            // Its RET_SUBMIT goes first, so the unlink
                            // finds it completed.
                            write_completion(&mut out, &c)?;
                            0
                        }
                    };
                    write_ret_unlink(&mut out, seqnum, status)?;
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return Ok(()),
            }
            let produced = self.bus().poll();
            self.deliver(&mut out, produced, &mut asked)?;
            if self.bus().unplugged() {
                // What an unplugged device held is never answered.
                asked.clear();
            }
        }
    }

    /// Answers the completions of a step and passes its signals on.
    /// `asked` has the packets of the isochronous URBs submitted, as
    /// the host laid them out; the ones answered leave it.
    fn deliver(
        &self,
        out: &mut TcpStream,
        step: Step,
        asked: &mut HashMap<u32, Vec<(u32, u32)>>,
    ) -> io::Result<()> {
        for c in &step.completions {
            if c.iso.is_empty() {
                write_completion(out, c)?;
            } else {
                let packets = asked.remove(&c.seqnum).unwrap_or_default();
                write_ret_submit_iso(out, c, &packets)?;
            }
        }
        for signal in step.signals {
            match &self.signals {
                Some(f) => f(signal),
                None => self.comment(&format!("nobody to pass {signal:?} to")),
            }
        }
        Ok(())
    }
}

/// A client's import of `busid` from the server at `addr`: the socket
/// is ready to be handed to `vhci-hcd`, and the device record comes
/// with it.
pub fn import(addr: SocketAddr, busid: &str) -> Result<(TcpStream, Vec<u8>)> {
    let mut stream = TcpStream::connect(addr)?;
    stream.set_nodelay(true)?;
    let mut b = op_header(OP_REQ_IMPORT, 0);
    b.extend(fixed(busid, 32));
    stream.write_all(&b)?;
    let mut head = [0u8; 8];
    stream.read_exact(&mut head)?;
    let code = u16::from_be_bytes([head[2], head[3]]);
    let status = be32(&head[4..8]);
    if code != OP_REP_IMPORT || status != 0 {
        return Err(Error::Protocol(format!(
            "import of {busid} refused: code {code:#06x}, status {status}"
        )));
    }
    let mut record = vec![0u8; DEVICE_RECORD];
    stream.read_exact(&mut record)?;
    Ok((stream, record))
}

/// A client's `CMD_SUBMIT`, for tests and tools: returns the frame.
pub fn cmd_submit(
    seqnum: u32,
    devid: u32,
    direction: u32,
    ep: u8,
    length: usize,
    setup: Option<Setup>,
    data: &[u8],
) -> Vec<u8> {
    let mut b = Vec::with_capacity(HEADER + data.len());
    b.extend_from_slice(&CMD_SUBMIT.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&devid.to_be_bytes());
    b.extend_from_slice(&direction.to_be_bytes());
    b.extend_from_slice(&u32::from(ep).to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&(length as i32).to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&setup.map_or([0u8; 8], Setup::to_bytes));
    if direction == DIR_OUT {
        b.extend_from_slice(data);
    }
    b
}

/// A client's isochronous `CMD_SUBMIT`, for tests and tools: one packet
/// of each of `lengths`, laid out one after the other; `data` is what an
/// OUT sends.
pub fn cmd_submit_iso(
    seqnum: u32,
    devid: u32,
    direction: u32,
    ep: u8,
    lengths: &[usize],
    data: &[u8],
) -> Vec<u8> {
    let total: usize = lengths.iter().sum();
    let mut b = Vec::with_capacity(HEADER + data.len() + ISO_DESCRIPTOR * lengths.len());
    b.extend_from_slice(&CMD_SUBMIT.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&devid.to_be_bytes());
    b.extend_from_slice(&direction.to_be_bytes());
    b.extend_from_slice(&u32::from(ep).to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&(total as i32).to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&(lengths.len() as u32).to_be_bytes());
    b.extend_from_slice(&1u32.to_be_bytes());
    b.extend_from_slice(&[0u8; 8]);
    if direction == DIR_OUT {
        b.extend_from_slice(data);
    }
    let mut offset = 0u32;
    for &length in lengths {
        b.extend_from_slice(&offset.to_be_bytes());
        b.extend_from_slice(&(length as u32).to_be_bytes());
        b.extend_from_slice(&[0u8; 8]);
        offset += length as u32;
    }
    b
}

/// A client's `CMD_UNLINK` of `target`.
pub fn cmd_unlink(seqnum: u32, target: u32) -> Vec<u8> {
    let mut b = Vec::with_capacity(HEADER);
    b.extend_from_slice(&CMD_UNLINK.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&[0u8; 12]);
    b.extend_from_slice(&target.to_be_bytes());
    b.extend_from_slice(&[0u8; 24]);
    b
}

/// A reply read by a client: command, seqnum, status, data.
#[derive(Debug, PartialEq, Eq)]
pub struct Ret {
    /// `RET_SUBMIT` (3) or `RET_UNLINK` (4).
    pub command: u32,
    /// The sequence number answered.
    pub seqnum: u32,
    /// The status, 0 or a negative errno.
    pub status: i32,
    /// The data of an IN transfer.
    pub data: Vec<u8>,
    /// The packets of an isochronous transfer: what moved and the
    /// status of each.
    pub iso: Vec<(u32, i32)>,
}

/// Reads one reply, for tests and tools. `data_follows` says whether
/// the URB answered was an IN transfer, whose data trails a
/// `RET_SUBMIT`; a client knows its own URBs.
pub fn read_ret(stream: &mut TcpStream, data_follows: bool) -> io::Result<Ret> {
    let mut h = [0u8; HEADER];
    stream.read_exact(&mut h)?;
    let command = be32(&h[0..4]);
    let seqnum = be32(&h[4..8]);
    let status = be32(&h[20..24]) as i32;
    let mut data = Vec::new();
    if command == RET_SUBMIT && data_follows {
        let actual = usize::try_from(be32(&h[24..28]) as i32).unwrap_or(0);
        data.resize(actual, 0);
        stream.read_exact(&mut data)?;
    }
    Ok(Ret { command, seqnum, status, data, iso: Vec::new() })
}

/// Reads the reply to an isochronous URB of `packets` packets: the data
/// of an IN, then the descriptors.
pub fn read_ret_iso(stream: &mut TcpStream, data_follows: bool, packets: usize) -> io::Result<Ret> {
    let mut ret = read_ret(stream, data_follows)?;
    let mut raw = vec![0u8; ISO_DESCRIPTOR * packets];
    stream.read_exact(&mut raw)?;
    ret.iso = raw
        .as_chunks::<ISO_DESCRIPTOR>()
        .0
        .iter()
        .map(|d| (be32(&d[8..12]), be32(&d[12..16]) as i32))
        .collect();
    Ok(ret)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;
    use crate::examples::relay::UsbRelay;
    use crate::model::{DeviceModel, Reply, Violations};
    use crate::observer::{Outcome, Recorder, SharedObserver};

    fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index, length }
    }

    /// A server on a loopback port, with a recorder on its bus.
    struct Served {
        bus: Arc<Mutex<Bus>>,
        rec: Arc<Mutex<Recorder>>,
        devid: u32,
        addr: SocketAddr,
        stop: Arc<AtomicBool>,
        handle: thread::JoinHandle<Result<()>>,
    }

    fn serve(model: Box<dyn DeviceModel>) -> Served {
        let rec = Arc::new(Mutex::new(Recorder::default()));
        let shared: SharedObserver = rec.clone();
        let bus = Arc::new(Mutex::new(Bus::new(Device::new(model)).with_observer(shared)));
        let server = Server::new(Arc::clone(&bus), "1-1", 1, 2);
        let devid = server.devid();
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = thread::spawn(move || server.run(&listener, &stop2));
        Served { bus, rec, devid, addr, stop, handle }
    }

    impl Served {
        fn stop(self) -> Arc<Mutex<Recorder>> {
            self.stop.store(true, Ordering::Relaxed);
            self.handle.join().unwrap().unwrap();
            self.rec
        }
    }

    #[test]
    fn serves_a_relay_board_to_a_client() {
        let served = serve(Box::new(UsbRelay::new(2)));
        let devid = served.devid;
        assert!(import(served.addr, "9-9").is_err());
        let (mut s, record) = import(served.addr, "1-1").unwrap();
        assert_eq!(record.len(), DEVICE_RECORD);
        assert_eq!(&record[300..304], &[0x16, 0xc0, 0x05, 0xdf]);
        assert!(record.starts_with(b"/vusb/hid-relay\0"));

        // Device descriptor.
        s.write_all(&cmd_submit(1, devid, DIR_IN, 0, 18, Some(setup(0x80, 6, 0x0100, 0, 18)), &[]))
            .unwrap();
        let r = read_ret(&mut s, true).unwrap();
        assert_eq!((r.command, r.seqnum, r.status, r.data.len()), (RET_SUBMIT, 1, 0, 18));
        // Configure, then a feature report through the HID interface.
        s.write_all(&cmd_submit(2, devid, DIR_OUT, 0, 0, Some(setup(0x00, 9, 1, 0, 0)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        s.write_all(&cmd_submit(3, devid, DIR_IN, 0, 8, Some(setup(0xa1, 1, 0x0300, 0, 8)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().data, b"ABCDE\0\0\0");
        s.write_all(&cmd_submit(
            4,
            devid,
            DIR_OUT,
            0,
            8,
            Some(setup(0x21, 9, 0x0300, 0, 8)),
            &[0xff, 2, 0, 0, 0, 0, 0, 0],
        ))
        .unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        s.write_all(&cmd_submit(5, devid, DIR_IN, 0, 8, Some(setup(0xa1, 1, 0x0300, 0, 8)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().data, b"ABCDE\0\0\x02");
        // A fresh model behind the configured device: relays off again.
        served.bus.lock().unwrap().replace_model(Box::new(UsbRelay::new(2)));
        s.write_all(&cmd_submit(50, devid, DIR_IN, 0, 8, Some(setup(0xa1, 1, 0x0300, 0, 8)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().data, b"ABCDE\0\0\0");
        // An interrupt IN is held, then unlinked.
        s.write_all(&cmd_submit(6, devid, DIR_IN, 1, 64, None, &[])).unwrap();
        s.write_all(&cmd_submit(7, devid, DIR_IN, 0, 1, Some(setup(0x80, 8, 0, 0, 1)), &[]))
            .unwrap();
        let r = read_ret(&mut s, true).unwrap();
        assert_eq!((r.seqnum, r.data), (7, vec![1]));
        s.write_all(&cmd_unlink(8, 6)).unwrap();
        let r = read_ret(&mut s, false).unwrap();
        assert_eq!((r.command, r.seqnum, r.status), (RET_UNLINK, 8, ECONNRESET));
        s.write_all(&cmd_unlink(9, 6)).unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        // A device qualifier request stalls.
        s.write_all(&cmd_submit(
            10,
            devid,
            DIR_IN,
            0,
            10,
            Some(setup(0x80, 6, 0x0600, 0, 10)),
            &[],
        ))
        .unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().status, EPIPE);
        // A URB for another device is answered, and flagged.
        s.write_all(&cmd_submit(11, 7, DIR_IN, 0, 1, Some(setup(0x80, 8, 0, 0, 1)), &[])).unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().data, vec![1]);
        drop(s);
        let rec = served.stop();
        let rec = rec.lock().unwrap();
        assert_eq!(rec.violations.len(), 1, "{:?}", rec.violations);
        assert!(rec.violations[0].starts_with("URB for devid 0x00000007"));
        // The device descriptor fetch, as the observers saw it.
        let first = &rec.transfers[0];
        assert_eq!((first.seqnum, first.endpoint, first.is_in), (1, 0, true));
        assert_eq!(first.setup, Some(setup(0x80, 6, 0x0100, 0, 18)));
        assert_eq!((first.requested, first.offered, first.outcome), (18, 18, Outcome::Completed));
        // The interrupt IN was reported held once, never completed.
        let held: Vec<u32> =
            rec.transfers.iter().filter(|t| t.outcome == Outcome::Held).map(|t| t.seqnum).collect();
        assert_eq!(held, [6]);
        assert!(rec.notices.iter().any(|n| n == "1-1 imported"), "{:?}", rec.notices);
    }

    /// An isochronous IN on 0x83 that sends its interval's number, and
    /// nothing in every third; an isochronous OUT on 0x04.
    struct Stream(crate::model::Descriptors, u8, Arc<Mutex<Vec<Vec<u8>>>>, bool);

    /// The device, keeping real time or not, and what its OUT endpoint
    /// got: full speed and `bInterval` 1, a packet a millisecond.
    fn stream(realtime: bool) -> (Stream, Arc<Mutex<Vec<Vec<u8>>>>) {
        use crate::descriptor::{self, DeviceDescriptor};
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
        let model = Stream(
            crate::model::Descriptors {
                speed: Speed::Full,
                device: device.bytes(),
                configs: vec![descriptor::configuration(1, 0, 0x80, 50, &body)],
                strings: Vec::new(),
            },
            0,
            Arc::clone(&got),
            realtime,
        );
        (model, got)
    }

    impl DeviceModel for Stream {
        fn name(&self) -> &'static str {
            "stream"
        }
        fn descriptors(&self) -> &crate::model::Descriptors {
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
            self.1 += 1;
            (!self.1.is_multiple_of(3)).then(|| vec![self.1; usize::from(self.1)])
        }
        fn iso_out(&mut self, _: u8, data: &[u8], _: &mut Violations) {
            self.2.lock().unwrap().push(data.to_vec());
        }
        fn realtime(&self) -> bool {
            self.3
        }
        fn reset(&mut self) {}
    }

    /// A device that keeps real time has its URBs answered when their
    /// service intervals are over, in order, each with the packets the
    /// host laid out; one unlinked before its time is cancelled.
    #[test]
    fn in_real_time_isochronous_urbs_are_answered_when_due() {
        let (model, got) = stream(true);
        let served = serve(Box::new(model));
        let devid = served.devid;
        let (mut s, _) = import(served.addr, "1-1").unwrap();
        s.write_all(&cmd_submit(1, devid, DIR_OUT, 0, 0, Some(setup(0x00, 9, 1, 0, 0)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        // Three URBs of ten packets, one of 150 and one of 1000, submitted
        // together as a driver does: the first four are 180 packets,
        // 180 ms, each answered when its last packet's time is over.
        let started = Instant::now();
        for seqnum in 2..5 {
            s.write_all(&cmd_submit_iso(seqnum, devid, DIR_IN, 3, &[8; 10], &[])).unwrap();
        }
        s.write_all(&cmd_submit_iso(5, devid, DIR_IN, 3, &[8; 150], &[])).unwrap();
        s.write_all(&cmd_submit_iso(6, devid, DIR_IN, 3, &[8; 1000], &[])).unwrap();
        let mut last = Duration::ZERO;
        for (seqnum, packets, at_least) in [(2, 10, 10), (3, 10, 20), (4, 10, 30), (5, 150, 180)] {
            let r = read_ret_iso(&mut s, true, packets).unwrap();
            let elapsed = started.elapsed();
            assert_eq!((r.seqnum, r.status, r.iso.len()), (seqnum, 0, packets));
            assert!(elapsed >= Duration::from_millis(at_least), "seq {seqnum} after {elapsed:?}");
            last = elapsed;
        }
        // And not much later, even on a machine that is busy.
        assert!(last < Duration::from_secs(1), "the last after {last:?}");
        // The last has a second to go: cancelled, it is never answered
        // and the device never sees it.
        s.write_all(&cmd_unlink(7, 6)).unwrap();
        let r = read_ret(&mut s, false).unwrap();
        assert_eq!((r.command, r.seqnum, r.status), (RET_UNLINK, 7, ECONNRESET));
        // An OUT URB keeps the same time.
        let started = Instant::now();
        s.write_all(&cmd_submit_iso(8, devid, DIR_OUT, 4, &[1; 25], &[7; 25])).unwrap();
        let r = read_ret_iso(&mut s, false, 25).unwrap();
        assert_eq!((r.seqnum, r.status), (8, 0));
        assert!(started.elapsed() >= Duration::from_millis(25), "{:?}", started.elapsed());
        assert_eq!(got.lock().unwrap().len(), 25);
        drop(s);
        let rec = served.stop();
        let rec = rec.lock().unwrap();
        let seen: Vec<u32> = rec.transfers.iter().map(|t| t.seqnum).collect();
        assert_eq!(seen, [1, 2, 3, 4, 5, 8]);
        assert!(rec.violations.is_empty(), "{:?}", rec.violations);
    }

    #[test]
    fn isochronous_urbs_carry_their_packets_both_ways() {
        let (model, got) = stream(false);
        let served = serve(Box::new(model));
        let devid = served.devid;
        let (mut s, _) = import(served.addr, "1-1").unwrap();
        s.write_all(&cmd_submit(1, devid, DIR_OUT, 0, 0, Some(setup(0x00, 9, 1, 0, 0)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        // Four intervals: 1 and 2 bytes, nothing, 4 bytes. The data comes
        // packed, the descriptors say what each packet got.
        s.write_all(&cmd_submit_iso(2, devid, DIR_IN, 3, &[8, 8, 8, 8], &[])).unwrap();
        let r = read_ret_iso(&mut s, true, 4).unwrap();
        assert_eq!((r.seqnum, r.status), (2, 0));
        assert_eq!(r.data, [1, 2, 2, 4, 4, 4, 4]);
        assert_eq!(r.iso, [(1, 0), (2, 0), (0, EXDEV), (4, 0)]);
        // OUT: three packets, one of them empty.
        s.write_all(&cmd_submit_iso(3, devid, DIR_OUT, 4, &[2, 0, 3], &[9, 8, 7, 6, 5])).unwrap();
        let r = read_ret_iso(&mut s, false, 3).unwrap();
        assert_eq!((r.seqnum, r.status), (3, 0));
        assert_eq!(r.iso, [(2, 0), (0, 0), (3, 0)]);
        drop(s);
        let rec = served.stop();
        assert_eq!(*got.lock().unwrap(), [vec![9, 8], vec![], vec![7, 6, 5]]);
        assert_eq!(rec.lock().unwrap().violations, Vec::<String>::new());
    }

    /// Holds each bulk IN, and has 64 bytes for one that is unlinked.
    struct Holder(crate::model::Descriptors);

    impl DeviceModel for Holder {
        fn name(&self) -> &'static str {
            "holder"
        }
        fn descriptors(&self) -> &crate::model::Descriptors {
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
        fn in_unlinked(&mut self, _: u8, _: usize) -> Option<Vec<u8>> {
            Some(vec![0x5a; 64])
        }
        fn reset(&mut self) {}
    }

    /// What a held queue has received is in its oldest URB, whichever is
    /// unlinked first: its `RET_SUBMIT` with the data goes before the
    /// `RET_UNLINK`.
    #[test]
    fn an_unlinked_urb_completes_with_what_the_device_holds() {
        use crate::descriptor::{self, DeviceDescriptor};
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 1, [0xff, 0, 0], 0));
        body.extend_from_slice(&descriptor::endpoint(0x82, 2, 64, 0));
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
        let model = Holder(crate::model::Descriptors {
            speed: Speed::Full,
            device: device.bytes(),
            configs: vec![descriptor::configuration(1, 0, 0x80, 50, &body)],
            strings: Vec::new(),
        });
        let served = serve(Box::new(model));
        let devid = served.devid;
        let (mut s, _) = import(served.addr, "1-1").unwrap();
        s.write_all(&cmd_submit(1, devid, DIR_OUT, 0, 0, Some(setup(0x00, 9, 1, 0, 0)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        s.write_all(&cmd_submit(2, devid, DIR_IN, 2, 64, None, &[])).unwrap();
        s.write_all(&cmd_submit(3, devid, DIR_IN, 2, 64, None, &[])).unwrap();
        // The younger goes first and comes back with nothing.
        s.write_all(&cmd_unlink(4, 3)).unwrap();
        let r = read_ret(&mut s, false).unwrap();
        assert_eq!((r.command, r.seqnum, r.status), (RET_UNLINK, 4, ECONNRESET));
        // The oldest completes with the data, then its unlink.
        s.write_all(&cmd_unlink(5, 2)).unwrap();
        let r = read_ret(&mut s, true).unwrap();
        assert_eq!((r.seqnum, r.status, r.data), (2, ECONNRESET, vec![0x5a; 64]));
        let r = read_ret(&mut s, false).unwrap();
        assert_eq!((r.command, r.seqnum, r.status), (RET_UNLINK, 5, 0));
        drop(s);
        let rec = served.stop();
        let rec = rec.lock().unwrap();
        let unlinked: Vec<u32> = rec
            .transfers
            .iter()
            .filter(|t| t.outcome == Outcome::Unlinked)
            .map(|t| t.seqnum)
            .collect();
        assert_eq!(unlinked, [2]);
    }
}
