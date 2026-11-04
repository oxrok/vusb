// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The CDC-ACM shell: a serial instrument the kernel's `cdc_acm` binds,
//! which the tools reach through `/dev/ttyACMn`.
//!
//! A model implements [`Serial`], a byte stream, and never sees a URB:
//! [`Serial::received`] gets what the host wrote and answers through
//! [`Port::send`]; [`Port`] also says how the line is set and how DTR and
//! RTS stand. [`Cdc`] is the [`DeviceModel`] around it: the descriptors
//! (a control interface with the header, call management, ACM and union
//! functional descriptors and a notification endpoint, a data interface
//! with bulk IN and OUT), the class requests and the checks on them.
//!
//! ```text
//! impl Serial for Meter {
//!     fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
//!         port.send(b"reply");
//!     }
//! }
//! let id = Identity::new(0x1234, 0x5678, "Maker", "Meter");
//! Box::new(Cdc::new(NAME, &id, faults, Meter))
//! ```
//!
//! What the kernel does (`drivers/usb/class/cdc-acm.c`, 7.2), which is
//! what a model sees through this shell:
//!
//! - At probe, one `SET_LINE_CODING` of 9600/8n1.
//! - While the tty is open, 16 bulk IN transfers of two packets each and
//!   one interrupt IN are pending; [`Port::send`]'s bytes complete the
//!   oldest. Closed, none is, so nothing moves and [`Serial::tick`]
//!   stands still.
//! - Each open sends `SET_CONTROL_LINE_STATE` 3, unless the speed is B0,
//!   whatever the lines were: see [`LineEvent::opened`]. The last close
//!   sends 0 only under `HUPCL`, which libserialport clears, so at a
//!   reopen DTR does not rise.
//! - The driver sends `SET_LINE_CODING` only when the settings differ
//!   from those it last sent, and termios outlives a close, so what a
//!   tool causes depends on the tool before it. A harness that wants its
//!   runs alike puts the tty back at 9600/8n1 before each, which is also
//!   what a fresh model assumes.
//! - One `write()` is one bulk OUT of up to 20 packets. The chunks mean
//!   nothing to a model, which sees one stream.
//! - `GET_LINE_CODING` is never sent; the shell answers it for a usbfs
//!   tool.
//!
//! Time in this shell is counted in the host's offers, never the clock.
//! Every model takes the [`Faults`] options, all off by default.

use std::collections::VecDeque;

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{
    CLASS, Descriptors, DeviceModel, RECIPIENT_INTERFACE, Reply, Setup, Speed, Violations,
};
use crate::{Error, Result};

/// The notification endpoint's number (interrupt IN, `0x81`).
pub const EP_NOTIFY: u8 = 1;
/// The data endpoints' number (bulk IN `0x82`, bulk OUT `0x02`), which
/// an armed event's `in=2:N` and `out=2:N` count ([`crate::arm`]).
pub const EP_DATA: u8 = 2;

const SET_LINE_CODING: u8 = 0x20;
const GET_LINE_CODING: u8 = 0x21;
const SET_CONTROL_LINE_STATE: u8 = 0x22;
const SEND_BREAK: u8 = 0x23;
const SERIAL_STATE: u8 = 0x20;

const CONTROL_INTERFACE: u8 = 0;
const DATA_INTERFACE: u8 = 1;

/// Bytes a model may have queued for the host. A host that writes and
/// never reads could otherwise grow the queue without bound; a real
/// device has a buffer too.
const QUEUE_LIMIT: usize = 1 << 20;

/// Line settings, as `SET_LINE_CODING` carries them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Line {
    /// `dwDTERate`, bits per second.
    pub baud: u32,
    /// `bDataBits`: 5, 6, 7, 8 or 16.
    pub data_bits: u8,
    /// `bParityType`: 0 none, 1 odd, 2 even, 3 mark, 4 space.
    pub parity: u8,
    /// `bCharFormat`: 0 for 1 stop bit, 1 for 1.5, 2 for 2.
    pub stop: u8,
}

impl Default for Line {
    /// 9600/8n1, what the kernel sets at probe and a harness before
    /// each run.
    fn default() -> Line {
        Line { baud: 9600, data_bits: 8, parity: 0, stop: 0 }
    }
}

impl Line {
    /// From the `serialcomm` syntax: `1200/7n2`, `2400/8e1`.
    ///
    /// # Errors
    ///
    /// When it is not `BAUD/DPS` with `P` one of `n`, `o`, `e` and `S` 1
    /// or 2.
    pub fn parse(text: &str) -> Result<Line> {
        let invalid = || Error::Invalid(format!("{text}: not a line as in 1200/7n2"));
        let (rate, frame) = text.split_once('/').ok_or_else(invalid)?;
        let baud: u32 = rate.parse().ok().filter(|b| *b > 0).ok_or_else(invalid)?;
        let &[d, p, s] = frame.as_bytes() else { return Err(invalid()) };
        let data_bits = match d {
            b'5'..=b'8' => d - b'0',
            _ => return Err(invalid()),
        };
        let parity = match p {
            b'n' => 0,
            b'o' => 1,
            b'e' => 2,
            _ => return Err(invalid()),
        };
        let stop = match s {
            b'1' => 0,
            b'2' => 2,
            _ => return Err(invalid()),
        };
        Ok(Line { baud, data_bits, parity, stop })
    }

    /// The seven bytes on the wire.
    pub fn to_bytes(self) -> [u8; 7] {
        let b = self.baud.to_le_bytes();
        [b[0], b[1], b[2], b[3], self.stop, self.parity, self.data_bits]
    }

    /// From the seven bytes on the wire.
    pub fn from_bytes(b: &[u8; 7]) -> Line {
        Line {
            baud: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            stop: b[4],
            parity: b[5],
            data_bits: b[6],
        }
    }

    /// What the host may not send, as CDC PSTN 1.2 table 17 has it.
    fn problem(self) -> Option<String> {
        if self.baud == 0 {
            Some("a rate of 0".to_owned())
        } else if !matches!(self.data_bits, 5..=8 | 16) {
            Some(format!("{} data bits", self.data_bits))
        } else if self.parity > 4 {
            Some(format!("parity type {}", self.parity))
        } else if self.stop > 2 {
            Some(format!("stop bits code {}", self.stop))
        } else {
            None
        }
    }
}

/// What changed on the line, for [`Serial::changed`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineEvent {
    /// `SET_LINE_CODING`: the new settings.
    Coding(Line),
    /// `SET_CONTROL_LINE_STATE`: the lines as they are now, sent whether
    /// or not they changed.
    Lines {
        /// DTR.
        dtr: bool,
        /// RTS.
        rts: bool,
    },
    /// `SEND_BREAK`: milliseconds; 0 ends a break, `0xffff` holds it.
    Break(u16),
}

impl LineEvent {
    /// How a device learns of an open: each open sends DTR and RTS up,
    /// also when they already are (with `HUPCL` off a close leaves them),
    /// so DTR rising is not it. A tool that later raises RTS by hand looks
    /// the same.
    pub fn opened(self) -> bool {
        self == LineEvent::Lines { dtr: true, rts: true }
    }
}

/// The modem lines a device reports (`SERIAL_STATE`), see
/// [`Port::set_modem_lines`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ModemLines {
    /// DCD, `bRxCarrier`.
    pub dcd: bool,
    /// DSR, `bTxCarrier`.
    pub dsr: bool,
    /// RI, `bRingSignal`.
    pub ring: bool,
}

/// An instrument on a byte stream.
pub trait Serial: Send {
    /// The host wrote `data`. How it was cut into transfers means
    /// nothing: a protocol with frames collects them itself, in a buffer
    /// with a bound, as nothing limits what a host writes.
    fn received(&mut self, data: &[u8], port: &mut Port<'_>);

    /// The line settings or the lines changed. By default nothing
    /// happens.
    fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
        let _ = (event, port);
    }

    /// One offer of a pending read with nothing queued, a count that
    /// stands still while the tty is closed. A meter that sends without
    /// being asked paces itself by these (about one per 20 ms, more while
    /// the host is busy), never by the clock. By default nothing happens.
    fn tick(&mut self, port: &mut Port<'_>) {
        let _ = port;
    }

    /// A bus reset or a new attach: back to the state at power on. A
    /// model built fresh is in it already.
    fn reset(&mut self) {}
}

/// What a [`Serial`] model sees of the line and sends through.
pub struct Port<'a> {
    state: &'a mut State,
    violations: &'a mut Violations,
}

impl Port<'_> {
    /// Queues bytes for the host, unless the line is misset behind a
    /// UART ([`Cdc::behind_uart`]). The bytes wait until the host reads:
    /// a tty closed in between finds them at its next open, which is
    /// what a device with a FIFO does too.
    pub fn send(&mut self, data: &[u8]) {
        if !self.state.line_matches() {
            return;
        }
        if self.state.queue.len() + data.len() > QUEUE_LIMIT {
            self.violations.report(format!(
                "{} bytes queued and the host does not read; {} more dropped",
                self.state.queue.len(),
                data.len()
            ));
            return;
        }
        self.state.queue.extend(data);
    }

    /// The line settings the host set.
    pub fn line(&self) -> Line {
        self.state.line
    }

    /// DTR.
    pub fn dtr(&self) -> bool {
        self.state.dtr
    }

    /// RTS.
    pub fn rts(&self) -> bool {
        self.state.rts
    }

    /// Records what the host did outside the protocol.
    pub fn violation(&mut self, message: impl Into<String>) {
        self.violations.report(message);
    }

    /// Marks the point an event armed with `model` waits for
    /// ([`crate::arm::When::Model`]).
    pub fn mark(&mut self) {
        self.state.point = true;
    }

    /// Reports the modem lines to the host (a `SERIAL_STATE` notification
    /// on the interrupt endpoint) when they changed. The kernel takes
    /// them for `TIOCMGET`; only a lost DCD without `CLOCAL` hangs up.
    pub fn set_modem_lines(&mut self, lines: ModemLines) {
        if lines == self.state.modem {
            return;
        }
        self.state.modem = lines;
        let bits = u16::from(lines.dcd) | u16::from(lines.dsr) << 1 | u16::from(lines.ring) << 3;
        let bits = bits.to_le_bytes();
        let interface = u16::from(CONTROL_INTERFACE).to_le_bytes();
        self.state.notifications.push_back([
            0xa1,
            SERIAL_STATE,
            0,
            0,
            interface[0],
            interface[1],
            2,
            0,
            bits[0],
            bits[1],
        ]);
    }
}

/// The ids and strings of a function; the rest of the descriptors is
/// the shell's.
#[derive(Clone, Debug)]
pub struct Identity {
    /// `idVendor`.
    pub vendor: u16,
    /// `idProduct`.
    pub product: u16,
    /// `iManufacturer`'s string.
    pub manufacturer: String,
    /// `iProduct`'s string.
    pub name: String,
    /// `iSerialNumber`'s string.
    pub serial: String,
    /// Full speed by default, with 64 byte bulk endpoints; high speed has
    /// 512, which makes the kernel's reads 1024 bytes.
    pub speed: Speed,
    /// The control interface's `bInterfaceProtocol`: 0 by default (no AT
    /// commands), 1 for V.250 AT commands. The kernel binds both.
    pub protocol: u8,
}

impl Identity {
    /// A full speed function with serial number `0001`.
    pub fn new(vendor: u16, product: u16, manufacturer: &str, name: &str) -> Identity {
        Identity {
            vendor,
            product,
            manufacturer: manufacturer.to_owned(),
            name: name.to_owned(),
            serial: "0001".to_owned(),
            speed: Speed::Full,
            protocol: 0,
        }
    }
}

/// The descriptors of a function.
pub fn descriptors(id: &Identity) -> Descriptors {
    let data_packet = if id.speed == Speed::High { 512 } else { 64 };
    let notify_interval = if id.speed == Speed::High { 8 } else { 16 };
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(CONTROL_INTERFACE, 0, 1, [2, 2, id.protocol], 0));
    // Header, CDC 1.10.
    body.extend_from_slice(&[5, 0x24, 0x00, 0x10, 0x01]);
    // Call management: not by the device, on the data interface.
    body.extend_from_slice(&[5, 0x24, 0x01, 0x00, DATA_INTERFACE]);
    // ACM: line coding and control line state, SEND_BREAK.
    body.extend_from_slice(&[4, 0x24, 0x02, 0x06]);
    // Union.
    body.extend_from_slice(&[5, 0x24, 0x06, CONTROL_INTERFACE, DATA_INTERFACE]);
    body.extend_from_slice(&descriptor::endpoint(0x80 | EP_NOTIFY, 3, 16, notify_interval));
    body.extend_from_slice(&descriptor::interface(DATA_INTERFACE, 0, 2, [0x0a, 0, 0], 0));
    body.extend_from_slice(&descriptor::endpoint(0x80 | EP_DATA, 2, data_packet, 0));
    body.extend_from_slice(&descriptor::endpoint(EP_DATA, 2, data_packet, 0));
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let device = DeviceDescriptor {
        usb: 0x0200,
        class: [2, 0, 0],
        max_packet0: 64,
        vendor: id.vendor,
        product: id.product,
        device: 0x0100,
        strings: [1, 2, 3],
        configurations: 1,
    };
    Descriptors {
        speed: id.speed,
        device: device.bytes(),
        configs: vec![config],
        strings: vec![id.manufacturer.clone(), id.name.clone(), id.serial.clone()],
    }
}

/// What every model on this shell takes as options, all off by default;
/// a model's `build` hands each option to [`Faults::option`] first.
///
/// - `stall=line|lines|break` STALLs `SET_LINE_CODING`,
///   `SET_CONTROL_LINE_STATE` or `SEND_BREAK`, without effect
///   (repeatable). The kernel ignores a failed line coding; a failed
///   `lines` fails the tool's `TIOCMBIC`.
/// - `chunk=N` at most N bytes per bulk IN transfer, which a tool must
///   put together; the stream is the same.
/// - `in-limit=N` hands the host N bytes in all per power on, then holds
///   and marks the event point at the next read: a stable count for an
///   event armed with `model`.
/// - `in-stall=N` STALLs one bulk IN transfer after N completed ones.
///   The kernel clears the halt and goes on; no tool sees it. One shot,
///   an emulation artifact: `Device` does not tell the model of the
///   `CLEAR_FEATURE`.
/// - `out-hold=N` takes N bulk OUT transfers, then holds the next and
///   marks the event point: a device that stopped reading. The kernel
///   waits for it in the tool's `close()`, up to `closing_wait` (30 s).
#[derive(Clone, Debug, Default)]
pub struct Faults {
    stall_line: bool,
    stall_lines: bool,
    stall_break: bool,
    chunk: Option<usize>,
    in_limit: Option<u64>,
    in_stall: Option<u32>,
    out_hold: Option<u32>,
}

impl Faults {
    /// Takes an option of the shell: whether it was one.
    ///
    /// # Errors
    ///
    /// On a value it cannot use.
    pub fn option(&mut self, key: &str, value: &str) -> Result<bool> {
        let number = |range: std::ops::RangeInclusive<u32>| -> Result<u32> {
            crate::options::parse_number(key, value, range)
        };
        match key {
            "stall" => match value {
                "line" => self.stall_line = true,
                "lines" => self.stall_lines = true,
                "break" => self.stall_break = true,
                _ => return Err(Error::Invalid(format!("stall={value}: line, lines or break"))),
            },
            "chunk" => self.chunk = Some(number(1..=1 << 20)? as usize),
            "in-limit" => self.in_limit = Some(u64::from(number(0..=u32::MAX)?)),
            "in-stall" => self.in_stall = Some(number(0..=u32::MAX)?),
            "out-hold" => self.out_hold = Some(number(0..=u32::MAX)?),
            _ => return Ok(false),
        }
        Ok(true)
    }
}

#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Default)]
struct State {
    line: Line,
    dtr: bool,
    rts: bool,
    modem: ModemLines,
    uart: Option<Line>,
    queue: VecDeque<u8>,
    notifications: VecDeque<[u8; 10]>,
    point: bool,
    sent: u64,
    limit_marked: bool,
    in_done: u32,
    in_stalled: bool,
    out_done: u32,
    out_marked: bool,
}

impl State {
    /// Whether bytes get through a UART set to `uart` while the host's
    /// line is `line`. The rate and the bits of a frame must agree. Odd
    /// against even only fails a parity check, and an extra stop bit
    /// nothing: the bytes arrive, as they do under a driver's swapped
    /// parity.
    fn line_matches(&self) -> bool {
        self.uart.is_none_or(|u| {
            u.baud == self.line.baud
                && u.data_bits == self.line.data_bits
                && (u.parity == 0) == (self.line.parity == 0)
        })
    }

    /// Back to power on; the UART's settings are the device's.
    fn reset(&mut self) {
        *self = State { uart: self.uart, ..State::default() };
    }
}

/// A CDC-ACM function around a [`Serial`] model.
pub struct Cdc<S> {
    name: &'static str,
    descriptors: Descriptors,
    faults: Faults,
    state: State,
    /// The model.
    pub model: S,
}

impl<S: Serial> Cdc<S> {
    /// The function `name` (the model's `NAME`).
    pub fn new(name: &'static str, id: &Identity, faults: Faults, model: S) -> Cdc<S> {
        Cdc { name, descriptors: descriptors(id), faults, state: State::default(), model }
    }

    /// The instrument sits behind a UART set to `line`, as a meter behind
    /// its cable: while the host's rate, data bits or presence of parity
    /// differ, no byte gets through in either direction. On a real cable
    /// they would arrive garbled; a model that wants that sends noise
    /// itself. Without this, as on an instrument whose USB port is native,
    /// the settings do nothing.
    #[must_use]
    pub fn behind_uart(mut self, line: Line) -> Cdc<S> {
        self.state.uart = Some(line);
        self
    }

    fn class_request(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        let name = match setup.request {
            SET_LINE_CODING => "SET_LINE_CODING",
            GET_LINE_CODING => "GET_LINE_CODING",
            SET_CONTROL_LINE_STATE => "SET_CONTROL_LINE_STATE",
            SEND_BREAK => "SEND_BREAK",
            other => {
                v.report(format!("class request {other:#04x}: not one CDC-ACM has"));
                return Reply::Stall;
            }
        };
        let is_in = setup.request == GET_LINE_CODING;
        let length = match setup.request {
            SET_LINE_CODING | GET_LINE_CODING => 7,
            _ => 0,
        };
        if setup.is_in() != is_in {
            v.report(format!("{name} in the wrong direction"));
            return Reply::Stall;
        }
        if usize::from(setup.length) != length {
            v.report(format!("{name} with wLength {}, it takes {length}", setup.length));
            return Reply::Stall;
        }
        if setup.request == GET_LINE_CODING {
            return Reply::Data(self.state.line.to_bytes().to_vec());
        }
        let stalled = match setup.request {
            SET_LINE_CODING => self.faults.stall_line,
            SET_CONTROL_LINE_STATE => self.faults.stall_lines,
            _ => self.faults.stall_break,
        };
        if stalled {
            return Reply::Stall;
        }
        let event = match setup.request {
            SET_LINE_CODING => {
                let Ok(bytes) = <[u8; 7]>::try_from(data) else {
                    v.report(format!("{name} carries {} bytes", data.len()));
                    return Reply::Stall;
                };
                if setup.value != 0 {
                    v.report(format!("{name} with wValue {:#06x}", setup.value));
                }
                let line = Line::from_bytes(&bytes);
                if let Some(problem) = line.problem() {
                    v.report(format!("{name} with {problem}"));
                }
                self.state.line = line;
                LineEvent::Coding(line)
            }
            SET_CONTROL_LINE_STATE => {
                if setup.value & !3 != 0 {
                    v.report(format!("{name} with reserved bits in {:#06x}", setup.value));
                }
                self.state.dtr = setup.value & 1 != 0;
                self.state.rts = setup.value & 2 != 0;
                LineEvent::Lines { dtr: self.state.dtr, rts: self.state.rts }
            }
            _ => LineEvent::Break(setup.value),
        };
        let mut port = Port { state: &mut self.state, violations: v };
        self.model.changed(event, &mut port);
        Reply::ack()
    }
}

impl<S: Serial> DeviceModel for Cdc<S> {
    fn name(&self) -> &'static str {
        self.name
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if setup.kind() != CLASS
            || setup.recipient() != RECIPIENT_INTERFACE
            || setup.index != u16::from(CONTROL_INTERFACE)
        {
            v.report(format!(
                "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the function answers",
                setup.request_type, setup.request, setup.value, setup.index
            ));
            return Reply::Stall;
        }
        self.class_request(setup, data, v)
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP_DATA {
            v.report(format!("OUT transfer to endpoint {ep}; data goes to {EP_DATA}"));
            return Reply::Stall;
        }
        if self.faults.out_hold.is_some_and(|n| self.state.out_done >= n) {
            if !std::mem::replace(&mut self.state.out_marked, true) {
                self.state.point = true;
            }
            return Reply::Nak;
        }
        self.state.out_done = self.state.out_done.saturating_add(1);
        if self.state.line_matches() && !data.is_empty() {
            let mut port = Port { state: &mut self.state, violations: v };
            self.model.received(data, &mut port);
        }
        Reply::ack()
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep == EP_NOTIFY {
            return match self.state.notifications.pop_front() {
                Some(n) if max >= n.len() => Reply::Data(n.to_vec()),
                Some(n) => {
                    v.report(format!(
                        "interrupt IN of {max} bytes; a notification has {}",
                        n.len()
                    ));
                    Reply::Stall
                }
                None => Reply::Nak,
            };
        }
        if ep != EP_DATA {
            v.report(format!("IN transfer on endpoint {ep}; data comes from {EP_DATA}"));
            return Reply::Stall;
        }
        if self.state.queue.is_empty() {
            let mut port = Port { state: &mut self.state, violations: v };
            self.model.tick(&mut port);
        }
        if self.state.queue.is_empty() {
            return Reply::Nak;
        }
        if !self.state.in_stalled && self.faults.in_stall.is_some_and(|n| self.state.in_done >= n) {
            self.state.in_stalled = true;
            return Reply::Stall;
        }
        let mut n = self.state.queue.len().min(max);
        if let Some(chunk) = self.faults.chunk {
            n = n.min(chunk);
        }
        if let Some(limit) = self.faults.in_limit {
            let left = limit.saturating_sub(self.state.sent);
            if left == 0 {
                if !std::mem::replace(&mut self.state.limit_marked, true) {
                    self.state.point = true;
                }
                return Reply::Nak;
            }
            n = n.min(usize::try_from(left).unwrap_or(usize::MAX));
        }
        self.state.sent += n as u64;
        self.state.in_done = self.state.in_done.saturating_add(1);
        Reply::Data(self.state.queue.drain(..n).collect())
    }

    fn reset(&mut self) {
        self.state.reset();
        self.model.reset();
    }

    fn mark(&mut self) -> bool {
        std::mem::take(&mut self.state.point)
    }
}

/// The host's side, for a model's unit tests: what the kernel would do,
/// without URBs.
#[cfg(test)]
pub mod host {
    use super::*;

    /// A function with the tty open.
    pub struct Host<S> {
        /// The function.
        pub cdc: Cdc<S>,
        /// What it reported.
        pub v: Violations,
    }

    impl<S: Serial> Host<S> {
        /// Sets the line and the lines, as a tool's open with these
        /// settings does: `dtr` and `rts` as given.
        pub fn open(cdc: Cdc<S>, line: Line, dtr: bool, rts: bool) -> Host<S> {
            let mut h = Host { cdc, v: Violations::default() };
            h.set_lines(true, true);
            h.set_line(line);
            h.set_lines(dtr, rts);
            h
        }

        /// `SET_LINE_CODING`.
        pub fn set_line(&mut self, line: Line) -> Reply {
            let s = setup(0x21, SET_LINE_CODING, 0, 7);
            self.cdc.control(s, &line.to_bytes(), &mut self.v)
        }

        /// `SET_CONTROL_LINE_STATE`.
        pub fn set_lines(&mut self, dtr: bool, rts: bool) -> Reply {
            let value = u16::from(dtr) | u16::from(rts) << 1;
            self.cdc.control(setup(0x21, SET_CONTROL_LINE_STATE, value, 0), &[], &mut self.v)
        }

        /// A `write()`.
        pub fn write(&mut self, data: &[u8]) -> Reply {
            self.cdc.out(EP_DATA, data, &mut self.v)
        }

        /// One read of up to 128 bytes, the kernel's on a full speed
        /// function.
        pub fn offer(&mut self) -> Reply {
            self.cdc.in_transfer(EP_DATA, 128, &mut self.v)
        }

        /// All there is to read now.
        pub fn read(&mut self) -> Vec<u8> {
            let mut out = Vec::new();
            while let Reply::Data(d) = self.offer() {
                out.extend(d);
            }
            out
        }
    }

    /// A class setup packet to the control interface.
    pub fn setup(request_type: u8, request: u8, value: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index: 0, length }
    }
}

#[cfg(test)]
mod tests {
    use super::host::{Host, setup};
    use super::*;
    use crate::model::Descriptors as D;

    /// Answers each byte with itself and remembers what changed.
    #[derive(Default)]
    struct Echo {
        events: Vec<LineEvent>,
        ticks: u32,
    }

    impl Serial for Echo {
        fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
            port.send(data);
        }
        fn changed(&mut self, event: LineEvent, port: &mut Port<'_>) {
            self.events.push(event);
            if let LineEvent::Lines { dtr: true, .. } = event {
                port.set_modem_lines(ModemLines { dcd: true, dsr: true, ring: false });
            }
        }
        fn tick(&mut self, _port: &mut Port<'_>) {
            self.ticks += 1;
        }
    }

    fn cdc(faults: &[(&str, &str)]) -> Cdc<Echo> {
        let mut f = Faults::default();
        for (k, v) in faults {
            assert!(f.option(k, v).unwrap(), "{k}");
        }
        Cdc::new("echo", &Identity::new(0x1234, 0x5678, "Maker", "Echo"), f, Echo::default())
    }

    fn opened(faults: &[(&str, &str)]) -> Host<Echo> {
        Host::open(cdc(faults), Line::default(), true, true)
    }

    fn drained(h: &mut Host<Echo>) -> usize {
        h.v.drain().len()
    }

    #[test]
    fn the_descriptors_are_those_the_kernel_binds() {
        let d = descriptors(&Identity::new(0x1234, 0x5678, "M", "P"));
        assert_eq!(d.device_class(), [2, 0, 0]);
        let c = d.config_by_value(1).unwrap();
        assert_eq!(D::interfaces(c), vec![[0, 0, 2, 2, 0], [1, 0, 0x0a, 0, 0]]);
        assert_eq!(c[4], 2, "two interfaces");
        let functional: Vec<&[u8]> = descriptor::records(c).filter(|r| r[1] == 0x24).collect();
        assert_eq!(functional.iter().map(|r| r[2]).collect::<Vec<_>>(), vec![0, 1, 2, 6]);
        assert_eq!(functional[1][4], 1, "call management names the data interface");
        assert_eq!(functional[3][3..], [0, 1], "the union joins them");
        let e = D::endpoints(c);
        assert_eq!(e.iter().map(|e| e.address).collect::<Vec<_>>(), vec![0x81, 0x82, 0x02]);
        assert_eq!(e.iter().map(|e| e.transfer_type()).collect::<Vec<_>>(), vec![3, 2, 2]);
        assert_eq!(e[1].max_packet, 64);
        let mut high = Identity::new(1, 2, "M", "P");
        high.speed = Speed::High;
        let d = descriptors(&high);
        let e = D::endpoints(d.config_by_value(1).unwrap());
        assert_eq!(e[1].max_packet, 512);
    }

    #[test]
    fn the_line_parses_and_round_trips() {
        let l = Line::parse("1200/7n2").unwrap();
        assert_eq!(l, Line { baud: 1200, data_bits: 7, parity: 0, stop: 2 });
        assert_eq!(Line::from_bytes(&l.to_bytes()), l);
        assert_eq!(l.to_bytes(), [0xb0, 0x04, 0, 0, 2, 0, 7]);
        assert_eq!(Line::parse("2400/8e1").unwrap().parity, 2);
        assert_eq!(Line::parse("2400/8o1").unwrap().parity, 1);
        for bad in ["", "9600", "0/8n1", "x/8n1", "9600/9n1", "9600/8x1", "9600/8n3", "9600/8n"] {
            assert!(Line::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn the_class_requests_reach_the_model() {
        let mut h = Host { cdc: cdc(&[]), v: Violations::default() };
        let line = Line::parse("1200/7o2").unwrap();
        assert_eq!(h.set_line(line), Reply::ack());
        let got = h.cdc.control(setup(0xa1, GET_LINE_CODING, 0, 7), &[], &mut h.v);
        assert_eq!(got, Reply::Data(line.to_bytes().to_vec()));
        assert_eq!(h.set_lines(true, false), Reply::ack());
        let brk = setup(0x21, SEND_BREAK, 0xffff, 0);
        assert_eq!(h.cdc.control(brk, &[], &mut h.v), Reply::ack());
        assert_eq!(
            h.cdc.model.events,
            vec![
                LineEvent::Coding(line),
                LineEvent::Lines { dtr: true, rts: false },
                LineEvent::Break(0xffff)
            ]
        );
        assert!(h.v.is_empty(), "{:?}", h.v);
        assert_eq!(h.cdc.state.line, line);
        assert!(h.cdc.state.dtr && !h.cdc.state.rts);
    }

    #[test]
    fn each_check_on_a_request_fires() {
        let mut h = opened(&[]);
        let line = Line::default().to_bytes();
        let cases: [(Setup, &[u8]); 9] = [
            (setup(0x21, 0x02, 0, 0), &[]),                    // not CDC-ACM
            (setup(0xa1, SET_LINE_CODING, 0, 7), &[]),         // direction
            (setup(0x21, GET_LINE_CODING, 0, 7), &line),       // direction
            (setup(0xa1, GET_LINE_CODING, 0, 8), &[]),         // length
            (setup(0x21, SET_LINE_CODING, 0, 6), &line[..6]),  // length
            (setup(0x21, SET_CONTROL_LINE_STATE, 0, 1), &[0]), // length
            (setup(0x21, SEND_BREAK, 0, 2), &[0, 0]),          // length
            (setup(0x21, SET_LINE_CODING, 0, 7), &line[..6]),  // carries 6
            (Setup { request_type: 0x21, request: 0x22, value: 0, index: 1, length: 0 }, &[]), // the data interface
        ];
        for (i, (s, data)) in cases.iter().enumerate() {
            assert_eq!(h.cdc.control(*s, data, &mut h.v), Reply::Stall, "case {i}");
            assert_eq!(drained(&mut h), 1, "case {i}");
        }
        // Accepted, but wrong.
        let vendor = Setup { request_type: 0x41, request: 1, value: 0, index: 0, length: 0 };
        assert_eq!(h.cdc.control(vendor, &[], &mut h.v), Reply::Stall);
        assert_eq!(drained(&mut h), 1);
        let reserved = setup(0x21, SET_CONTROL_LINE_STATE, 4, 0);
        assert_eq!(h.cdc.control(reserved, &[], &mut h.v), Reply::ack());
        assert_eq!(drained(&mut h), 1);
        let value = setup(0x21, SET_LINE_CODING, 1, 7);
        assert_eq!(h.cdc.control(value, &line, &mut h.v), Reply::ack());
        assert_eq!(drained(&mut h), 1);
        for bad in [
            Line { baud: 0, ..Line::default() },
            Line { data_bits: 9, ..Line::default() },
            Line { parity: 5, ..Line::default() },
            Line { stop: 3, ..Line::default() },
        ] {
            assert_eq!(h.set_line(bad), Reply::ack());
            assert_eq!(drained(&mut h), 1, "{bad:?}");
        }
        assert_eq!(h.set_line(Line { data_bits: 16, parity: 4, stop: 1, baud: 1 }), Reply::ack());
        assert!(h.v.is_empty());
    }

    #[test]
    fn bytes_go_both_ways_cut_to_the_transfer() {
        let mut h = opened(&[]);
        assert_eq!(h.offer(), Reply::Nak);
        assert_eq!(h.write(&[7u8; 300]), Reply::ack());
        assert_eq!(h.offer(), Reply::Data(vec![7; 128]));
        assert_eq!(h.offer(), Reply::Data(vec![7; 128]));
        assert_eq!(h.offer(), Reply::Data(vec![7; 44]));
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.v.is_empty());
        // Other endpoints.
        assert_eq!(h.cdc.out(1, b"x", &mut h.v), Reply::Stall);
        assert_eq!(drained(&mut h), 1);
        assert_eq!(h.cdc.in_transfer(3, 64, &mut h.v), Reply::Stall);
        assert_eq!(drained(&mut h), 1);
        // A short interrupt IN cannot hold the notification.
        h.cdc.state.notifications.push_back([0; 10]);
        assert_eq!(h.cdc.in_transfer(EP_NOTIFY, 8, &mut h.v), Reply::Stall);
        assert_eq!(drained(&mut h), 1);
    }

    #[test]
    fn each_open_is_seen_whether_or_not_the_lines_change() {
        let mut h = Host { cdc: cdc(&[]), v: Violations::default() };
        h.set_lines(true, true);
        h.set_lines(true, false);
        h.set_lines(true, true);
        h.set_lines(true, true);
        let opens = h.cdc.model.events.iter().filter(|e| e.opened()).count();
        assert_eq!(opens, 3);
        assert!(!LineEvent::Lines { dtr: true, rts: false }.opened());
        assert!(!LineEvent::Break(0).opened());
    }

    #[test]
    fn a_tick_only_comes_with_nothing_queued() {
        let mut h = opened(&[]);
        h.offer();
        assert_eq!(h.cdc.model.ticks, 1);
        h.write(b"x");
        h.offer();
        assert_eq!(h.cdc.model.ticks, 1);
        h.offer();
        assert_eq!(h.cdc.model.ticks, 2);
    }

    #[test]
    fn the_queue_has_a_bound() {
        let mut h = opened(&[]);
        let block = vec![0u8; 1 << 19];
        h.write(&block);
        h.write(&block);
        assert!(h.v.is_empty());
        h.write(b"x");
        assert_eq!(drained(&mut h), 1);
        assert_eq!(h.cdc.state.queue.len(), QUEUE_LIMIT);
    }

    #[test]
    fn modem_lines_notify_when_they_change() {
        let mut h = Host { cdc: cdc(&[]), v: Violations::default() };
        assert_eq!(h.cdc.in_transfer(EP_NOTIFY, 16, &mut h.v), Reply::Nak);
        h.set_lines(true, true);
        let Reply::Data(n) = h.cdc.in_transfer(EP_NOTIFY, 16, &mut h.v) else { panic!() };
        assert_eq!(n, vec![0xa1, 0x20, 0, 0, 0, 0, 2, 0, 3, 0]);
        // The same lines again: no notification.
        h.set_lines(true, true);
        assert_eq!(h.cdc.in_transfer(EP_NOTIFY, 16, &mut h.v), Reply::Nak);
    }

    #[test]
    fn behind_a_uart_a_misset_line_passes_nothing() {
        let uart = Line::parse("1200/7n2").unwrap();
        let mut h = Host { cdc: cdc(&[]).behind_uart(uart), v: Violations::default() };
        h.set_lines(true, true);
        h.write(b"a");
        assert_eq!(h.read(), b"");
        h.set_line(uart);
        h.write(b"b");
        assert_eq!(h.read(), b"b");
        // Sent, then misset before it is read: already in the FIFO.
        h.write(b"c");
        h.set_line(Line::default());
        assert_eq!(h.read(), b"c");
        assert!(h.v.is_empty());
        // Parity's kind and the stop bits do not garble a byte.
        h.set_line(Line::parse("1200/7n1").unwrap());
        h.write(b"d");
        assert_eq!(h.read(), b"d");
        h.set_line(Line::parse("1200/8n2").unwrap());
        h.write(b"e");
        assert_eq!(h.read(), b"");
        h.set_line(Line::parse("1200/7e2").unwrap());
        h.write(b"f");
        assert_eq!(h.read(), b"", "parity where the UART has none");
        let mut odd = Host {
            cdc: cdc(&[]).behind_uart(Line::parse("1200/7o1").unwrap()),
            v: Violations::default(),
        };
        odd.set_line(Line::parse("1200/7e1").unwrap());
        odd.write(b"g");
        assert_eq!(odd.read(), b"g");
        // A reset keeps the UART and goes back to the default line.
        h.cdc.reset();
        assert_eq!(h.cdc.state.line, Line::default());
        assert_eq!(h.cdc.state.uart, Some(uart));
    }

    #[test]
    fn a_reset_is_power_on() {
        let mut h = opened(&[("in-limit", "1")]);
        h.write(b"ab");
        assert_eq!(h.offer(), Reply::Data(b"a".to_vec()));
        h.cdc.reset();
        assert!(!h.cdc.state.dtr && !h.cdc.state.rts);
        assert_eq!(h.offer(), Reply::Nak, "the queue went");
        h.write(b"c");
        assert_eq!(h.offer(), Reply::Data(b"c".to_vec()), "the limit counts anew");
    }

    #[test]
    fn the_faults_do_what_they_say() {
        // stall: without effect.
        let mut h = Host { cdc: cdc(&[("stall", "lines")]), v: Violations::default() };
        assert_eq!(h.set_lines(true, true), Reply::Stall);
        assert!(!h.cdc.state.dtr);
        assert!(h.v.is_empty());
        let mut h = Host { cdc: cdc(&[("stall", "line")]), v: Violations::default() };
        assert_eq!(h.set_line(Line::parse("1200/7n2").unwrap()), Reply::Stall);
        assert_eq!(h.cdc.state.line, Line::default());
        let mut h = Host { cdc: cdc(&[("stall", "break")]), v: Violations::default() };
        let brk = setup(0x21, SEND_BREAK, 0, 0);
        assert_eq!(h.cdc.control(brk, &[], &mut h.v), Reply::Stall);
        assert_eq!(h.cdc.model.events, vec![]);
        // chunk.
        let mut h = opened(&[("chunk", "2")]);
        h.write(b"abc");
        assert_eq!(h.offer(), Reply::Data(b"ab".to_vec()));
        assert_eq!(h.offer(), Reply::Data(b"c".to_vec()));
        // in-limit: holds and marks once.
        let mut h = opened(&[("in-limit", "2")]);
        h.write(b"abc");
        assert_eq!(h.offer(), Reply::Data(b"ab".to_vec()));
        assert!(!h.cdc.mark());
        assert_eq!(h.offer(), Reply::Nak);
        assert!(h.cdc.mark());
        assert_eq!(h.offer(), Reply::Nak);
        assert!(!h.cdc.mark(), "once");
        // in-stall: once, after N.
        let mut h = opened(&[("in-stall", "1")]);
        h.write(b"a");
        assert_eq!(h.offer(), Reply::Data(b"a".to_vec()));
        assert_eq!(h.offer(), Reply::Nak, "no STALL with nothing to send");
        h.write(b"b");
        assert_eq!(h.offer(), Reply::Stall);
        assert_eq!(h.offer(), Reply::Data(b"b".to_vec()));
        h.write(b"c");
        assert_eq!(h.offer(), Reply::Data(b"c".to_vec()));
        // out-hold: holds and marks once.
        let mut h = opened(&[("out-hold", "1")]);
        assert_eq!(h.write(b"a"), Reply::ack());
        assert_eq!(h.write(b"b"), Reply::Nak);
        assert!(h.cdc.mark());
        assert_eq!(h.write(b"b"), Reply::Nak);
        assert!(!h.cdc.mark());
        assert_eq!(h.read(), b"a", "the held write did not arrive");
        // Options.
        let mut f = Faults::default();
        assert!(!f.option("packet", "00").unwrap(), "not the shell's");
        assert!(f.option("stall", "nothing").is_err());
        assert!(f.option("chunk", "0").is_err());
        assert!(f.option("in-limit", "x").is_err());
    }
}
