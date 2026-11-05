// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A transaction log in the fixture dialect: the compact lines of
//! recorded USB fixtures, made to be compared with golden files.
//!
//! [`Log`] is an [`Observer`] that writes one line per completed
//! transfer:
//!
//! ```text
//! # section run
//! ctl_out 00 09 0001 0000 - -> ok
//! ctl_in 21 01 0300 0000 8 -> 4142434445000000
//! bulk_out 02 0102 -> 2
//! bulk_in 82 64 -> a5a5
//! int_in 81 8 -> err stall
//! ```
//!
//! # The lines
//!
//! - The first word is the transfer type and the direction: `ctl_in`,
//!   `ctl_out`, `bulk_in`, `bulk_out`, `int_in`, `int_out`, `iso_in`,
//!   `iso_out`. A transfer on an endpoint the configuration lacks, a
//!   violation already, is written as `iso`.
//! - A control transfer goes on with `bmRequestType` (its direction bit
//!   cleared: the word has it), `bRequest`, `wValue` and `wIndex` in hex,
//!   then `wLength` in decimal for an IN, the data for an OUT.
//! - The others go on with the endpoint address, two hex digits, then
//!   the length the device was offered, in decimal, for an IN, the data
//!   for an OUT.
//! - After the arrow: the data of an IN; `ok` for a control OUT; for
//!   another OUT the bytes the device took, in decimal; `err stall`;
//!   `nak` for a transfer that ended while the device still held it.
//! - Data is hex, and `-` when there is none.
//!
//! A transfer the device holds with a NAK is not written until it
//! completes, so a log does not depend on how often the host asks. A
//! babble is the line and ` (babble: the URB holds N)`. An IN unlinked
//! while the device held data for it is a line with the bytes that
//! reached the URB.
//!
//! Between them: `# section NAME`, `! violation: MESSAGE`, `disconnect`
//! and `connect VVVV:PPPP` for a renumeration, and the word of an armed
//! event right after the transfer that fired it. In verbose mode
//! comments are `# TEXT` lines, a held transfer among them:
//! `# seq N: LINE (held)`.
//!
//! # Rules
//!
//! [`Rules`] are the caller's, and may change while the log runs
//! ([`Log::set_rules`]), as when another model takes the device's place:
//!
//! - Data longer than [`Rules::digest_above`], in either direction, is
//!   `@LEN:SHA256`: its length and its SHA-256, so a firmware upload is
//!   not a line of megabytes.
//! - Bytes of an OUT transfer that [`Rules::undefined`] names are `xx`:
//!   what a host leaves uninitialised, which would differ from run to
//!   run. In a digest they count as 0 and their ranges follow it,
//!   `@LEN:SHA256/xx=A-B,C-D`, the ends included.
//!
//! # Isochronous transfers
//!
//! One line per transfer, as for the others, with the transfer's whole
//! data, and then its packets: ` (packets: ...)`, a word for each in
//! order. `N` is a packet of N bytes, `missed` one the device did not
//! send or take, `N/babble` one cut to the N bytes the host had room
//! for; `*K` after a word says K packets alike in a row.
//!
//! ```text
//! iso_in 81 960 -> @960:5d0c... (packets: 96*10)
//! iso_in 83 48 -> 01020203030305050505050606060606060606 (packets: 1 2 3 missed 5 8/babble)
//! iso_out 04 010203 -> 3 (packets: 2 0 1)
//! ```
//!
//! The number after the arrow of an `iso_out` is the bytes the device
//! took in all. The length of a missed OUT packet is not recorded.
//!
//! # Examples
//!
//! ```
//! use std::sync::{Arc, Mutex};
//! use vusb::bus::Bus;
//! use vusb::device::Device;
//! use vusb::examples::relay::UsbRelay;
//! use vusb::model::Setup;
//! use vusb::observer::fixture::{Log, Rules};
//! use vusb::observer::SharedObserver;
//!
//! let log = Arc::new(Mutex::new(Log::new(Vec::new(), Rules::default())));
//! let shared: SharedObserver = log.clone();
//! let mut bus = Bus::new(Device::new(Box::new(UsbRelay::new(2)))).with_observer(shared);
//! bus.control(Setup { request_type: 0x00, request: 9, value: 1, index: 0, length: 0 }, &[]);
//! bus.control(Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 8 }, &[]);
//!
//! let log = log.lock().unwrap();
//! let text = String::from_utf8_lossy(log.get_ref());
//! assert_eq!(
//!     text,
//!     "ctl_out 00 09 0001 0000 - -> ok\n\
//!      ctl_in 21 01 0300 0000 8 -> 4142434445000000\n"
//! );
//! ```

use std::fmt::Write as _;
use std::io::Write;
use std::ops::Range;

use sha2::{Digest as _, Sha256};

use super::{Notice, Observer, Outcome, Transfer};
use crate::bus::{IsoPacket, Status};
use crate::model::{BULK, CONTROL, INTERRUPT, Reply, Setup};

/// The usual value of [`Rules::digest_above`]: data of more than a
/// kilobyte is written as its digest.
pub const DIGEST_ABOVE: usize = 1024;

/// The bytes of an OUT transfer to endpoint number `ep` that the host
/// leaves undefined, see [`Rules::undefined`].
pub type UndefinedBytes = fn(ep: u8, data: &[u8]) -> Vec<Range<usize>>;

/// How transfers are written: what depends on the device behind the
/// log, not on the dialect.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rules {
    /// Data longer than this is written as `@LEN:SHA256` rather than in
    /// hex ([`DIGEST_ABOVE`] is the usual value); `None` writes every
    /// byte.
    pub digest_above: Option<usize>,
    /// The bytes of an OUT transfer on a bulk, interrupt or isochronous
    /// endpoint that the host leaves undefined, such as a C driver's
    /// uninitialised struct field, which would make the log differ from
    /// one run to the next. The log shows them as `xx`.
    pub undefined: Option<UndefinedBytes>,
}

impl Rules {
    /// Data longer than `above` as its digest, every byte defined.
    pub fn digest_above(above: usize) -> Rules {
        Rules { digest_above: Some(above), undefined: None }
    }
}

/// The log: an [`Observer`] that writes the dialect to `W`, a line at a
/// time, flushed after each.
#[derive(Debug)]
pub struct Log<W> {
    out: W,
    verbose: bool,
    rules: Rules,
}

impl<W: Write + Send> Log<W> {
    /// A log to `out` under `rules`, without comments.
    pub fn new(out: W, rules: Rules) -> Log<W> {
        Log { out, verbose: false, rules }
    }

    /// Whether comments and held transfers are written too, as `#` lines.
    #[must_use]
    pub fn verbose(mut self, verbose: bool) -> Log<W> {
        self.verbose = verbose;
        self
    }

    /// The rules from now on: those of the model now behind the device.
    pub fn set_rules(&mut self, rules: Rules) {
        self.rules = rules;
    }

    /// The rules in force.
    pub fn rules(&self) -> Rules {
        self.rules
    }

    /// Comments and held transfers from now on, or no more.
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose;
    }

    /// What the log writes to.
    pub fn get_ref(&self) -> &W {
        &self.out
    }

    /// The writer back.
    pub fn into_inner(self) -> W {
        self.out
    }

    fn write(&mut self, line: &str) {
        // An error writing the log is not worth stopping the device.
        let _ = writeln!(self.out, "{line}");
        let _ = self.out.flush();
    }

    /// A comment, in verbose mode only.
    fn comment(&mut self, message: &str) {
        if self.verbose {
            self.write(&format!("# {message}"));
        }
    }
}

impl<W: Write + Send> Observer for Log<W> {
    fn transfer(&mut self, t: &Transfer) {
        let text = line(t, &self.rules);
        if t.outcome == Outcome::Held {
            self.comment(&format!("seq {}: {text} (held)", t.seqnum));
        } else {
            self.write(&text);
        }
    }

    fn violation(&mut self, message: &str) {
        self.write(&format!("! violation: {message}"));
    }

    fn notice(&mut self, notice: Notice<'_>) {
        match notice {
            Notice::Comment(c) => self.comment(c),
            Notice::Section(name) => self.write(&format!("# section {name}")),
            // Lines a comparison keeps, in the section of the run that
            // caused them.
            Notice::Renumeration { vendor, product } => {
                self.write("disconnect");
                self.write(&format!("connect {vendor:04x}:{product:04x}"));
            }
            // Right after the transfer that fired it.
            Notice::Fired(event) => self.write(event.word()),
        }
    }
}

/// The line of a transfer under `rules`, whatever its outcome: a babble
/// has its note, an unlinked IN the bytes that reached the URB, an
/// isochronous transfer its packets. A held transfer's line ends in
/// `-> nak`; [`Log`] writes it as a comment, and only in verbose mode.
pub fn line(t: &Transfer, rules: &Rules) -> String {
    let above = rules.digest_above;
    let mut text = if let Some(setup) = t.setup {
        control_line(setup, &t.data, &t.reply, above)
    } else if !t.packets.is_empty() {
        isochronous_line(t, rules)
    } else {
        // An endpoint the configuration lacks has no type: `iso`.
        let kind = t.kind.unwrap_or(0xff);
        if t.is_in {
            // An unlinked transfer shows what reached the URB.
            let cut = match &t.reply {
                Reply::Data(d) if t.outcome == Outcome::Unlinked => {
                    Some(Reply::Data(d[..d.len().min(t.requested)].to_vec()))
                }
                _ => None,
            };
            let reply = cut.as_ref().unwrap_or(&t.reply);
            transfer_line(kind, t.endpoint, t.offered, &t.data, reply, above)
        } else {
            let undefined = undefined(t, rules);
            transfer_line_undefined(
                kind, t.endpoint, t.offered, &t.data, &undefined, &t.reply, above,
            )
        }
    };
    if t.outcome == Outcome::Babble {
        let _ = write!(text, " (babble: the URB holds {})", t.requested);
    }
    text
}

fn undefined(t: &Transfer, rules: &Rules) -> Vec<Range<usize>> {
    rules.undefined.map_or_else(Vec::new, |f| f(t.endpoint & 0x0f, &t.data))
}

/// An isochronous transfer: the line of the others, the bytes the
/// device took in all after the arrow of an OUT, and the packets.
fn isochronous_line(t: &Transfer, rules: &Rules) -> String {
    let above = rules.digest_above;
    let mut text = if t.is_in {
        format!("iso_in {:02x} {} -> {}", t.endpoint, t.offered, result(&t.reply, true, above))
    } else {
        let taken: usize = t.packets.iter().map(|p| p.actual).sum();
        let bytes = data_undefined(&t.data, &undefined(t, rules), above);
        format!("iso_out {:02x} {bytes} -> {taken}", t.endpoint)
    };
    text.push_str(" (packets:");
    let mut packets = t.packets.iter().peekable();
    while let Some(packet) = packets.next() {
        let mut alike = 1;
        while packets.next_if_eq(&packet).is_some() {
            alike += 1;
        }
        let _ = write!(text, " {}", packet_word(*packet));
        if alike > 1 {
            let _ = write!(text, "*{alike}");
        }
    }
    text.push(')');
    text
}

fn packet_word(packet: IsoPacket) -> String {
    match packet.status {
        Status::Missed => "missed".to_owned(),
        Status::Babble => format!("{}/babble", packet.actual),
        _ => packet.actual.to_string(),
    }
}

/// Bytes as hex pairs, `-` when there are none.
pub fn hex(bytes: &[u8]) -> String {
    if bytes.is_empty() {
        return "-".to_owned();
    }
    super::hex(bytes)
}

/// Bytes as [`hex`], or as `@LEN:SHA256` when longer than `above`.
pub fn data(bytes: &[u8], above: Option<usize>) -> String {
    if above.is_some_and(|n| bytes.len() > n) {
        format!("@{}:{}", bytes.len(), hex(&Sha256::digest(bytes)))
    } else {
        hex(bytes)
    }
}

/// Bytes as [`data`], those in `undefined` (bytes the host left
/// undefined, [`Rules::undefined`]) as `xx`; in a digest they count as 0
/// and their ranges follow it, `@LEN:SHA256/xx=A-B,...`.
pub fn data_undefined(bytes: &[u8], undefined: &[Range<usize>], above: Option<usize>) -> String {
    let inside = |i: usize| undefined.iter().any(|r| r.contains(&i));
    if undefined.is_empty() {
        data(bytes, above)
    } else if above.is_some_and(|n| bytes.len() > n) {
        let zeroed: Vec<u8> =
            bytes.iter().enumerate().map(|(i, b)| if inside(i) { 0 } else { *b }).collect();
        let ranges: Vec<String> =
            undefined.iter().map(|r| format!("{}-{}", r.start, r.end.saturating_sub(1))).collect();
        format!("{}/xx={}", data(&zeroed, above), ranges.join(","))
    } else {
        bytes.iter().enumerate().fold(String::with_capacity(2 * bytes.len()), |mut s, (i, b)| {
            if inside(i) {
                s.push_str("xx");
            } else {
                let _ = write!(s, "{b:02x}");
            }
            s
        })
    }
}

fn result(reply: &Reply, is_in: bool, above: Option<usize>) -> String {
    match reply {
        Reply::Data(d) if is_in => data(d, above),
        Reply::Data(_) => "ok".to_owned(),
        Reply::Nak => "nak".to_owned(),
        Reply::Stall => "err stall".to_owned(),
    }
}

/// The line of a control transfer, the direction bit of
/// `bmRequestType` left to the `ctl_in` or `ctl_out` word; data longer
/// than `above` as its digest.
pub fn control_line(setup: Setup, bytes: &[u8], reply: &Reply, above: Option<usize>) -> String {
    let kind = setup.request_type & 0x7f;
    if setup.is_in() {
        format!(
            "ctl_in {kind:02x} {:02x} {:04x} {:04x} {} -> {}",
            setup.request,
            setup.value,
            setup.index,
            setup.length,
            result(reply, true, above)
        )
    } else {
        format!(
            "ctl_out {kind:02x} {:02x} {:04x} {:04x} {} -> {}",
            setup.request,
            setup.value,
            setup.index,
            data(bytes, above),
            result(reply, false, above)
        )
    }
}

/// The line of a bulk or interrupt transfer on endpoint `address`, of
/// the type `transfer_type` ([`crate::model::BULK`] and the others):
/// `max` is the length asked of an IN, `bytes` the payload of an OUT.
pub fn transfer_line(
    transfer_type: u8,
    address: u8,
    max: usize,
    bytes: &[u8],
    reply: &Reply,
    above: Option<usize>,
) -> String {
    transfer_line_undefined(transfer_type, address, max, bytes, &[], reply, above)
}

/// [`transfer_line`] with the OUT payload's `undefined` bytes shown as
/// [`data_undefined`] shows them.
pub fn transfer_line_undefined(
    transfer_type: u8,
    address: u8,
    max: usize,
    bytes: &[u8],
    undefined: &[Range<usize>],
    reply: &Reply,
    above: Option<usize>,
) -> String {
    let word = match transfer_type {
        BULK => "bulk",
        INTERRUPT => "int",
        CONTROL => "ctl",
        _ => "iso",
    };
    if address & 0x80 != 0 {
        format!("{word}_in {address:02x} {max} -> {}", result(reply, true, above))
    } else {
        let taken = match reply {
            Reply::Data(d) if d.is_empty() => bytes.len().to_string(),
            Reply::Data(d) => d.len().to_string(),
            other => result(other, false, above),
        };
        format!("{word}_out {address:02x} {} -> {taken}", data_undefined(bytes, undefined, above))
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::arm::Event;
    use crate::bus::Bus;
    use crate::device::Device;
    use crate::examples::relay::UsbRelay;
    use crate::examples::uac_mic;
    use crate::host::enumerate;
    use crate::model::ISOCHRONOUS;
    use crate::observer::SharedObserver;

    /// SHA-256 of `abc`.
    const ABC: &str = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

    /// A completed transfer of 64 bytes on `endpoint`, of type `kind`.
    fn transfer(kind: u8, endpoint: u8, data: &[u8], reply: Reply) -> Transfer {
        Transfer {
            seqnum: 7,
            endpoint,
            kind: Some(kind),
            setup: None,
            is_in: endpoint & 0x80 != 0,
            requested: 64,
            offered: 64,
            data: data.to_vec(),
            reply,
            outcome: Outcome::Completed,
            packets: Vec::new(),
        }
    }

    fn control(request_type: u8, length: u16, data: &[u8], reply: Reply) -> Transfer {
        let setup = Setup { request_type, request: 1, value: 0x0300, index: 2, length };
        Transfer { setup: Some(setup), ..transfer(CONTROL, 0, data, reply) }
    }

    /// What a log writes of what `seen` shows it.
    fn written(verbose: bool, rules: Rules, seen: impl FnOnce(&mut Log<Vec<u8>>)) -> String {
        let mut log = Log::new(Vec::new(), rules).verbose(verbose);
        seen(&mut log);
        String::from_utf8(log.into_inner()).unwrap()
    }

    fn plain(t: &Transfer) -> String {
        line(t, &Rules::default())
    }

    /// What a log says of a relay board configured and read, with `rules`.
    fn relay_log(rules: Rules) -> String {
        let log = Arc::new(Mutex::new(Log::new(Vec::new(), rules)));
        let shared: SharedObserver = log.clone();
        let mut bus = Bus::new(Device::new(Box::new(UsbRelay::new(2)))).with_observer(shared);
        let setup = |request_type, request, value, length| Setup {
            request_type,
            request,
            value,
            index: 0,
            length,
        };
        bus.notice(Notice::Section("run"));
        bus.control(setup(0x00, 9, 1, 0), &[]);
        bus.control(setup(0xa1, 1, 0x0300, 8), &[]);
        bus.notice(Notice::Renumeration { vendor: 0x1234, product: 0xabcd });
        let log = log.lock().unwrap();
        String::from_utf8(log.get_ref().clone()).unwrap()
    }

    #[test]
    fn a_session_reads_as_the_goldens_do() {
        assert_eq!(
            relay_log(Rules::default()),
            "# section run\n\
             ctl_out 00 09 0001 0000 - -> ok\n\
             ctl_in 21 01 0300 0000 8 -> 4142434445000000\n\
             disconnect\n\
             connect 1234:abcd\n"
        );
    }

    #[test]
    fn rules_apply_to_control_data() {
        let text = relay_log(Rules::digest_above(2));
        assert!(text.contains("ctl_in 21 01 0300 0000 8 -> @8:"), "{text}");
    }

    #[test]
    fn undefined_bytes_show_as_xx() {
        let r = Reply::ack();
        let mid = 1..3;
        let head = 0..2;
        assert_eq!(
            transfer_line_undefined(
                BULK,
                0x02,
                0,
                &[1, 2, 3, 4],
                std::slice::from_ref(&mid),
                &r,
                None
            ),
            "bulk_out 02 01xxxx04 -> 4"
        );
        let long = vec![7u8; DIGEST_ABOVE + 2];
        let mut zeroed = long.clone();
        zeroed[..2].fill(0);
        assert_eq!(
            data_undefined(&long, std::slice::from_ref(&head), Some(DIGEST_ABOVE)),
            format!("{}/xx=0-1", data(&zeroed, Some(DIGEST_ABOVE)))
        );
        assert_eq!(data_undefined(&[5], &[], None), "05");
    }

    #[test]
    fn lines_follow_the_fixture_dialect() {
        let s = Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 8 };
        assert_eq!(
            control_line(s, &[], &Reply::Data(vec![0x41, 0x42]), None),
            "ctl_in 21 01 0300 0000 8 -> 4142"
        );
        let s = Setup { request_type: 0x21, request: 9, value: 0x0300, index: 0, length: 2 };
        assert_eq!(
            control_line(s, &[0xff, 1], &Reply::ack(), None),
            "ctl_out 21 09 0300 0000 ff01 -> ok"
        );
        assert_eq!(
            control_line(s, &[0xff, 1], &Reply::Stall, None),
            "ctl_out 21 09 0300 0000 ff01 -> err stall"
        );
        assert_eq!(
            transfer_line(INTERRUPT, 0x81, 64, &[], &Reply::Data(vec![1]), None),
            "int_in 81 64 -> 01"
        );
        assert_eq!(
            transfer_line(BULK, 0x02, 0, &[1, 2], &Reply::ack(), None),
            "bulk_out 02 0102 -> 2"
        );
        // Long data as its digest, the fixtures' form, when asked for.
        let long = vec![0u8; DIGEST_ABOVE + 1];
        assert_eq!(data(&long, None).len(), 2 * (DIGEST_ABOVE + 1));
        assert_eq!(data(&long[..DIGEST_ABOVE], Some(DIGEST_ABOVE)), "00".repeat(DIGEST_ABOVE));
        assert_eq!(
            transfer_line(BULK, 0x82, 2048, &[], &Reply::Data(long), Some(DIGEST_ABOVE)),
            format!("bulk_in 82 2048 -> @1025:{}", hex(&Sha256::digest([0u8; 1025])))
        );
    }

    #[test]
    fn every_kind_and_direction_has_its_word() {
        let data = Reply::Data(vec![0xa5, 0xa5]);
        assert_eq!(plain(&transfer(BULK, 0x82, &[], data.clone())), "bulk_in 82 64 -> a5a5");
        assert_eq!(plain(&transfer(INTERRUPT, 0x81, &[], data.clone())), "int_in 81 64 -> a5a5");
        assert_eq!(plain(&transfer(BULK, 0x02, &[1, 2], Reply::ack())), "bulk_out 02 0102 -> 2");
        assert_eq!(plain(&transfer(INTERRUPT, 0x03, &[9], Reply::ack())), "int_out 03 09 -> 1");
        assert_eq!(
            plain(&control(0xa1, 8, &[], Reply::Data(vec![0x41, 0x42]))),
            "ctl_in 21 01 0300 0002 8 -> 4142"
        );
        assert_eq!(
            plain(&control(0x21, 2, &[0xff, 1], Reply::ack())),
            "ctl_out 21 01 0300 0002 ff01 -> ok"
        );
        // Nothing is `-`, on either side of the arrow.
        assert_eq!(plain(&transfer(BULK, 0x82, &[], Reply::ack())), "bulk_in 82 64 -> -");
        assert_eq!(plain(&transfer(BULK, 0x02, &[], Reply::ack())), "bulk_out 02 - -> 0");
        assert_eq!(plain(&control(0x21, 0, &[], Reply::ack())), "ctl_out 21 01 0300 0002 - -> ok");
        assert_eq!(plain(&control(0xa1, 8, &[], Reply::ack())), "ctl_in 21 01 0300 0002 8 -> -");
        // An OUT the device took part of: the bytes it says it took.
        assert_eq!(
            plain(&transfer(BULK, 0x02, &[1, 2, 3], Reply::Data(vec![0, 0]))),
            "bulk_out 02 010203 -> 2"
        );
        // An endpoint the configuration lacks has no type.
        let unknown = Transfer { kind: None, ..transfer(BULK, 0x85, &[], Reply::Stall) };
        assert_eq!(plain(&unknown), "iso_in 85 64 -> err stall");
    }

    #[test]
    fn every_outcome_has_its_line() {
        let stalled = transfer(INTERRUPT, 0x81, &[], Reply::Stall);
        assert_eq!(plain(&stalled), "int_in 81 64 -> err stall");
        assert_eq!(
            plain(&transfer(BULK, 0x02, &[1, 2], Reply::Stall)),
            "bulk_out 02 0102 -> err stall"
        );
        assert_eq!(
            plain(&control(0xa1, 8, &[], Reply::Stall)),
            "ctl_in 21 01 0300 0002 8 -> err stall"
        );
        // Held: nothing until it completes, a comment in verbose mode.
        let held =
            Transfer { outcome: Outcome::Held, ..transfer(INTERRUPT, 0x81, &[], Reply::Nak) };
        assert_eq!(written(false, Rules::default(), |log| log.transfer(&held)), "");
        assert_eq!(
            written(true, Rules::default(), |log| log.transfer(&held)),
            "# seq 7: int_in 81 64 -> nak (held)\n"
        );
        // A transfer that ended while the device still held it.
        let ended = Transfer { outcome: Outcome::Completed, ..held.clone() };
        assert_eq!(
            written(false, Rules::default(), |log| log.transfer(&ended)),
            "int_in 81 64 -> nak\n"
        );
        assert_eq!(plain(&transfer(BULK, 0x02, &[1], Reply::Nak)), "bulk_out 02 01 -> nak");
        // A babble: the whole answer, and what the URB had room for.
        let babble = Transfer {
            outcome: Outcome::Babble,
            requested: 4,
            ..transfer(BULK, 0x82, &[], Reply::Data(vec![1, 2, 3, 4, 5]))
        };
        assert_eq!(
            written(false, Rules::default(), |log| log.transfer(&babble)),
            "bulk_in 82 64 -> 0102030405 (babble: the URB holds 4)\n"
        );
        // An unlinked IN: the bytes that reached the URB.
        let unlinked = Transfer {
            outcome: Outcome::Unlinked,
            requested: 4,
            ..transfer(BULK, 0x82, &[], Reply::Data(vec![1, 2, 3, 4, 5, 6]))
        };
        assert_eq!(plain(&unlinked), "bulk_in 82 64 -> 01020304");
        let fits = Transfer { requested: 64, ..unlinked };
        assert_eq!(plain(&fits), "bulk_in 82 64 -> 010203040506");
    }

    #[test]
    fn notices_and_violations_have_their_lines() {
        let all = |log: &mut Log<Vec<u8>>| {
            log.notice(Notice::Section("capture"));
            log.violation("SET_CONFIGURATION 7: no such configuration");
            log.notice(Notice::Comment("usbip client connected"));
            log.transfer(&transfer(BULK, 0x82, &[], Reply::Data(vec![1])));
            log.notice(Notice::Fired(&Event::Unplug));
            log.notice(Notice::Fired(&Event::Mark("interrupt".to_owned())));
            log.notice(Notice::Renumeration { vendor: 0x04b4, product: 0x8613 });
        };
        assert_eq!(
            written(false, Rules::default(), all),
            "# section capture\n\
             ! violation: SET_CONFIGURATION 7: no such configuration\n\
             bulk_in 82 64 -> 01\n\
             unplug\n\
             interrupt\n\
             disconnect\n\
             connect 04b4:8613\n"
        );
        // Comments in verbose mode only, and from when it is set.
        let text = written(true, Rules::default(), |log| {
            all(log);
            log.set_verbose(false);
            log.notice(Notice::Comment("not written"));
        });
        assert!(text.contains("\n# usbip client connected\nbulk_in"), "{text}");
        assert!(!text.contains("not written"), "{text}");
    }

    #[test]
    fn long_data_is_its_digest_in_every_place() {
        let rules = Rules::digest_above(2);
        let abc = Reply::Data(b"abc".to_vec());
        assert_eq!(data(b"abc", Some(2)), format!("@3:{ABC}"));
        assert_eq!(data(b"abc", Some(3)), "616263", "longer than, not as long as");
        assert_eq!(data(b"abc", None), "616263");
        assert_eq!(data(&[], Some(0)), "-", "nothing has no digest");
        let line = |t: &Transfer| line(t, &rules);
        assert_eq!(
            line(&transfer(BULK, 0x82, &[], abc.clone())),
            format!("bulk_in 82 64 -> @3:{ABC}")
        );
        assert_eq!(
            line(&transfer(INTERRUPT, 0x81, &[], abc.clone())),
            format!("int_in 81 64 -> @3:{ABC}")
        );
        assert_eq!(
            line(&transfer(BULK, 0x02, b"abc", Reply::ack())),
            format!("bulk_out 02 @3:{ABC} -> 3")
        );
        assert_eq!(
            line(&control(0xa1, 8, &[], abc.clone())),
            format!("ctl_in 21 01 0300 0002 8 -> @3:{ABC}")
        );
        assert_eq!(
            line(&control(0x21, 3, b"abc", Reply::ack())),
            format!("ctl_out 21 01 0300 0002 @3:{ABC} -> ok")
        );
        // A babble's whole answer, and an unlinked IN's cut one.
        let babble = Transfer {
            outcome: Outcome::Babble,
            requested: 2,
            ..transfer(BULK, 0x82, &[], abc.clone())
        };
        assert_eq!(line(&babble), format!("bulk_in 82 64 -> @3:{ABC} (babble: the URB holds 2)"));
        let unlinked =
            Transfer { outcome: Outcome::Unlinked, requested: 2, ..transfer(BULK, 0x82, &[], abc) };
        assert_eq!(line(&unlinked), "bulk_in 82 64 -> 6162");
    }

    /// Endpoint 2: the second byte, and the fourth and fifth.
    fn undefined_on_two(ep: u8, data: &[u8]) -> Vec<Range<usize>> {
        if ep == 2 { vec![1..2.min(data.len()), 3..5.min(data.len())] } else { Vec::new() }
    }

    /// Any endpoint: the first byte.
    fn first_byte(_: u8, _: &[u8]) -> Vec<Range<usize>> {
        std::iter::once(0..1).collect()
    }

    #[test]
    fn undefined_bytes_follow_the_rules_of_the_endpoint() {
        let mut rules = Rules { digest_above: None, undefined: Some(undefined_on_two) };
        let out = |kind, endpoint| transfer(kind, endpoint, b"abcdef", Reply::ack());
        assert_eq!(line(&out(BULK, 0x02), &rules), "bulk_out 02 61xx63xxxx66 -> 6");
        assert_eq!(line(&out(INTERRUPT, 0x02), &rules), "int_out 02 61xx63xxxx66 -> 6");
        assert_eq!(line(&out(BULK, 0x03), &rules), "bulk_out 03 616263646566 -> 6");
        // OUT data on an endpoint only: not a control transfer's, not an IN's.
        assert_eq!(
            line(&control(0x21, 6, b"abcdef", Reply::ack()), &rules),
            "ctl_out 21 01 0300 0002 616263646566 -> ok"
        );
        assert_eq!(
            line(&transfer(BULK, 0x82, &[], Reply::Data(b"abcdef".to_vec())), &rules),
            "bulk_in 82 64 -> 616263646566"
        );
        // In a digest they count as 0, and their ranges follow it, the
        // ends included: SHA-256 of 61 00 63 00 00 66.
        rules.digest_above = Some(5);
        assert_eq!(
            line(&out(BULK, 0x02), &rules),
            "bulk_out 02 \
             @6:183be29d65d80e6585d12fa667c287b9a0313ebf782d4efc050bc53ea68b852d/xx=1-1,3-4 -> 6"
        );
        let first = 0..1;
        assert_eq!(
            data_undefined(b"abc", std::slice::from_ref(&first), Some(2)),
            "@3:4075b6a68556aaa03188190d906199743692269dd8556b034c418f194a70e188/xx=0-0"
        );
        assert_eq!(
            data_undefined(b"abcdef", &[0..1, 3..5], Some(2)),
            "@6:2b509ac4d6a73441a5f563f36e4d3f55c3246b52b1fbf9c906b6c87bff324ec0/xx=0-0,3-4"
        );
        // Without ranges for it, a digest is the plain one.
        assert_eq!(line(&out(BULK, 0x03), &rules).len(), "bulk_out 03 @6: -> 6".len() + 64);
    }

    #[test]
    fn the_rules_change_while_the_log_runs() {
        let t = transfer(BULK, 0x82, &[], Reply::Data(b"abc".to_vec()));
        let text = written(false, Rules::default(), |log| {
            log.transfer(&t);
            log.set_rules(Rules::digest_above(2));
            assert_eq!(log.rules().digest_above, Some(2));
            log.transfer(&t);
            log.set_rules(Rules::default());
            log.transfer(&t);
        });
        assert_eq!(
            text,
            format!(
                "bulk_in 82 64 -> 616263\nbulk_in 82 64 -> @3:{ABC}\nbulk_in 82 64 -> 616263\n"
            )
        );
    }

    #[test]
    fn an_isochronous_transfer_is_a_line_with_its_packets() {
        let packet = |actual, status| IsoPacket { actual, status };
        let ok = |actual| packet(actual, Status::Ok);
        let read = Transfer {
            requested: 48,
            offered: 48,
            packets: vec![
                ok(1),
                ok(2),
                ok(3),
                packet(0, Status::Missed),
                ok(5),
                packet(8, Status::Babble),
            ],
            ..transfer(
                ISOCHRONOUS,
                0x83,
                &[],
                Reply::Data(vec![1, 2, 2, 3, 3, 3, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6]),
            )
        };
        assert_eq!(
            plain(&read),
            "iso_in 83 48 -> 01020203030305050505050606060606060606 \
             (packets: 1 2 3 missed 5 8/babble)"
        );
        // Packets alike in a row are said once, and a packet of nothing
        // is not a missed one.
        let alike = Transfer {
            packets: vec![
                ok(2),
                ok(2),
                ok(2),
                packet(0, Status::Missed),
                packet(0, Status::Missed),
                ok(0),
                ok(2),
            ],
            ..transfer(ISOCHRONOUS, 0x83, &[], Reply::Data(vec![9; 8]))
        };
        assert_eq!(plain(&alike), "iso_in 83 64 -> 0909090909090909 (packets: 2*3 missed*2 0 2)");
        // OUT: the data, the bytes the device took, the packets.
        let write = Transfer {
            packets: vec![ok(2), ok(0), ok(1)],
            ..transfer(ISOCHRONOUS, 0x04, &[1, 2, 3], Reply::ack())
        };
        assert_eq!(plain(&write), "iso_out 04 010203 -> 3 (packets: 2 0 1)");
        let refused = Transfer {
            packets: vec![packet(0, Status::Missed), ok(1)],
            ..transfer(ISOCHRONOUS, 0x04, &[1, 2, 3], Reply::ack())
        };
        assert_eq!(plain(&refused), "iso_out 04 010203 -> 1 (packets: missed 1)");
        // The rules apply to the transfer's whole data.
        let rules = Rules { digest_above: Some(2), undefined: Some(first_byte) };
        assert_eq!(
            line(&write, &rules),
            "iso_out 04 \
             @3:e9969bdc67747c3797d62fd1c3e4277269a0945fdb999e8ccc6c7bfb4eeb06bd/xx=0-0 -> 3 \
             (packets: 2 0 1)"
        );
        assert_eq!(
            line(&Transfer { reply: Reply::Data(b"abc".to_vec()), ..read }, &rules),
            format!("iso_in 83 48 -> @3:{ABC} (packets: 1 2 3 missed 5 8/babble)")
        );
    }

    #[test]
    fn a_microphone_with_dropouts_reads_as_its_packets() {
        let options = [("silent".to_owned(), "3".to_owned())];
        let log = Arc::new(Mutex::new(Log::new(Vec::new(), Rules::digest_above(DIGEST_ABOVE))));
        let shared: SharedObserver = log.clone();
        let device = Device::new(uac_mic::build(&options).unwrap());
        let mut bus = Bus::new(device).with_observer(shared);
        enumerate(&mut bus).unwrap();
        bus.iso_read(uac_mic::EP_AUDIO, &[96; 7]);
        bus.iso_read(uac_mic::EP_AUDIO, &[96; 16]);
        let log = log.lock().unwrap();
        let text = String::from_utf8_lossy(log.get_ref());
        let lines: Vec<&str> = text.lines().filter(|l| l.starts_with("iso_in")).collect();
        let (first, packets) = lines[0].split_once(" (").unwrap();
        assert_eq!(packets, "packets: 96*2 missed 96*2 missed 96)");
        assert_eq!(first.len(), "iso_in 81 672 -> ".len() + 5 * 96 * 2, "480 bytes, in hex");
        // Eleven packets came, more than a kilobyte: their digest.
        assert!(lines[1].starts_with("iso_in 81 1536 -> @1056:"), "{}", lines[1]);
        assert!(lines[1].ends_with(
            " (packets: 96 missed 96*2 missed 96*2 missed 96*2 missed 96*2 missed 96*2)"
        ));
    }

    /// Counts the lines and the flushes that reach it.
    #[derive(Default)]
    struct Counted {
        lines: usize,
        flushed_at: Vec<usize>,
    }

    impl Write for Counted {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            // A line is never written with a newline inside it.
            self.lines += usize::from(bytes.ends_with(b"\n"));
            Ok(bytes.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            self.flushed_at.push(self.lines);
            Ok(())
        }
    }

    #[test]
    fn every_line_is_flushed() {
        let mut log = Log::new(Counted::default(), Rules::default());
        log.notice(Notice::Section("run"));
        log.transfer(&transfer(BULK, 0x82, &[], Reply::ack()));
        log.violation("no");
        log.notice(Notice::Renumeration { vendor: 1, product: 2 });
        assert_eq!(log.get_ref().flushed_at, [1, 2, 3, 4, 5]);
    }
}
