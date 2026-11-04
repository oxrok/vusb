// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The dcttech `USBRelay` board, the smallest useful device: a V-USB
//! HID device `16c0:05df`, vendor string `www.dcttech.com`, product
//! `USBRelayN` where N is the relay count, with one eight byte feature
//! report. An example of a model on the HID shell, with options and
//! faults; the crate's own tests drive it.
//!
//! Reading the feature report gives the board's serial number in the
//! first five bytes (`ABCDE` on the emulator) and the relay state as a
//! bit mask in the last. Writing it switches relays; the first byte is
//! the command and the second the relay number, 1 based:
//!
//! | byte 0 | meaning |
//! |---|---|
//! | `0xfe` | all relays on |
//! | `0xfc` | all relays off |
//! | `0xff` | relay `byte 1` on |
//! | `0xfd` | relay `byte 1` off |
//!
//! The pedantic checks follow what the C driver and the firmware
//! agree on: reports are eight bytes with report id 0, commands are
//! one of the four, a relay number is within the board's count. The
//! input and output reports the report descriptor declares are never
//! used: a `GET_REPORT` of the input report stalls, as the firmware's
//! callback returns no data, and an output report is accepted and
//! flagged.
//!
//! Options for the driver's error paths, all off by default:
//! `serial=TEXT` or `serial=0xHEX` sets the five serial bytes the
//! report carries (the C driver skips a board whose serial has a
//! character outside 0x20 to 0x7e), `stall-get=N` STALLs the Nth
//! `GET_REPORT` of the feature report and `short-get=N` answers it
//! with its first four bytes only, `stall-set=N` STALLs the Nth
//! `SET_REPORT` of the feature report, which the board then has not
//! taken (each N counts from 1, once, as firmware that refuses one
//! request does; the kernel's own requests at the attach come before
//! a case's model, so do not count).
//!
//! The device is full speed with a 64 byte endpoint 0 and a 64 byte
//! interrupt IN endpoint polled every 2 ms; a real board is a low speed
//! V-USB device with 8 byte endpoints. The driver takes the board's
//! serial number from the report, not from the USB serial string.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::shells::hid::{self, HidInterface, HidReports};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "hid-relay";
/// One line about the device and its options, for a host that lists its models.
pub const ABOUT: &str = "dcttech USBRelay board (HID, 16c0:05df); relays=N (1 to 8, default 2), \
     serial=TEXT|0xHEX (5 bytes), faults: stall-get=N, short-get=N, stall-set=N";

/// A board from the `-o` options: `relays=N`, 1 to 8, default 2;
/// `serial=` five bytes as text or `0x` and ten hex digits; the faults
/// `stall-get=N`, `short-get=N`, `stall-set=N` (N from 1).
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut relays = 2;
    let mut serial = *SERIAL;
    let mut faults = Faults::default();
    let nth = |k: &str, v: &str| -> Result<Option<u32>> {
        v.parse()
            .ok()
            .filter(|n| *n > 0)
            .map(Some)
            .ok_or_else(|| Error::Invalid(format!("{k}={v}: N from 1")))
    };
    for (k, v) in options {
        match k.as_str() {
            "relays" => {
                relays = v
                    .parse()
                    .ok()
                    .filter(|n| (1..=8).contains(n))
                    .ok_or_else(|| Error::Invalid(format!("relays={v}: 1 to 8")))?;
            }
            "serial" => serial = parse_serial(v)?,
            "stall-get" => faults.stall_get = nth(k, v)?,
            "short-get" => faults.short_get = nth(k, v)?,
            "stall-set" => faults.stall_set = nth(k, v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let mut dev = UsbRelay::new(relays);
    dev.board.serial = serial;
    dev.board.faults = faults;
    Ok(Box::new(dev))
}

/// Five serial bytes: text, or `0x` and ten hex digits.
fn parse_serial(v: &str) -> Result<[u8; 5]> {
    let bytes: Option<Vec<u8>> = if let Some(hex) = v.strip_prefix("0x") {
        (0..hex.len())
            .step_by(2)
            .map(|i| hex.get(i..i + 2).and_then(|b| u8::from_str_radix(b, 16).ok()))
            .collect()
    } else {
        Some(v.as_bytes().to_vec())
    };
    bytes.and_then(|b| <[u8; 5]>::try_from(b).ok()).ok_or_else(|| {
        Error::Invalid(format!("serial={v}: five bytes, as text or 0x and ten hex digits"))
    })
}

/// The faults of the `-o` options; none by default.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// This feature `GET_REPORT` (from 1) gets a STALL.
    pub stall_get: Option<u32>,
    /// This feature `GET_REPORT` is answered with four bytes.
    pub short_get: Option<u32>,
    /// This feature `SET_REPORT` (from 1) gets a STALL and is not taken.
    pub stall_set: Option<u32>,
}

/// The USB vendor id (shared by many V-USB projects).
pub const VENDOR_ID: u16 = 0x16c0;
/// The USB product id.
pub const PRODUCT_ID: u16 = 0x05df;
/// The vendor string the driver checks.
pub const VENDOR_STRING: &str = "www.dcttech.com";
/// The serial number the emulator reports.
pub const SERIAL: &[u8; 5] = b"ABCDE";
/// The feature report length.
pub const REPORT_BYTES: usize = 8;

const ALL_ON: u8 = 0xfe;
const ALL_OFF: u8 = 0xfc;
const ONE_ON: u8 = 0xff;
const ONE_OFF: u8 = 0xfd;

/// The vendor page report descriptor: input, output
/// and feature reports of eight bytes, no report ids.
pub const REPORT_DESCRIPTOR: [u8; 47] = [
    0x06, 0x00, 0xff, // usage page, vendor defined
    0x09, 0x01, // usage 1
    0xa1, 0x01, // collection, application
    0x09, 0x02, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x08, 0x81,
    0x02, // input, 8 bytes
    0x09, 0x03, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x08, 0x91,
    0x02, // output, 8 bytes
    0x09, 0x04, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x08, 0xb1,
    0x02, // feature, 8 bytes
    0xc0, // end collection
];

/// The relays behind the reports.
#[derive(Clone, Debug)]
pub struct Board {
    /// Relay count, 1 to 8.
    pub relays: u8,
    /// One bit per relay, bit 0 is `R1`.
    pub state: u8,
    /// The serial number in the report.
    pub serial: [u8; 5],
    /// The faults.
    pub faults: Faults,
    /// Feature reports read and written so far, for the faults.
    gets: u32,
    sets: u32,
}

impl Board {
    /// The feature report as the board reports it.
    pub fn report(&self) -> [u8; REPORT_BYTES] {
        let mut r = [0u8; REPORT_BYTES];
        r[..5].copy_from_slice(&self.serial);
        r[7] = self.state;
        r
    }

    /// Applies a feature report written by the host, the firmware's
    /// way: unknown commands are ignored, relay numbers 1 to 8 are
    /// honoured whatever the board's count.
    pub fn command(&mut self, report: &[u8], v: &mut Violations) {
        if report.len() < 2 {
            v.report(format!("relay command of {} bytes, needs 2", report.len()));
            return;
        }
        let relay = report[1];
        let mask = if (1..=8).contains(&relay) { 1u8 << (relay - 1) } else { 0 };
        match report[0] {
            ALL_ON => self.state = 0xff,
            ALL_OFF => self.state = 0,
            ONE_ON | ONE_OFF => {
                if relay < 1 || relay > self.relays {
                    v.report(format!("relay number {relay}, the board has 1 to {}", self.relays));
                }
                if report[0] == ONE_ON {
                    self.state |= mask;
                } else {
                    self.state &= !mask;
                }
            }
            other => v.report(format!("relay command {other:#04x} is not fc, fd, fe or ff")),
        }
    }
}

impl HidReports for Board {
    fn get_report(&mut self, kind: u8, id: u8, length: usize, v: &mut Violations) -> Reply {
        if kind != hid::FEATURE {
            // The firmware answers no data: a STALL.
            return Reply::Stall;
        }
        if id != 0 {
            v.report(format!("GET_REPORT with report id {id}; the board has none"));
        }
        if length != REPORT_BYTES {
            v.report(format!(
                "GET_REPORT of {length} bytes; the feature report has {REPORT_BYTES}"
            ));
        }
        self.gets += 1;
        if self.faults.stall_get == Some(self.gets) {
            return Reply::Stall;
        }
        let r = self.report();
        if self.faults.short_get == Some(self.gets) {
            return Reply::Data(r[..4].to_vec());
        }
        Reply::Data(r[..length.min(REPORT_BYTES)].to_vec())
    }

    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply {
        if id != 0 {
            v.report(format!("SET_REPORT with report id {id}; the board has none"));
        }
        match kind {
            hid::FEATURE => {
                self.sets += 1;
                if self.faults.stall_set == Some(self.sets) {
                    return Reply::Stall;
                }
                if data.len() != REPORT_BYTES {
                    v.report(format!(
                        "SET_REPORT of {} bytes; the feature report has {REPORT_BYTES}",
                        data.len()
                    ));
                }
                self.command(data, v);
                Reply::ack()
            }
            hid::OUTPUT => {
                v.report("output report to a relay board, which has no use for one");
                Reply::ack()
            }
            other => {
                v.report(format!("SET_REPORT of kind {other}"));
                Reply::Stall
            }
        }
    }
}

/// The emulated board.
#[derive(Clone, Debug)]
pub struct UsbRelay {
    descriptors: Descriptors,
    hid: HidInterface,
    board: Board,
}

impl UsbRelay {
    /// A board with `relays` relays (1 to 8), all off.
    pub fn new(relays: u8) -> UsbRelay {
        let hid = HidInterface::new(0, REPORT_DESCRIPTOR.to_vec());
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 1, [3, 0, 0], 0));
        body.extend_from_slice(&hid.descriptor());
        body.extend_from_slice(&descriptor::endpoint(0x81, 3, 64, 2));
        let config = descriptor::configuration(1, 0, 0xa0, 100, &body);
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: VENDOR_ID,
            product: PRODUCT_ID,
            device: 0x0100,
            strings: [1, 2, 3],
            configurations: 1,
        };
        UsbRelay {
            descriptors: Descriptors {
                speed: Speed::Full,
                device: device.bytes(),
                configs: vec![config],
                strings: vec![
                    VENDOR_STRING.to_owned(),
                    format!("USBRelay{relays}"),
                    "0001".to_owned(),
                ],
            },
            hid,
            board: Board {
                relays,
                state: 0,
                serial: *SERIAL,
                faults: Faults::default(),
                gets: 0,
                sets: 0,
            },
        }
    }

    /// The relay state, bit 0 is `R1`.
    pub fn state(&self) -> u8 {
        self.board.state
    }
}

impl DeviceModel for UsbRelay {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if let Some(reply) = self.hid.control(&mut self.board, setup, data, v) {
            return reply;
        }
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the board answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the board has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, _ep: u8, _max: usize, _v: &mut Violations) -> Reply {
        // The board never sends an input report; the host's poll waits.
        Reply::Nak
    }

    fn reset(&mut self) {
        // Relays keep their state across a bus reset, as on the board.
        self.hid.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feature(request_type: u8, request: u8, length: u16) -> Setup {
        Setup { request_type, request, value: 0x0300, index: 0, length }
    }

    #[test]
    fn switches_relays_as_the_driver_does() {
        let mut dev = UsbRelay::new(2);
        let mut v = Violations::default();
        assert_eq!(
            dev.control(feature(0xa1, 1, 8), &[], &mut v),
            Reply::Data(b"ABCDE\0\0\0".to_vec())
        );
        assert_eq!(
            dev.control(feature(0x21, 9, 8), &[0xff, 2, 0, 0, 0, 0, 0, 0], &mut v),
            Reply::ack()
        );
        assert_eq!(dev.state(), 0x02);
        assert_eq!(
            dev.control(feature(0x21, 9, 8), &[0xfe, 0, 0, 0, 0, 0, 0, 0], &mut v),
            Reply::ack()
        );
        assert_eq!(dev.state(), 0xff);
        assert_eq!(
            dev.control(feature(0x21, 9, 8), &[0xfd, 1, 0, 0, 0, 0, 0, 0], &mut v),
            Reply::ack()
        );
        assert_eq!(dev.state(), 0xfe);
        assert_eq!(
            dev.control(feature(0x21, 9, 8), &[0xfc, 0, 0, 0, 0, 0, 0, 0], &mut v),
            Reply::ack()
        );
        assert_eq!(
            dev.control(feature(0xa1, 1, 8), &[], &mut v),
            Reply::Data(b"ABCDE\0\0\0".to_vec())
        );
        assert!(v.is_empty(), "{v:?}");
        // The report descriptor and the HID descriptor are served.
        let s = Setup { request_type: 0x81, request: 6, value: 0x2200, index: 0, length: 255 };
        assert_eq!(dev.control(s, &[], &mut v), Reply::Data(REPORT_DESCRIPTOR.to_vec()));
        assert_eq!(dev.in_transfer(1, 64, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn flags_what_the_protocol_forbids() {
        let mut dev = UsbRelay::new(2);
        let mut v = Violations::default();
        dev.control(feature(0xa1, 1, 9), &[], &mut v);
        assert_eq!(v.drain().len(), 1, "length 9");
        dev.control(feature(0x21, 9, 8), &[0xff, 3, 0, 0, 0, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "relay 3 on a two relay board");
        assert_eq!(dev.state(), 0x04, "the firmware switches it anyway");
        dev.control(feature(0x21, 9, 8), &[0x12, 0, 0, 0, 0, 0, 0, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "unknown command");
        dev.control(feature(0x21, 9, 2), &[0xfc, 0], &mut v);
        assert_eq!(v.drain().len(), 1, "short report");
        assert_eq!(dev.state(), 0);
        let input = Setup { request_type: 0xa1, request: 1, value: 0x0100, index: 0, length: 8 };
        assert_eq!(dev.control(input, &[], &mut v), Reply::Stall);
        assert!(v.is_empty());
        let output = Setup { request_type: 0x21, request: 9, value: 0x0200, index: 0, length: 8 };
        assert_eq!(dev.control(output, &[0; 8], &mut v), Reply::ack());
        assert_eq!(v.drain().len(), 1, "output report");
    }

    #[test]
    fn faults_and_serial_options() {
        let o = |k: &str, val: &str| vec![(k.to_owned(), val.to_owned())];
        assert!(build(&o("serial", "ABCD")).is_err());
        assert!(build(&o("serial", "0x41424344zz")).is_err());
        assert!(build(&o("stall-get", "0")).is_err());
        let mut v = Violations::default();
        let mut dev = build(&o("serial", "0x4142014445")).unwrap();
        assert_eq!(
            dev.control(feature(0xa1, 1, 8), &[], &mut v),
            Reply::Data(b"AB\x01DE\0\0\0".to_vec())
        );
        let mut dev = build(&o("stall-get", "2")).unwrap();
        assert!(matches!(dev.control(feature(0xa1, 1, 8), &[], &mut v), Reply::Data(_)));
        assert_eq!(dev.control(feature(0xa1, 1, 8), &[], &mut v), Reply::Stall);
        assert!(matches!(dev.control(feature(0xa1, 1, 8), &[], &mut v), Reply::Data(_)));
        let mut dev = build(&o("short-get", "1")).unwrap();
        assert_eq!(dev.control(feature(0xa1, 1, 8), &[], &mut v), Reply::Data(b"ABCD".to_vec()));
        let mut dev = build(&o("stall-set", "1")).unwrap();
        let on = [0xfe, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(dev.control(feature(0x21, 9, 8), &on, &mut v), Reply::Stall);
        assert_eq!(
            dev.control(feature(0xa1, 1, 8), &[], &mut v),
            Reply::Data(b"ABCDE\0\0\0".to_vec()),
            "not taken"
        );
        assert!(v.is_empty(), "{v:?}");
    }
}
