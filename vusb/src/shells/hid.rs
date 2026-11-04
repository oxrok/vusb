// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The HID interface shell: the class descriptor, the report
//! descriptor fetch and the class requests (`GET_REPORT`,
//! `SET_REPORT`, idle, protocol). A model implements [`HidReports`]
//! for what its reports mean.

use crate::descriptor::{self, truncate};
use crate::model::{CLASS, RECIPIENT_INTERFACE, Reply, STANDARD, Setup, Violations};

/// Report kinds, the high byte of `wValue` in `GET_REPORT` and
/// `SET_REPORT`.
pub const INPUT: u8 = 1;
/// Output report.
pub const OUTPUT: u8 = 2;
/// Feature report.
pub const FEATURE: u8 = 3;

const GET_DESCRIPTOR: u8 = 6;
const GET_REPORT: u8 = 1;
const GET_IDLE: u8 = 2;
const GET_PROTOCOL: u8 = 3;
const SET_REPORT: u8 = 9;
const SET_IDLE: u8 = 0x0a;
const SET_PROTOCOL: u8 = 0x0b;

/// The reports of a HID device.
pub trait HidReports {
    /// `GET_REPORT` of `kind` and `id`, `length` bytes asked.
    fn get_report(&mut self, kind: u8, id: u8, length: usize, v: &mut Violations) -> Reply;
    /// `SET_REPORT` of `kind` and `id` with `data`.
    fn set_report(&mut self, kind: u8, id: u8, data: &[u8], v: &mut Violations) -> Reply;
}

/// One HID interface.
#[derive(Clone, Debug)]
pub struct HidInterface {
    /// `bInterfaceNumber`.
    pub number: u8,
    /// The report descriptor.
    pub report_descriptor: Vec<u8>,
    /// `bcdHID` of the class descriptor.
    pub version: u16,
    idle: u8,
    protocol: u8,
}

impl HidInterface {
    /// A HID interface with this report descriptor.
    pub fn new(number: u8, report_descriptor: Vec<u8>) -> HidInterface {
        HidInterface { number, report_descriptor, version: 0x0111, idle: 0, protocol: 1 }
    }

    /// The same interface with another `bcdHID` than the default 1.11.
    #[must_use]
    pub fn with_version(mut self, version: u16) -> HidInterface {
        self.version = version;
        self
    }

    /// The HID class descriptor (no country, one report descriptor), which
    /// goes after the interface descriptor.
    pub fn descriptor(&self) -> [u8; 9] {
        let len = (self.report_descriptor.len() as u16).to_le_bytes();
        let version = self.version.to_le_bytes();
        [9, descriptor::HID, version[0], version[1], 0, 1, descriptor::HID_REPORT, len[0], len[1]]
    }

    /// Back to the defaults: no idle, report protocol.
    pub fn reset(&mut self) {
        self.idle = 0;
        self.protocol = 1;
    }

    /// Answers a request addressed to this interface, or `None` when
    /// it is not one.
    pub fn control(
        &mut self,
        reports: &mut dyn HidReports,
        setup: Setup,
        data: &[u8],
        v: &mut Violations,
    ) -> Option<Reply> {
        if setup.recipient() != RECIPIENT_INTERFACE || (setup.index & 0xff) as u8 != self.number {
            return None;
        }
        let kind = (setup.value >> 8) as u8;
        let id = setup.value as u8;
        let reply = match (setup.kind(), setup.request, setup.is_in()) {
            (STANDARD, GET_DESCRIPTOR, true) => match kind {
                descriptor::HID => Reply::Data(truncate(&self.descriptor(), setup.length)),
                descriptor::HID_REPORT => {
                    Reply::Data(truncate(&self.report_descriptor, setup.length))
                }
                other => {
                    v.report(format!("GET_DESCRIPTOR of type {other:#04x} on the HID interface"));
                    Reply::Stall
                }
            },
            (CLASS, GET_REPORT, true) => {
                if setup.length == 0 {
                    v.report("GET_REPORT with wLength 0");
                }
                reports.get_report(kind, id, usize::from(setup.length), v)
            }
            (CLASS, GET_IDLE, true) => Reply::Data(vec![self.idle]),
            (CLASS, GET_PROTOCOL, true) => Reply::Data(vec![self.protocol]),
            (CLASS, SET_REPORT, false) => {
                if data.len() != usize::from(setup.length) {
                    v.report(format!(
                        "SET_REPORT carries {} bytes, wLength says {}",
                        data.len(),
                        setup.length
                    ));
                }
                reports.set_report(kind, id, data, v)
            }
            (CLASS, SET_IDLE, false) => {
                self.idle = kind;
                Reply::ack()
            }
            (CLASS, SET_PROTOCOL, false) => {
                self.protocol = id;
                Reply::ack()
            }
            _ => return None,
        };
        Some(reply)
    }
}
