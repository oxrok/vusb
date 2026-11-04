// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Watching what the host does to a device.
//!
//! A host calls its [`Observer`]s after every transfer it offers to the
//! device, for every violation the device or the model reports, and
//! with notices on the run (connections, held transfers, a
//! renumeration the model asks for, an armed event firing). An
//! observer writes a transaction log, counts violations for an exit
//! status, or streams the transfers to whoever watches; [`Recorder`]
//! keeps them in memory, for tests.
//!
//! Observers run under the host's device lock: they must not block,
//! and whatever is slow (a socket, a file) gets the data through a
//! channel.

use std::fmt;
use std::sync::{Arc, Mutex};

use crate::arm::Event;
use crate::model::{Reply, Setup};

/// One transfer the host submitted, as it was offered to the device,
/// with its outcome.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Transfer {
    /// The transport's sequence number of the URB (USB/IP's `seqnum`).
    pub seqnum: u32,
    /// The endpoint address, direction bit included; 0 for a control
    /// transfer.
    pub endpoint: u8,
    /// The transfer type as the active configuration declares the
    /// endpoint ([`crate::model::CONTROL`] and the others), or `None`
    /// when the configuration lacks it.
    pub kind: Option<u8>,
    /// The setup packet of a control transfer.
    pub setup: Option<Setup>,
    /// Device to host (for a control transfer, the setup's direction).
    pub is_in: bool,
    /// The length the URB asked for.
    pub requested: usize,
    /// The length the device was offered: for a bulk IN, `requested`
    /// rounded up to whole packets, which is what the device sees on
    /// the bus.
    pub offered: usize,
    /// The OUT payload; empty for IN.
    pub data: Vec<u8>,
    /// What the device answered, before the host cut it to the URB.
    pub reply: Reply,
    /// How the URB ended, or that it has not.
    pub outcome: Outcome,
}

/// How a transfer ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Outcome {
    /// Completed with the reply: data (cut to what the URB holds), an
    /// accepted OUT, or a STALL.
    Completed,
    /// The device answered NAK: the URB is held and offered again until
    /// it completes or is unlinked. Reported at the first offer only.
    Held,
    /// More data than the URB holds: completed with the bytes that fit
    /// and `-EOVERFLOW`, as a host controller reports a babbling device.
    Babble,
    /// Unlinked by the host while held, and completed with the bytes the
    /// device held for it (the reply) and `-ECONNRESET`.
    Unlinked,
}

/// Something that happened on the run other than a transfer.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Notice<'a> {
    /// A comment on what the host did: connections, held and queued
    /// transfers, unlinks, transfers dropped after an unplug.
    Comment(&'a str),
    /// The model asks to leave the bus and come back with the
    /// descriptors it has now (its new vendor and product ids).
    Renumeration {
        /// `idVendor` after the renumeration.
        vendor: u16,
        /// `idProduct` after it.
        product: u16,
    },
    /// An armed event fired.
    Fired(&'a Event),
}

impl fmt::Display for Notice<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Notice::Comment(c) => f.write_str(c),
            Notice::Renumeration { vendor, product } => {
                write!(f, "renumeration {vendor:04x}:{product:04x}")
            }
            Notice::Fired(e) => write!(f, "event {}", e.word()),
        }
    }
}

/// Sees what the host does. Every method does nothing by default.
pub trait Observer: Send {
    /// A transfer was offered to the device and completed, or is held.
    fn transfer(&mut self, transfer: &Transfer) {
        let _ = transfer;
    }

    /// The host left the protocol as the device or the model defines
    /// it. Reported before the transfer that caused it.
    fn violation(&mut self, message: &str) {
        let _ = message;
    }

    /// A notice on the run.
    fn notice(&mut self, notice: Notice<'_>) {
        let _ = notice;
    }
}

/// An observer shared between the host and whoever else holds it (to
/// read a count, to write a marker into a log).
pub type SharedObserver = Arc<Mutex<dyn Observer>>;

/// Keeps everything it sees, for tests.
#[derive(Debug, Default)]
pub struct Recorder {
    /// The transfers, in order.
    pub transfers: Vec<Transfer>,
    /// The violations, in order.
    pub violations: Vec<String>,
    /// The notices, as their [`fmt::Display`] text.
    pub notices: Vec<String>,
}

impl Observer for Recorder {
    fn transfer(&mut self, transfer: &Transfer) {
        self.transfers.push(transfer.clone());
    }

    fn violation(&mut self, message: &str) {
        self.violations.push(message.to_owned());
    }

    fn notice(&mut self, notice: Notice<'_>) {
        self.notices.push(notice.to_string());
    }
}
