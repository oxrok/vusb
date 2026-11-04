// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Virtual USB devices.
//!
//! A device is a [`model::DeviceModel`]: descriptors, control
//! transfers, OUT and IN transfers, reset. Plain Rust with no I/O, so
//! the same model runs in a unit test, on a host here and, later, on a
//! microcontroller. [`device::Device`] wraps a model with the standard
//! requests every USB device answers (descriptors, configuration,
//! status, halts) and the checks on what the host sends. [`shells`]
//! carry what every device of a kind shares (HID, CDC-ACM, FTDI, the
//! EZ-USB FX2 boot loader). [`bus::Bus`] is the host controller around
//! a device: it holds the transfers the device NAKs, keeps each
//! endpoint's order, and cuts a babble, whatever carries the transfers;
//! a test drives it directly ([`host::enumerate`]). [`transport::usbip`]
//! is the USB/IP server the Linux `vhci-hcd` module connects to, and
//! [`attach::vhci`] attaches and detaches through its sysfs files, so
//! no `usbip` tool is needed. Everything the host does goes to the
//! [`observer::Observer`]s the bus is given: one call per completed
//! transfer, one per violation of the protocol as the model defines
//! it, and notices such as a renumeration the model asks for or an
//! armed [`arm::Event`] firing.
//!
//! A [`session::Session`] puts it together for a program: it serves a
//! model, attaches it, and takes commands from a [`control`] socket, so
//! that `vusb-ctl` or any client can reset it, unplug it, arm faults,
//! and watch what the host does while it runs.
//!
//! Only Linux can attach the device; the models, shells and the
//! protocol code build and test everywhere.

#![forbid(unsafe_code)]

pub mod arm;
pub mod bus;
pub mod descriptor;
pub mod device;
pub mod examples;
pub mod host;
pub mod model;
pub mod observer;
pub mod options;
pub mod shells;

#[cfg(feature = "usbip")]
pub mod attach;
#[cfg(feature = "control")]
pub mod control;
#[cfg(feature = "control")]
pub mod session;
#[cfg(feature = "usbip")]
pub mod transport;

use std::fmt;

/// The README's example, compiled and run with the doctests.
#[cfg(doctest)]
#[doc = include_str!("../../README.md")]
struct ReadmeDoctests;

/// What can go wrong.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// An I/O error of a transport or of the attach path.
    Io(std::io::Error),
    /// An option or argument that cannot be used: a model option a model
    /// or shell refuses, an `arm` specification that does not parse.
    Invalid(String),
    /// The peer left the protocol: a USB/IP import refused, a frame that
    /// is not one.
    Protocol(String),
    /// Attaching or detaching the device failed: `vhci-hcd` missing,
    /// no free port, no root, a node that keeps its mode.
    Attach(String),
    /// Anything else, as a message.
    Other(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => e.fmt(f),
            Error::Invalid(m) | Error::Protocol(m) | Error::Attach(m) | Error::Other(m) => {
                f.write_str(m)
            }
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Error {
        Error::Io(e)
    }
}

impl From<String> for Error {
    fn from(s: String) -> Error {
        Error::Other(s)
    }
}

impl From<&str> for Error {
    fn from(s: &str) -> Error {
        Error::Other(s.to_owned())
    }
}

/// Result with [`Error`].
pub type Result<T> = std::result::Result<T, Error>;
