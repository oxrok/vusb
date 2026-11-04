// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A USB device that is only Rust code. Test your driver without the
//! hardware, in CI, and with fault injection.
//!
//! A device is a [`DeviceModel`](model::DeviceModel): its descriptors
//! and how it answers control, OUT and IN transfers. It is plain Rust
//! with no I/O, so the same model runs in a unit test, where
//! [`Bus`](bus::Bus) drives it directly, and on a Linux machine, where
//! [`Session`](session::Session) serves it over USB/IP and attaches it
//! through `vhci-hcd`, so that `lsusb`, libusb, hidraw and the kernel's
//! class drivers see a real device. Either way every transfer goes to
//! [observers](observer), and every departure from the USB protocol,
//! or from the device's own, is reported.
//!
//! # A device in a test
//!
//! The CDC-ACM [shell](shells::cdc) handles the USB side of a serial
//! port, so the model only sees bytes. No kernel is involved: the bus
//! enumerates the device and moves data as a host would.
//!
//! ```
//! use vusb::bus::Bus;
//! use vusb::device::Device;
//! use vusb::host::enumerate;
//! use vusb::shells::cdc::{Cdc, Faults, Identity, Port, Serial};
//!
//! struct Shout;
//!
//! impl Serial for Shout {
//!     fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
//!         port.send(&data.to_ascii_uppercase());
//!     }
//! }
//!
//! # fn main() -> vusb::Result<()> {
//! let id = Identity::new(0x1209, 0x0001, "Maker", "Shouter");
//! let model = Cdc::new("shout", &id, Faults::default(), Shout);
//! let mut bus = Bus::new(Device::new(Box::new(model)));
//!
//! let found = enumerate(&mut bus)?;
//! assert_eq!(found.product, "Shouter");
//! bus.write(2, b"hello");
//! assert_eq!(bus.read(2, 64).data, b"HELLO");
//! # Ok(())
//! # }
//! ```
//!
//! # The same device on the machine
//!
//! A [`session::Builder`] takes a function that builds the model, so
//! that a `reset` from the control socket starts a fresh one. With
//! `attach` the device appears on the machine (Linux, root, and
//! `modprobe vhci-hcd`); the control socket is what `vusb-ctl` talks
//! to.
//!
//! ```no_run
//! use vusb::control::Address;
//! use vusb::examples::cdc_echo;
//! use vusb::session::Builder;
//!
//! # fn main() -> vusb::Result<()> {
//! let report = Builder::new(Box::new(|options| cdc_echo::build(options)))
//!     .attach(true)
//!     .control(Address::parse("/tmp/echo.sock"))
//!     .start()?
//!     .run()?;
//! println!("violations: {}", report.violations);
//! # Ok(())
//! # }
//! ```
//!
//! # How the parts fit
//!
//! From the host down to your code:
//!
//! | Layer | What it does |
//! |---|---|
//! | [`transport::usbip`], [`attach::vhci`] | The USB/IP server the kernel connects to, and the sysfs attach. Linux only. |
//! | [`session`], [`control`] | A served device with a control socket: reset, options, unplug, arm, live streams. |
//! | [`bus::Bus`] | The host controller: holds a transfer the device NAKs and offers it again, keeps each endpoint's order, cuts a babble, completes an unlink. Tests and transports all drive this one. |
//! | [`device::Device`] | The standard requests every device answers, and the checks on what the host does. |
//! | [`shells`] | What every device of a kind shares: [HID](shells::hid), [CDC-ACM](shells::cdc), [FTDI](shells::ftdi), the [EZ-USB](shells::ezusb) boot loader. |
//! | [`model::DeviceModel`] | Your device: what the bytes mean. |
//!
//! Beside them, [`observer`] sees every transfer, [`arm`] fires an
//! event at a chosen point of the traffic, [`host::enumerate`] does
//! what a host does with a new device, and [`examples`] has a short
//! model on each shell.
//!
//! # Writing a model
//!
//! A model answers each transfer with [`Reply::Data`](model::Reply),
//! [`Reply::Nak`](model::Reply) or [`Reply::Stall`](model::Reply), and
//! never waits: asked for data it does not have, it answers NAK and
//! returns, and the bus asks again later. So a model has no threads, no
//! sockets and no clock. One that needs time counts the host's offers
//! instead ([`shells::ftdi::Countdown`]), so a run does not depend on
//! how fast the machine is.
//!
//! Most devices start from a shell and implement its smaller trait:
//! [`shells::cdc::Serial`] for a serial port, [`shells::hid::HidReports`]
//! for a HID device, [`shells::ftdi::FtdiPorts`] for an FTDI chip. A
//! device that fits no shell implements [`model::DeviceModel`] itself;
//! its documentation has a complete bare vendor device. Descriptors are
//! built with [`descriptor`].
//!
//! A model reports what the host does wrong through the
//! [`Violations`](model::Violations) it is handed: a request the device
//! does not have, a report of the wrong length, a command out of
//! sequence. [`device::Device`] reports what breaks USB itself. Both
//! reach the observers, and a session counts them for its exit status.
//!
//! # Faults
//!
//! Faults are of two kinds. Options are the model's own, `key=value`
//! pairs it is built with ([`options`]): the shells take `stall=`,
//! `in-stall=N` and the like through their `Faults::option`, and a model
//! adds its own. They take effect from a fresh model, which is what a
//! session's `reset` builds. An [`Arm`](arm::Arm) picks a point in a
//! run instead, the Nth transfer of an endpoint or the moment the model
//! marks, and fires once there: an unplug, or a named mark whoever
//! drives the run acts on.
//!
//! # Watching
//!
//! An [`Observer`](observer::Observer) gets every
//! [`Transfer`](observer::Transfer) with the length the device saw, the
//! reply before the host cut it, and how it ended; every violation; and
//! notices on the run. [`observer::Recorder`] keeps them for a test,
//! and [`Transfer::line`](observer::Transfer::line) is one line of a
//! transaction log.
//!
//! # Platforms and features
//!
//! Attaching a device needs Linux. The models, shells, bus and control
//! protocol build and test on Linux, macOS and Windows. The `usbip`
//! feature (on by default) holds the server and the attach path, and
//! `control` (on by default, needs `usbip`) the session and its socket;
//! neither is needed to write and unit test a model. Isochronous
//! transfers, hubs and super speed are not supported yet.
//!
//! The command line tools, `vusb` and `vusb-ctl`, are the `vusb-cli`
//! crate; the control protocol is documented in the repository's
//! `docs/control.md`.

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
#[doc = include_str!("../README.md")]
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
