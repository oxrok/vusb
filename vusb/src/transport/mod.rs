// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Transports: the protocols that carry a device to a host.
//!
//! - [`usbip`]: USB/IP, which Linux's `vhci-hcd` speaks, so the device
//!   is on the machine itself.
//! - [`usbredir`]: usbredir, which QEMU's `usb-redir` device speaks, so
//!   the device is inside a virtual machine, on any platform QEMU runs.
//! - [`gadget`]: Linux's Raw Gadget driver, so the device is on a USB
//!   device controller of this machine, at the end of a cable.
//!
//! All are codecs around one [`crate::bus::Bus`].

#[cfg(feature = "gadget")]
pub mod gadget;
#[cfg(any(feature = "usbip", feature = "usbredir"))]
pub mod socket;
#[cfg(feature = "usbip")]
pub mod usbip;
#[cfg(feature = "usbredir")]
pub mod usbredir;
