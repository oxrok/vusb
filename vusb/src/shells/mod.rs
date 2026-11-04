// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Shells: the parts every device of a kind shares, which a model
//! builds on and fills in with what its own protocol means.
//!
//! - [`hid`]: the HID class descriptor, the report descriptor fetch and
//!   the class requests (`GET_REPORT`, `SET_REPORT`, idle, protocol).
//! - [`cdc`]: a CDC-ACM function: its descriptors and class requests,
//!   with a model that is a byte stream ([`cdc::Serial`]), reached
//!   through `/dev/ttyACMn` on Linux.
//! - [`ftdi`]: the FTDI chips' descriptors, vendor requests (reset,
//!   purge, baud rate, line, flow and modem control, latency timer, bit
//!   modes, pins, EEPROM) and the two status bytes of every IN packet.
//! - [`ezusb`]: the Cypress FX2 without firmware, its boot loader
//!   (request `0xa0`, the 8051 reset in `CPUCS`) and a record of the
//!   upload; the model renumerates as the firmware's device.

pub mod cdc;
pub mod ezusb;
pub mod ftdi;
pub mod hid;
