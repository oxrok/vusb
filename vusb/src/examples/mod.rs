// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Example devices: small models that show the contract, one per shell,
//! and that the crate's own tests and `vusb run` use.
//!
//! - [`relay`]: a USB relay board, one HID feature report.
//! - [`cdc_echo`]: a CDC-ACM serial port that sends back what it gets.
//! - [`ftdi_loopback`]: an FTDI chip with its pins looped back.

pub mod cdc_echo;
pub mod ftdi_loopback;
pub mod relay;

use crate::model::DeviceModel;
use crate::{Error, Result};

/// Name and description of each, with its options.
pub const NAMES: &[(&str, &str)] = &[
    (cdc_echo::NAME, cdc_echo::ABOUT),
    (ftdi_loopback::NAME, ftdi_loopback::ABOUT),
    (relay::NAME, relay::ABOUT),
];

/// Builds an example from its name and `key=value` options.
///
/// # Errors
///
/// When no example has that name, or it refuses an option.
pub fn build(name: &str, options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    match name {
        cdc_echo::NAME => cdc_echo::build(options),
        ftdi_loopback::NAME => ftdi_loopback::build(options),
        relay::NAME => relay::build(options),
        other => Err(Error::Invalid(format!("no example named {other}"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_example_builds_under_its_name() {
        for (name, about) in NAMES {
            assert_eq!(build(name, &[]).unwrap().name(), *name);
            assert_ne!(*about, "");
            let bad = [("no-such-option".to_owned(), "1".to_owned())];
            assert!(build(name, &bad).is_err(), "{name} refuses what it does not know");
        }
        assert!(build("nope", &[]).is_err());
    }
}
