// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A serial port that sends back what it gets: the shortest model on the
//! CDC-ACM shell. On Linux it is a `/dev/ttyACMn`.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Port, Serial};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "cdc-echo";
/// One line about the device and its options, for a host that lists its models.
pub const ABOUT: &str = "a CDC-ACM serial port that echoes; the shell's faults: stall=, chunk=N, \
     in-limit=N, in-stall=N, out-hold=N";

/// The echo.
#[derive(Clone, Copy, Debug, Default)]
pub struct Echo;

impl Serial for Echo {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        port.send(data);
    }
}

/// The device, from the shell's fault options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut faults = Faults::default();
    for (k, v) in options {
        if !faults.option(k, v)? {
            return Err(Error::Invalid(format!("{NAME} has no option {k}")));
        }
    }
    // pid.codes' test product id.
    let id = Identity::new(0x1209, 0x0001, "vusb", "CDC echo");
    Ok(Box::new(Cdc::new(NAME, &id, faults, Echo)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{Bus, Status};
    use crate::device::Device;
    use crate::host::enumerate;

    #[test]
    fn echoes() {
        let mut bus = Bus::new(Device::new(build(&[]).unwrap()));
        let e = enumerate(&mut bus).unwrap();
        assert_eq!(e.product, "CDC echo");
        assert_eq!(bus.write(2, b"hello").status, Status::Ok);
        assert_eq!(bus.read(2, 64).data, b"hello");
    }
}
