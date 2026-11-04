// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! An FTDI chip with TX wired to RX on each port: the shortest model on
//! the FTDI shell. On Linux `ftdi_sio` makes it a `/dev/ttyUSBn`, and
//! libftdi talks to it directly.

use std::collections::VecDeque;

use crate::model::{Descriptors, DeviceModel, Reply, Setup, Violations};
use crate::shells::ftdi::{Chip, Faults, Fill, Ftdi, FtdiConfig, FtdiPorts, PortState};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "ftdi-loopback";
/// One line about the device and its options, for a host that lists its models.
pub const ABOUT: &str = "an FTDI chip with its ports looped back; chip=ft232r|ft2232h, the shell's \
     faults: stall=REQUEST[/WVALUE], in-stall=N, idle=N";

/// What is written waits to be read, per port.
#[derive(Debug, Default)]
struct Loops(Vec<VecDeque<u8>>);

impl FtdiPorts for Loops {
    fn write(&mut self, port: usize, data: &[u8], _: &PortState, _: &mut Violations) -> Reply {
        self.0[port].extend(data);
        Reply::ack()
    }

    fn read(&mut self, port: usize, max: usize, _: &PortState, _: &mut Violations) -> Fill {
        let queue = &mut self.0[port];
        if queue.is_empty() {
            return Fill::Wait;
        }
        let n = max.min(queue.len());
        Fill::Data(queue.drain(..n).collect())
    }

    fn pins(&mut self, _: usize, _: &PortState, _: &mut Violations) -> u8 {
        0xff
    }
}

/// The device.
#[derive(Debug)]
pub struct FtdiLoopback {
    ftdi: Ftdi,
    loops: Loops,
}

impl DeviceModel for FtdiLoopback {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        self.ftdi.descriptors()
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        self.ftdi.control(&mut self.loops, setup, data, v)
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        self.ftdi.out(&mut self.loops, ep, data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        self.ftdi.in_transfer(&mut self.loops, ep, max, v)
    }

    fn reset(&mut self) {
        self.ftdi.reset();
        self.loops.0.iter_mut().for_each(VecDeque::clear);
    }
}

/// The device, from `chip=` and the shell's fault options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut chip = Chip::R;
    let mut faults = Faults::default();
    for (k, v) in options {
        if k == "chip" {
            chip = match v.as_str() {
                "ft232r" => Chip::R,
                "ft2232h" => Chip::Ft2232h,
                other => return Err(Error::Invalid(format!("chip={other}: ft232r or ft2232h"))),
            };
        } else if !faults.option(k, v)? {
            return Err(Error::Invalid(format!("{NAME} has no option {k}")));
        }
    }
    let mut ftdi = Ftdi::new(FtdiConfig::new(chip));
    ftdi.set_faults(faults);
    let loops = Loops(vec![VecDeque::new(); usize::from(chip.ports())]);
    Ok(Box::new(FtdiLoopback { ftdi, loops }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::Bus;
    use crate::device::Device;
    use crate::host::enumerate;

    #[test]
    fn what_is_written_comes_back_behind_the_status_bytes() {
        let options = [("chip".to_owned(), "ft2232h".to_owned())];
        let mut bus = Bus::new(Device::new(build(&options).unwrap()));
        let e = enumerate(&mut bus).unwrap();
        assert_eq!(e.vendor_id(), 0x0403);
        assert!(bus.write(4, b"on B").is_ok());
        assert!(bus.write(2, b"on A").is_ok());
        assert_eq!(&bus.read(1, 512).data[2..], b"on A");
        assert_eq!(&bus.read(3, 512).data[2..], b"on B");
    }
}
