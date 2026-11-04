// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The Cypress EZ-USB FX2 shell: the chip as it comes up without
//! firmware, its boot loader's vendor request `0xa0`, and the checks on
//! a firmware upload; a model of an FX2 based device builds on it and
//! becomes the firmware's device once the upload is done.
//!
//! Without an EEPROM (or with a `0xC0` EEPROM holding only the ids) the
//! FX2 enumerates with its built in descriptors: vendor class, a 64
//! byte endpoint 0, no strings, one configuration whose interface 0 has
//! the alternate settings of the TRM's default table ([`bare_descriptors`]).
//! Its 8051 is held in reset, and the boot loader in silicon answers
//! vendor request `0xa0` on endpoint 0 whatever runs: `wValue` is an
//! address, an OUT writes the data there, an IN reads it back. Code and
//! data RAM are `0x0000` up to 16 KiB on the FX2LP (8 KiB on the older
//! FX2), and `0xe000` to `0xe1ff`; one register is reachable, `CPUCS`
//! at `0xe600`, whose bit 0 holds the 8051 in reset. Releasing it starts
//! the uploaded code, which for most firmware disconnects from
//! the bus and comes back as the firmware's device: a renumeration
//! (`DeviceModel::take_renumeration`).
//!
//! [`Loader`] is that boot loader; it records every write and `CPUCS`
//! value ([`Loader::ops`]), flags writes outside the chip's memory,
//! code written while the 8051 runs and `CPUCS` values other than 0
//! and 1, and reports the release. A model that knows how its driver
//! uploads (the chunk size, the order) holds the record against it
//! once the 8051 is released, and [`Loader::image`] against the
//! firmware file.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, Reply, Setup, Speed, VENDOR, Violations};

/// The boot loader's request: firmware load.
pub const REQ_FIRMWARE_LOAD: u8 = 0xa0;
/// The `CPUCS` register; bit 0 holds the 8051 in reset.
pub const CPUCS: u16 = 0xe600;
/// The Cypress vendor id of a bare FX2.
pub const CYPRESS_VID: u16 = 0x04b4;
/// The product id of a bare FX2.
pub const FX2_PID: u16 = 0x8613;
/// `bcdDevice` of a bare FX2LP (CY7C68013A).
pub const FX2LP_DID: u16 = 0xa001;

/// The scratch RAM, `0xe000` to `0xe1ff`.
const SCRATCH: std::ops::Range<u32> = 0xe000..0xe200;

/// Which chip: how much code RAM it has.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chip {
    /// CY7C68013, 8 KiB.
    Fx2,
    /// CY7C68013A, 16 KiB.
    Fx2lp,
}

impl Chip {
    /// The end of code RAM.
    pub fn code_end(self) -> u32 {
        match self {
            Chip::Fx2 => 0x2000,
            Chip::Fx2lp => 0x4000,
        }
    }
}

/// The descriptors of an FX2 without firmware, under `vendor:product`
/// (the chip's own, or an EEPROM's) and `device` as `bcdDevice`: high
/// speed, vendor class, no strings, interface 0 with alternate setting
/// 0 (no bandwidth) and 1 (the bulk endpoints of the TRM's default
/// table: 1 OUT and IN of 64 bytes, 2 and 4 OUT, 6 and 8 IN of 512).
/// The interrupt and isochronous alternate settings 2 and 3 are left
/// out; no host driver of a bare FX2 selects them.
pub fn bare_descriptors(vendor: u16, product: u16, device: u16) -> Descriptors {
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 0, [0xff, 0xff, 0xff], 0));
    body.extend_from_slice(&descriptor::interface(0, 1, 6, [0xff, 0xff, 0xff], 0));
    for (address, size) in
        [(0x01, 64), (0x81, 64), (0x02, 512), (0x04, 512), (0x86, 512), (0x88, 512)]
    {
        body.extend_from_slice(&descriptor::endpoint(address, crate::model::BULK, size, 0));
    }
    let config = descriptor::configuration(1, 0, 0x80, 100, &body);
    let dev = DeviceDescriptor {
        usb: 0x0200,
        class: [0xff, 0xff, 0xff],
        max_packet0: 64,
        vendor,
        product,
        device,
        strings: [0, 0, 0],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::High,
        device: dev.bytes(),
        configs: vec![config],
        strings: Vec::new(),
    }
}

/// One thing the host did through the boot loader.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Op {
    /// A value written to `CPUCS`.
    Cpucs(u8),
    /// A write of `len` bytes at `address`.
    Write {
        /// `wValue`.
        address: u16,
        /// The data length.
        len: usize,
    },
}

/// The FX2 boot loader: request `0xa0` and the 8051 reset.
#[derive(Clone, Debug)]
pub struct Loader {
    chip: Chip,
    ram: Vec<u8>,
    /// The highest address written, plus one.
    top: usize,
    held: bool,
    ops: Vec<Op>,
}

impl Loader {
    /// A chip just powered: RAM zero, the 8051 held in reset.
    pub fn new(chip: Chip) -> Loader {
        Loader { chip, ram: vec![0; 0x1_0000], top: 0, held: true, ops: Vec::new() }
    }

    /// Whether the 8051 is held in reset.
    pub fn held(&self) -> bool {
        self.held
    }

    /// What the host did so far.
    pub fn ops(&self) -> &[Op] {
        &self.ops
    }

    /// The RAM from address 0 up to the highest byte written.
    pub fn image(&self) -> &[u8] {
        &self.ram[..self.top]
    }

    /// Answers request `0xa0`, or `None` when `setup` is not one. The
    /// second value is true when this write released the 8051 from
    /// reset, so the uploaded code starts.
    pub fn control(
        &mut self,
        setup: Setup,
        data: &[u8],
        v: &mut Violations,
    ) -> Option<(Reply, bool)> {
        if setup.kind() != VENDOR || setup.request != REQ_FIRMWARE_LOAD {
            return None;
        }
        if setup.recipient() != crate::model::RECIPIENT_DEVICE {
            v.report(format!(
                "firmware load with bmRequestType {:#04x}: the loader is the device's",
                setup.request_type
            ));
        }
        if setup.index != 0 {
            v.report(format!(
                "firmware load with wIndex {:#06x}; the loader uses none",
                setup.index
            ));
        }
        let start = u32::from(setup.value);
        let len = usize::from(setup.length);
        let end = start + len as u32;
        let cpucs = start == u32::from(CPUCS) && len == 1;
        let in_ram = (end <= self.chip.code_end()
            || (SCRATCH.contains(&start) && end <= SCRATCH.end))
            && len > 0;
        if !cpucs && !in_ram {
            v.report(format!(
                "firmware load of {len} bytes at {start:#06x}: outside the RAM (0x0000-{:#06x}, \
                 0xe000-0xe1ff) and not CPUCS",
                self.chip.code_end() - 1
            ));
        }
        if setup.is_in() {
            let bytes = (start..end).map(|a| self.ram[(a & 0xffff) as usize]).collect();
            return Some((Reply::Data(bytes), false));
        }
        if cpucs {
            let value = data.first().copied().unwrap_or(0);
            self.ops.push(Op::Cpucs(value));
            if value & !1 != 0 {
                v.report(format!("CPUCS {value:#04x}: only bit 0 (8051RES) is the host's"));
            }
            let release = self.held && value & 1 == 0;
            self.held = value & 1 != 0;
            if release && self.top == 0 {
                v.report("CPUCS released with no code loaded");
            }
            return Some((Reply::ack(), release));
        }
        self.ops.push(Op::Write { address: setup.value, len: data.len() });
        if !self.held && start < self.chip.code_end() {
            v.report(format!("code written at {start:#06x} while the 8051 runs"));
        }
        for (i, b) in data.iter().enumerate() {
            let a = start as usize + i;
            if a < self.ram.len() {
                self.ram[a] = *b;
                if a < self.chip.code_end() as usize {
                    self.top = self.top.max(a + 1);
                }
            }
        }
        Some((Reply::ack(), false))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn load(value: u16, length: u16) -> Setup {
        Setup { request_type: 0x40, request: REQ_FIRMWARE_LOAD, value, index: 0, length }
    }

    /// An image with no two blocks alike.
    fn image(size: usize) -> Vec<u8> {
        (0..size).map(|i| (i * 7 % 251) as u8).collect()
    }

    #[test]
    fn an_upload_is_recorded_and_releases_the_8051() {
        let image = image(4096 + 100);
        let mut l = Loader::new(Chip::Fx2lp);
        let mut v = Violations::default();
        assert_eq!(l.control(load(CPUCS, 1), &[1], &mut v), Some((Reply::ack(), false)));
        l.control(load(0, 4096), &image[..4096], &mut v);
        l.control(load(0x1000, 100), &image[4096..], &mut v);
        assert!(l.held());
        assert_eq!(l.control(load(CPUCS, 1), &[0], &mut v), Some((Reply::ack(), true)));
        assert!(!l.held());
        assert_eq!(
            l.ops(),
            [
                Op::Cpucs(1),
                Op::Write { address: 0, len: 4096 },
                Op::Write { address: 0x1000, len: 100 },
                Op::Cpucs(0),
            ]
        );
        assert_eq!(l.image(), &image[..]);
        assert!(v.is_empty(), "{v:?}");
        // Read back through the loader.
        let read = Setup { request_type: 0xc0, ..load(0x1000, 4) };
        assert_eq!(
            l.control(read, &[], &mut v),
            Some((Reply::Data(image[4096..4100].to_vec()), false))
        );
        // Not the loader's request.
        let other = Setup { request: 0xb0, ..read };
        assert_eq!(l.control(other, &[], &mut v), None);
    }

    #[test]
    fn flags_what_the_loader_forbids() {
        let mut v = Violations::default();
        let mut l = Loader::new(Chip::Fx2lp);
        l.control(load(0x4000, 16), &[0; 16], &mut v);
        assert_eq!(v.drain().len(), 1, "beyond the code RAM");
        l.control(load(0xe1f0, 32), &[0; 32], &mut v);
        assert_eq!(v.drain().len(), 1, "across the end of the scratch RAM");
        l.control(load(CPUCS, 1), &[0x09], &mut v);
        assert_eq!(v.drain().len(), 1, "a CPUCS bit not the host's");
        assert!(l.held());
        l.control(load(CPUCS, 1), &[0], &mut v);
        assert_eq!(v.drain().len(), 1, "released with nothing loaded");
        l.control(load(0, 16), &[0; 16], &mut v);
        assert_eq!(v.drain().len(), 1, "code written while the 8051 runs");
        let mut l = Loader::new(Chip::Fx2lp);
        l.control(load(CPUCS, 1), &[1], &mut v);
        assert_eq!(l.control(load(CPUCS, 1), &[0], &mut v), Some((Reply::ack(), true)));
        assert_eq!(v.drain().len(), 1, "released with nothing loaded");
        let mut l = Loader::new(Chip::Fx2);
        l.control(Setup { index: 1, ..load(0x2000, 1) }, &[0], &mut v);
        assert_eq!(v.drain().len(), 2, "wIndex, and beyond an FX2's 8 KiB");
    }
}
