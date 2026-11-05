// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A disk: the mass storage shell around a disk image, or around
//! memory. On Linux it is a `/dev/sdX`, to partition, format and mount
//! as any other.
//!
//! With `file=PATH` the disk is that file, made beforehand: its size is
//! the disk's, and what the host writes is in it.
//!
//! ```text
//! truncate -s 64M disk.img
//! sudo vusb run msc-disk -o file=disk.img --attach
//! ```
//!
//! Without it the disk is `size=MIB` of memory (8 by default), all
//! zeros at every start and after every `reset`, which is what a test
//! of a host's driver wants.
//!
//! The file is opened when the model is built, by whoever runs the
//! session, and a session builds a model from the options its control
//! socket hands it too. A program that takes `file=` from there lets
//! whoever reaches the socket name any file the program can open;
//! `vusb run` takes it from its own command line only.

use std::path::PathBuf;

use crate::model::DeviceModel;
use crate::options::parse_number;
use crate::shells::msc::{BLOCK, Faults, Identity, Image, Medium, Memory, Msc};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "msc-disk";
/// One line about the device and its options, for a host that lists its models.
pub const ABOUT: &str = "a mass storage disk: file=PATH a disk image made beforehand, or size=MIB \
     of memory (8); read-only=1, removable=1; the shell's faults: read-error=LBA, \
     write-error=LBA, not-ready=N";

/// The size of the disk in memory when `size=` does not say, in MiB.
const DEFAULT_SIZE: u32 = 8;

fn flag(key: &str, value: &str) -> Result<bool> {
    Ok(parse_number(key, value, 0..=1)? == 1)
}

/// The device, from `file=`, `size=`, `read-only=`, `removable=` and
/// the shell's fault options.
///
/// # Errors
///
/// On an unknown option, a value out of range, `file=` with `size=`, or
/// a file that cannot be opened or holds no block.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut faults = Faults::default();
    let mut file: Option<PathBuf> = None;
    let mut size = None;
    let mut read_only = false;
    // pid.codes' test product ids.
    let mut id = Identity::new(0x1209, 0x0005, "vusb", "Disk");
    for (k, v) in options {
        match k.as_str() {
            "file" => file = Some(PathBuf::from(v)),
            "size" => size = Some(parse_number(k, v, 1..=1024)?),
            "read-only" => read_only = flag(k, v)?,
            "removable" => id.removable = flag(k, v)?,
            _ if faults.option(k, v)? => {}
            _ => return Err(Error::Invalid(format!("{NAME} has no option {k}"))),
        }
    }
    let medium: Box<dyn Medium> = match (file, size) {
        (Some(_), Some(_)) => {
            return Err(Error::Invalid("size= is of a disk in memory: a file has its own".into()));
        }
        (Some(path), None) if read_only => Box::new(Image::open_read_only(path)?),
        (Some(path), None) => Box::new(Image::open(path)?),
        (None, size) => {
            let blocks = size.unwrap_or(DEFAULT_SIZE) as usize * (1 << 20) / BLOCK as usize;
            let memory = Memory::new(blocks);
            Box::new(if read_only { memory.read_only() } else { memory })
        }
    };
    Ok(Box::new(Msc::new(NAME, &id, faults, medium)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{Bus, Status};
    use crate::device::Device;
    use crate::host::enumerate;
    use crate::shells::msc::{EP_IN, EP_OUT};

    fn options(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
        pairs.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    /// `READ CAPACITY (10)`: the last block and the block size.
    fn capacity(bus: &mut Bus) -> (u32, u32) {
        let mut cbw = b"USBC".to_vec();
        cbw.extend_from_slice(&1u32.to_le_bytes());
        cbw.extend_from_slice(&8u32.to_le_bytes());
        cbw.extend_from_slice(&[0x80, 0, 10, 0x25]);
        cbw.resize(31, 0);
        assert_eq!(bus.write(EP_OUT, &cbw).status, Status::Ok);
        let data = bus.read(EP_IN, 8).data;
        assert_eq!(bus.read(EP_IN, 13).data[12], 0);
        (
            u32::from_be_bytes([data[0], data[1], data[2], data[3]]),
            u32::from_be_bytes([data[4], data[5], data[6], data[7]]),
        )
    }

    #[test]
    fn a_disk_in_memory_has_the_size_asked_for() {
        let mut bus = Bus::new(Device::new(build(&[]).unwrap()));
        let e = enumerate(&mut bus).unwrap();
        assert_eq!((e.vendor_id(), e.product_id()), (0x1209, 0x0005));
        assert_eq!(e.product, "Disk");
        assert_eq!(capacity(&mut bus), (8 * 2048 - 1, 512));

        let mut bus = Bus::new(Device::new(build(&options(&[("size", "1")])).unwrap()));
        enumerate(&mut bus).unwrap();
        assert_eq!(capacity(&mut bus), (2047, 512));
        assert!(build(&options(&[("size", "0")])).is_err());
        assert!(build(&options(&[("read-only", "2")])).is_err());
        assert!(build(&options(&[("read-error", "7"), ("removable", "1")])).is_ok());
    }

    #[test]
    fn a_file_is_the_disk() {
        let path = std::env::temp_dir().join(format!("vusb-msc-disk-{}.img", std::process::id()));
        std::fs::write(&path, vec![0; 100 * 512]).unwrap();
        let file = path.to_str().unwrap();
        let mut bus = Bus::new(Device::new(build(&options(&[("file", file)])).unwrap()));
        enumerate(&mut bus).unwrap();
        assert_eq!(capacity(&mut bus), (99, 512));
        assert!(build(&options(&[("file", file), ("read-only", "1")])).is_ok());
        assert!(build(&options(&[("file", file), ("size", "1")])).is_err());
        std::fs::remove_file(&path).unwrap();
        let gone = build(&options(&[("file", file)])).err().unwrap().to_string();
        assert!(gone.contains(file), "{gone}");
    }
}
