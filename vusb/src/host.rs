// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! What a host does to a device it finds: enumeration, as a convenience
//! for tests on a [`Bus`].

use crate::bus::{Bus, Status};
use crate::descriptor;
use crate::model::{Descriptors, Endpoint, Setup};
use crate::{Error, Result};

const GET_DESCRIPTOR: u8 = 6;
const SET_CONFIGURATION: u8 = 9;

/// What enumeration found.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Enumerated {
    /// The 18 byte device descriptor.
    pub device: Vec<u8>,
    /// Each configuration descriptor, `wTotalLength` bytes.
    pub configs: Vec<Vec<u8>>,
    /// `iManufacturer`, empty when the device names none or STALLs it.
    pub manufacturer: String,
    /// `iProduct`, likewise.
    pub product: String,
    /// `iSerialNumber`, likewise.
    pub serial: String,
    /// The `bConfigurationValue` set: the first configuration's.
    pub configuration: u8,
}

impl Enumerated {
    /// `idVendor`.
    pub fn vendor_id(&self) -> u16 {
        u16::from_le_bytes([self.device[8], self.device[9]])
    }

    /// `idProduct`.
    pub fn product_id(&self) -> u16 {
        u16::from_le_bytes([self.device[10], self.device[11]])
    }

    /// The endpoints of the configuration set.
    pub fn endpoints(&self) -> Vec<Endpoint> {
        self.configs
            .iter()
            .find(|c| c.get(5) == Some(&self.configuration))
            .map(|c| Descriptors::endpoints(c))
            .unwrap_or_default()
    }
}

fn get_descriptor(kind: u8, index: u8, lang: u16, length: u16) -> Setup {
    Setup {
        request_type: 0x80,
        request: GET_DESCRIPTOR,
        value: u16::from(kind) << 8 | u16::from(index),
        index: lang,
        length,
    }
}

/// Enumerates the device as a host does: the device descriptor, every
/// configuration descriptor in full, the strings the device descriptor
/// names, and `SET_CONFIGURATION` of the first configuration. Lenient
/// as a host is: a string the device STALLs is empty, not an error.
///
/// # Errors
///
/// When the device STALLs its device or a configuration descriptor, or
/// answers one too short to use, or refuses the configuration.
pub fn enumerate(bus: &mut Bus) -> Result<Enumerated> {
    let device = bus.control(get_descriptor(descriptor::DEVICE, 0, 0, 18), &[]);
    if device.status != Status::Ok || device.data.len() < 18 {
        return Err(Error::Protocol(format!(
            "device descriptor: {:?}, {} bytes",
            device.status,
            device.data.len()
        )));
    }
    let device = device.data;
    let mut configs = Vec::new();
    for i in 0..device[17] {
        let head = bus.control(get_descriptor(descriptor::CONFIGURATION, i, 0, 9), &[]);
        if head.status != Status::Ok || head.data.len() < 9 {
            return Err(Error::Protocol(format!(
                "configuration descriptor {i}: {:?}, {} bytes",
                head.status,
                head.data.len()
            )));
        }
        let total = u16::from_le_bytes([head.data[2], head.data[3]]);
        let full = bus.control(get_descriptor(descriptor::CONFIGURATION, i, 0, total), &[]);
        if full.status != Status::Ok || full.data.len() != usize::from(total) {
            return Err(Error::Protocol(format!(
                "configuration descriptor {i}: {:?}, {} of {total} bytes",
                full.status,
                full.data.len()
            )));
        }
        configs.push(full.data);
    }
    let mut strings = [String::new(), String::new(), String::new()];
    if device[14..17].iter().any(|&i| i != 0) {
        let table = bus.control(get_descriptor(descriptor::STRING, 0, 0, 255), &[]);
        let lang = match table.data.get(2..4) {
            Some(l) if table.status == Status::Ok => u16::from_le_bytes([l[0], l[1]]),
            _ => 0x0409,
        };
        for (slot, &index) in strings.iter_mut().zip(&device[14..17]) {
            if index == 0 {
                continue;
            }
            let s = bus.control(get_descriptor(descriptor::STRING, index, lang, 255), &[]);
            if s.status == Status::Ok && s.data.len() >= 2 {
                let units: Vec<u16> =
                    s.data[2..].as_chunks::<2>().0.iter().map(|c| u16::from_le_bytes(*c)).collect();
                *slot = String::from_utf16_lossy(&units);
            }
        }
    }
    let configuration = configs.first().and_then(|c| c.get(5).copied()).unwrap_or(0);
    let set = Setup {
        request_type: 0x00,
        request: SET_CONFIGURATION,
        value: u16::from(configuration),
        index: 0,
        length: 0,
    };
    if !bus.control(set, &[]).is_ok() {
        return Err(Error::Protocol(format!("SET_CONFIGURATION {configuration} refused")));
    }
    let [manufacturer, product, serial] = strings;
    Ok(Enumerated { device, configs, manufacturer, product, serial, configuration })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Device;
    use crate::examples::relay::UsbRelay;
    use crate::model::{DeviceModel, Reply, StandardFault, Violations};

    #[test]
    fn enumerates_a_relay_board() {
        let mut bus = Bus::new(Device::new(Box::new(UsbRelay::new(3))));
        let e = enumerate(&mut bus).unwrap();
        assert_eq!((e.vendor_id(), e.product_id()), (0x16c0, 0x05df));
        assert_eq!(e.manufacturer, "www.dcttech.com");
        assert_eq!(e.product, "USBRelay3");
        assert_eq!(e.serial, "0001");
        assert_eq!(e.configuration, 1);
        assert_eq!(bus.device().configuration(), 1);
        let eps = e.endpoints();
        assert_eq!(eps.len(), 1);
        assert_eq!((eps[0].address, eps[0].transfer_type()), (0x81, crate::model::INTERRUPT));
    }

    /// A board that STALLs its product string, and one that STALLs the
    /// configuration descriptor.
    struct Failing(UsbRelay, u16);

    impl DeviceModel for Failing {
        fn name(&self) -> &'static str {
            self.0.name()
        }
        fn descriptors(&self) -> &Descriptors {
            self.0.descriptors()
        }
        fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
            self.0.control(setup, data, v)
        }
        fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
            self.0.out(ep, data, v)
        }
        fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
            self.0.in_transfer(ep, max, v)
        }
        fn reset(&mut self) {
            self.0.reset();
        }
        fn fail_standard(&mut self, setup: Setup) -> Option<StandardFault> {
            (setup.request == GET_DESCRIPTOR && setup.value == self.1)
                .then_some(StandardFault::Stall)
        }
    }

    #[test]
    fn is_lenient_on_strings_only() {
        let mut bus = Bus::new(Device::new(Box::new(Failing(UsbRelay::new(2), 0x0302))));
        let e = enumerate(&mut bus).unwrap();
        assert_eq!(e.product, "");
        assert_eq!(e.manufacturer, "www.dcttech.com");
        let mut bus = Bus::new(Device::new(Box::new(Failing(UsbRelay::new(2), 0x0200))));
        assert!(enumerate(&mut bus).is_err());
    }
}
