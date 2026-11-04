// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Building and walking USB descriptors.

/// Descriptor types.
pub const DEVICE: u8 = 1;
/// Configuration.
pub const CONFIGURATION: u8 = 2;
/// String.
pub const STRING: u8 = 3;
/// Interface.
pub const INTERFACE: u8 = 4;
/// Endpoint.
pub const ENDPOINT: u8 = 5;
/// Device qualifier, asked of full speed devices by high speed hosts.
pub const DEVICE_QUALIFIER: u8 = 6;
/// OTG, which `lsusb -v` and OTG hosts ask for.
pub const OTG: u8 = 0x09;
/// Debug, which `lsusb -v` asks for.
pub const DEBUG: u8 = 0x0a;
/// Binary object store, asked when `bcdUSB` is 2.01 or newer.
pub const BOS: u8 = 0x0f;
/// HID class descriptor.
pub const HID: u8 = 0x21;
/// HID report descriptor.
pub const HID_REPORT: u8 = 0x22;

/// The fields of a device descriptor.
#[derive(Clone, Debug)]
pub struct DeviceDescriptor {
    /// `bcdUSB`.
    pub usb: u16,
    /// `bDeviceClass`, `bDeviceSubClass`, `bDeviceProtocol`.
    pub class: [u8; 3],
    /// `bMaxPacketSize0`.
    pub max_packet0: u8,
    /// `idVendor`.
    pub vendor: u16,
    /// `idProduct`.
    pub product: u16,
    /// `bcdDevice`.
    pub device: u16,
    /// `iManufacturer`, `iProduct`, `iSerialNumber`.
    pub strings: [u8; 3],
    /// `bNumConfigurations`.
    pub configurations: u8,
}

impl DeviceDescriptor {
    /// The 18 bytes.
    pub fn bytes(&self) -> Vec<u8> {
        let mut b = vec![18, DEVICE];
        b.extend_from_slice(&self.usb.to_le_bytes());
        b.extend_from_slice(&self.class);
        b.push(self.max_packet0);
        b.extend_from_slice(&self.vendor.to_le_bytes());
        b.extend_from_slice(&self.product.to_le_bytes());
        b.extend_from_slice(&self.device.to_le_bytes());
        b.extend_from_slice(&self.strings);
        b.push(self.configurations);
        b
    }
}

/// The device qualifier of a high speed device, from its 18 byte device
/// descriptor: what the device would be at full speed, which here is
/// the same device (`bcdUSB`, class, `bMaxPacketSize0`,
/// `bNumConfigurations`).
pub fn device_qualifier(device: &[u8]) -> Vec<u8> {
    vec![
        10,
        DEVICE_QUALIFIER,
        device[2],
        device[3],
        device[4],
        device[5],
        device[6],
        device[7],
        device[17],
        0,
    ]
}

/// A configuration descriptor with its interface, class and endpoint
/// descriptors behind it; `wTotalLength` and `bNumInterfaces` are
/// computed. `attributes` is `bmAttributes` (0x80 bus powered, 0xa0
/// with remote wakeup, 0xc0 self powered); `max_power` is in mA.
pub fn configuration(
    value: u8,
    string: u8,
    attributes: u8,
    max_power: u16,
    body: &[u8],
) -> Vec<u8> {
    let interfaces = records(body).filter(|r| r[1] == INTERFACE && r[3] == 0).count() as u8;
    let total = (9 + body.len()) as u16;
    let mut b = vec![9, CONFIGURATION];
    b.extend_from_slice(&total.to_le_bytes());
    b.push(interfaces);
    b.push(value);
    b.push(string);
    b.push(attributes);
    b.push((max_power / 2) as u8);
    b.extend_from_slice(body);
    b
}

/// An interface descriptor.
pub fn interface(number: u8, alternate: u8, endpoints: u8, class: [u8; 3], string: u8) -> [u8; 9] {
    [9, INTERFACE, number, alternate, endpoints, class[0], class[1], class[2], string]
}

/// An endpoint descriptor; `attributes` is the transfer type.
pub fn endpoint(address: u8, attributes: u8, max_packet: u16, interval: u8) -> [u8; 7] {
    let m = max_packet.to_le_bytes();
    [7, ENDPOINT, address, attributes, m[0], m[1], interval]
}

/// A string descriptor, UTF-16LE.
pub fn string(s: &str) -> Vec<u8> {
    let mut b = vec![0, STRING];
    for unit in s.encode_utf16() {
        b.extend_from_slice(&unit.to_le_bytes());
    }
    b[0] = b.len() as u8;
    b
}

/// String descriptor 0: US English only.
pub fn language_table() -> Vec<u8> {
    vec![4, STRING, 0x09, 0x04]
}

/// The descriptor records of a block: each slice starts at its
/// `bLength`. Stops at the first malformed record.
pub fn records(block: &[u8]) -> impl Iterator<Item = &[u8]> {
    let mut at = 0;
    std::iter::from_fn(move || {
        let len = usize::from(*block.get(at)?);
        if len < 2 || at + len > block.len() {
            return None;
        }
        let r = &block[at..at + len];
        at += len;
        Some(r)
    })
}

/// At most `length` bytes of a descriptor, as a host's `wLength` asks.
pub fn truncate(bytes: &[u8], length: u16) -> Vec<u8> {
    bytes[..bytes.len().min(usize::from(length))].to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configuration_counts_interfaces_and_length() {
        let mut body = Vec::new();
        body.extend_from_slice(&interface(0, 0, 1, [3, 0, 0], 0));
        body.extend_from_slice(&endpoint(0x81, 3, 64, 2));
        body.extend_from_slice(&interface(1, 0, 0, [0xff, 0, 0], 0));
        body.extend_from_slice(&interface(1, 1, 0, [0xff, 0, 0], 0));
        let c = configuration(1, 0, 0xa0, 100, &body);
        assert_eq!(c.len(), 9 + 9 + 7 + 9 + 9);
        assert_eq!(u16::from_le_bytes([c[2], c[3]]), c.len() as u16);
        assert_eq!(c[4], 2);
        assert_eq!(c[8], 50);
        assert_eq!(records(&c).count(), 5);
    }

    #[test]
    fn strings_are_utf16() {
        assert_eq!(string("Ab"), vec![6, 3, b'A', 0, b'b', 0]);
        assert_eq!(truncate(&string("Ab"), 2), vec![6, 3]);
    }
}
