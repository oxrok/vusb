// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The device model contract.

use std::time::Duration;

/// A setup packet, the eight bytes of a control transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Setup {
    /// `bmRequestType`.
    pub request_type: u8,
    /// `bRequest`.
    pub request: u8,
    /// `wValue`.
    pub value: u16,
    /// `wIndex`.
    pub index: u16,
    /// `wLength`.
    pub length: u16,
}

/// `bmRequestType` kinds, as [`Setup::kind`] returns them.
pub const STANDARD: u8 = 0;
/// Class request.
pub const CLASS: u8 = 1;
/// Vendor request.
pub const VENDOR: u8 = 2;

/// Recipients, as [`Setup::recipient`] returns them.
pub const RECIPIENT_DEVICE: u8 = 0;
/// The interface in the low byte of `wIndex`.
pub const RECIPIENT_INTERFACE: u8 = 1;
/// The endpoint in the low byte of `wIndex`.
pub const RECIPIENT_ENDPOINT: u8 = 2;

impl Setup {
    /// Parses the eight bytes.
    pub fn from_bytes(b: &[u8; 8]) -> Setup {
        Setup {
            request_type: b[0],
            request: b[1],
            value: u16::from_le_bytes([b[2], b[3]]),
            index: u16::from_le_bytes([b[4], b[5]]),
            length: u16::from_le_bytes([b[6], b[7]]),
        }
    }

    /// The eight bytes.
    pub fn to_bytes(self) -> [u8; 8] {
        let v = self.value.to_le_bytes();
        let i = self.index.to_le_bytes();
        let l = self.length.to_le_bytes();
        [self.request_type, self.request, v[0], v[1], i[0], i[1], l[0], l[1]]
    }

    /// Device to host.
    pub fn is_in(self) -> bool {
        self.request_type & 0x80 != 0
    }

    /// [`STANDARD`], [`CLASS`], [`VENDOR`] or 3 (reserved).
    pub fn kind(self) -> u8 {
        (self.request_type >> 5) & 3
    }

    /// [`RECIPIENT_DEVICE`], [`RECIPIENT_INTERFACE`], [`RECIPIENT_ENDPOINT`],
    /// 3 (other) or a reserved value.
    pub fn recipient(self) -> u8 {
        self.request_type & 0x1f
    }
}

/// What the device answers a transfer with.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Reply {
    /// The data of an IN transfer, or acceptance of an OUT transfer
    /// (empty, or the bytes taken when fewer than offered).
    Data(Vec<u8>),
    /// Not ready: the transfer stays pending and is offered again.
    Nak,
    /// Protocol STALL.
    Stall,
}

impl Reply {
    /// Acceptance of an OUT transfer.
    pub fn ack() -> Reply {
        Reply::Data(Vec::new())
    }
}

/// Bus speed. A transport maps it to its own codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Speed {
    /// 1.5 Mbit/s.
    Low,
    /// 12 Mbit/s.
    Full,
    /// 480 Mbit/s.
    High,
}

/// An endpoint of the active configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Endpoint {
    /// `bEndpointAddress`, direction bit included.
    pub address: u8,
    /// `bmAttributes`.
    pub attributes: u8,
    /// `wMaxPacketSize`.
    pub max_packet: u16,
    /// `bInterval`.
    pub interval: u8,
}

/// Transfer types, from the low bits of `bmAttributes`.
pub const CONTROL: u8 = 0;
/// Isochronous.
pub const ISOCHRONOUS: u8 = 1;
/// Bulk.
pub const BULK: u8 = 2;
/// Interrupt.
pub const INTERRUPT: u8 = 3;

impl Endpoint {
    /// The number without the direction bit.
    pub fn number(self) -> u8 {
        self.address & 0x0f
    }

    /// Device to host.
    pub fn is_in(self) -> bool {
        self.address & 0x80 != 0
    }

    /// [`CONTROL`], [`ISOCHRONOUS`], [`BULK`] or [`INTERRUPT`].
    pub fn transfer_type(self) -> u8 {
        self.attributes & 3
    }

    /// The service interval of an isochronous endpoint on a device of
    /// this speed, from `bInterval`: 2^(n-1) frames of 1 ms at full
    /// speed, microframes of 125 µs at high speed.
    pub fn iso_interval(self, speed: Speed) -> Duration {
        let unit: u64 = if speed == Speed::High { 125 } else { 1000 };
        Duration::from_micros(unit << u32::from(self.interval.clamp(1, 16) - 1))
    }
}

/// The descriptors of a device, as bytes, plus its speed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Descriptors {
    /// Bus speed.
    pub speed: Speed,
    /// The 18 byte device descriptor.
    pub device: Vec<u8>,
    /// Each configuration descriptor with its interfaces and endpoints,
    /// `wTotalLength` bytes.
    pub configs: Vec<Vec<u8>>,
    /// String descriptors 1 and up; 0 is the language table, always
    /// US English.
    pub strings: Vec<String>,
}

impl Descriptors {
    fn device_u16(&self, at: usize) -> u16 {
        u16::from_le_bytes([self.device[at], self.device[at + 1]])
    }

    /// `idVendor`.
    pub fn vendor_id(&self) -> u16 {
        self.device_u16(8)
    }

    /// `idProduct`.
    pub fn product_id(&self) -> u16 {
        self.device_u16(10)
    }

    /// `bcdDevice`.
    pub fn bcd_device(&self) -> u16 {
        self.device_u16(12)
    }

    /// `bDeviceClass`, `bDeviceSubClass`, `bDeviceProtocol`.
    pub fn device_class(&self) -> [u8; 3] {
        [self.device[4], self.device[5], self.device[6]]
    }

    /// The configuration with this `bConfigurationValue`.
    pub fn config_by_value(&self, value: u8) -> Option<&[u8]> {
        self.configs.iter().find(|c| c.get(5) == Some(&value)).map(Vec::as_slice)
    }

    /// The interfaces of a configuration: number, alternate setting,
    /// class, subclass, protocol.
    pub fn interfaces(config: &[u8]) -> Vec<[u8; 5]> {
        crate::descriptor::records(config)
            .filter(|r| r.len() >= 9 && r[1] == crate::descriptor::INTERFACE)
            .map(|r| [r[2], r[3], r[5], r[6], r[7]])
            .collect()
    }

    /// The endpoints of a configuration.
    pub fn endpoints(config: &[u8]) -> Vec<Endpoint> {
        crate::descriptor::records(config)
            .filter(|r| r.len() >= 7 && r[1] == crate::descriptor::ENDPOINT)
            .map(|r| Endpoint {
                address: r[2],
                attributes: r[3],
                max_packet: u16::from_le_bytes([r[4], r[5]]),
                interval: r[6],
            })
            .collect()
    }
}

/// Collects what the host did that the protocol does not allow or the
/// model does not expect. Drained into the transaction log after every
/// transfer.
#[derive(Debug, Default)]
pub struct Violations(Vec<String>);

impl Violations {
    /// Records one.
    pub fn report(&mut self, message: impl Into<String>) {
        self.0.push(message.into());
    }

    /// Takes them out.
    pub fn drain(&mut self) -> Vec<String> {
        std::mem::take(&mut self.0)
    }

    /// None recorded.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// A device, from the host's point of view.
///
/// The standard requests addressed to the device (descriptors,
/// address, configuration, status, features) are answered by
/// [`crate::device::Device`] before a model sees them; a model answers
/// the class and vendor requests and the standard requests addressed
/// to its interfaces, such as a HID report descriptor fetch.
///
/// # Examples
///
/// A vendor device with one bulk IN endpoint that counts: each read
/// gets the next number, and a vendor request resets the count. It
/// fits no shell, so it implements the contract itself.
///
/// ```
/// use vusb::descriptor::{self, DeviceDescriptor};
/// use vusb::model::{BULK, Descriptors, DeviceModel, Reply, Setup, Speed, VENDOR, Violations};
///
/// struct Counter {
///     descriptors: Descriptors,
///     next: u8,
/// }
///
/// impl Counter {
///     fn new() -> Counter {
///         let mut body = Vec::new();
///         body.extend_from_slice(&descriptor::interface(0, 0, 1, [0xff, 0, 0], 0));
///         body.extend_from_slice(&descriptor::endpoint(0x81, BULK, 64, 0));
///         let device = DeviceDescriptor {
///             usb: 0x0200,
///             class: [0, 0, 0],
///             max_packet0: 64,
///             vendor: 0x1209,
///             product: 0x0002,
///             device: 0x0100,
///             strings: [0, 1, 0],
///             configurations: 1,
///         };
///         Counter {
///             descriptors: Descriptors {
///                 speed: Speed::Full,
///                 device: device.bytes(),
///                 configs: vec![descriptor::configuration(1, 0, 0x80, 100, &body)],
///                 strings: vec!["Counter".to_owned()],
///             },
///             next: 0,
///         }
///     }
/// }
///
/// impl DeviceModel for Counter {
///     fn name(&self) -> &'static str {
///         "counter"
///     }
///
///     fn descriptors(&self) -> &Descriptors {
///         &self.descriptors
///     }
///
///     fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
///         if setup.kind() == VENDOR && setup.request == 0x01 {
///             self.next = 0;
///             return Reply::ack();
///         }
///         v.report(format!("request {:#04x} is not one the counter has", setup.request));
///         Reply::Stall
///     }
///
///     fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
///         v.report(format!("OUT to endpoint {ep}; the counter only sends"));
///         Reply::Stall
///     }
///
///     fn in_transfer(&mut self, _ep: u8, _max: usize, _v: &mut Violations) -> Reply {
///         self.next = self.next.wrapping_add(1);
///         Reply::Data(vec![self.next])
///     }
///
///     fn reset(&mut self) {
///         self.next = 0;
///     }
/// }
///
/// // Driven in process, as a test would.
/// use vusb::bus::Bus;
/// use vusb::device::Device;
/// use vusb::host::enumerate;
///
/// # fn main() -> vusb::Result<()> {
/// let mut bus = Bus::new(Device::new(Box::new(Counter::new())));
/// assert_eq!(enumerate(&mut bus)?.product, "Counter");
/// assert_eq!(bus.read(1, 64).data, [1]);
/// assert_eq!(bus.read(1, 64).data, [2]);
/// let reset = Setup { request_type: 0x40, request: 0x01, value: 0, index: 0, length: 0 };
/// assert!(bus.control(reset, &[]).is_ok());
/// assert_eq!(bus.read(1, 64).data, [1]);
/// # Ok(())
/// # }
/// ```
pub trait DeviceModel: Send {
    /// Short name, as a host that serves several models tells them
    /// apart; it ends up in the USB/IP device path.
    fn name(&self) -> &'static str;

    /// The descriptors.
    fn descriptors(&self) -> &Descriptors;

    /// A control transfer: `data` is the OUT payload (empty for IN).
    /// An IN reply longer than `setup.length` is truncated by the
    /// caller.
    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply;

    /// An OUT transfer to endpoint number `ep`.
    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply;

    /// An IN transfer from endpoint number `ep`, up to `max` bytes. For
    /// a bulk endpoint `max` is the URB's length rounded up to whole
    /// packets, which is what the device sees on the bus; an answer
    /// longer than the URB itself is a babble on the host (`-EOVERFLOW`).
    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply;

    /// A bus reset, or a new attach (also the one after a
    /// renumeration, so a model keeps what it was asked to become).
    fn reset(&mut self);

    /// A new IN transfer was submitted on endpoint number `ep`, asking
    /// for up to `max` bytes, the URB's own length (not rounded as for
    /// [`DeviceModel::in_transfer`]); called once per transfer, before
    /// [`DeviceModel::in_transfer`] first offers it. A streaming model
    /// can pace itself by the host's submissions instead of the
    /// clock, so that what completes does not depend on timing.
    fn in_submitted(&mut self, ep: u8, max: usize) {
        let _ = (ep, max);
    }

    /// The host cancelled a held IN transfer on endpoint number `ep`, of
    /// up to `max` bytes (rounded as for [`DeviceModel::in_transfer`]).
    /// What it holds already, if anything: the packets of a transfer the
    /// device has not ended with a short packet. It completes with them
    /// as a real host controller does, -ECONNRESET and the bytes, which
    /// libusb reports as a timed out transfer with data. By default a
    /// held transfer holds nothing.
    fn in_unlinked(&mut self, ep: u8, max: usize) -> Option<Vec<u8>> {
        let _ = (ep, max);
        None
    }

    /// One isochronous IN packet from endpoint number `ep`: what the
    /// device sends in this service interval, up to `max` bytes, or
    /// `None` when it sends nothing, which the host records as a missed
    /// packet. Isochronous transfers have no handshake: no NAK, no
    /// STALL and no retry, so the answer is the data or nothing. One
    /// call is one interval of the endpoint, which is how a streaming
    /// model keeps time: so many samples per call, never the clock. By
    /// default the model has no such endpoint, and says so.
    fn iso_in(&mut self, ep: u8, max: usize, v: &mut Violations) -> Option<Vec<u8>> {
        let _ = max;
        v.report(format!("isochronous IN on endpoint {ep}, which the model does not serve"));
        None
    }

    /// One isochronous OUT packet to endpoint number `ep`: what the
    /// host sent in this service interval. Nothing is answered. By
    /// default the model has no such endpoint, and says so.
    fn iso_out(&mut self, ep: u8, data: &[u8], v: &mut Violations) {
        let _ = data;
        v.report(format!("isochronous OUT to endpoint {ep}, which the model does not serve"));
    }

    /// Whether the device's isochronous streams keep real time. The
    /// model still reads no clock and never waits, and one call of
    /// [`DeviceModel::iso_in`] or [`DeviceModel::iso_out`] is still one
    /// service interval; it is the host that waits. A
    /// [`Bus`](crate::bus::Bus) whose device says `true` completes an
    /// isochronous transfer when the service interval of its last
    /// packet is over, as a host controller does, so a stream moves at
    /// the rate its descriptors say: a microphone gives a second of
    /// sound in a second. By default `false`: a transfer completes when
    /// it is submitted, as fast as the host asks, which is what a test
    /// wants. Asked at every isochronous transfer, so the answer may
    /// change while the device runs. What moves is the same either way.
    fn realtime(&self) -> bool {
        false
    }

    /// Whether the model wants to leave the bus and come back with
    /// what [`DeviceModel::descriptors`] says now, as an EZ-USB device
    /// does once its new firmware runs. Asked after every transfer; a
    /// model answers `true` once per renumeration. The host then takes
    /// the device off the bus and puts it back with the new
    /// descriptors (through `vhci-hcd`, a detach and a new import).
    fn take_renumeration(&mut self) -> bool {
        false
    }

    /// Whether the model has reached the point in its protocol that it
    /// marks for an event armed with `model` ([`crate::arm::When::Model`]),
    /// such as a capture waiting for a trigger that never comes. Asked
    /// after every offer of a transfer, one answered NAK too; a model
    /// answers `true` once per point. What the point is, and the options
    /// that make a model mark it, are the model's to document.
    fn mark(&mut self) -> bool {
        false
    }

    /// A standard request (`bmRequestType` type 0), before the device
    /// ([`crate::device::Device`]) or the model answers it: a fault
    /// option's way to make one fail as firmware can, such as a string
    /// descriptor read. [`StandardFault::Stall`] STALLs it and the
    /// request has no effect; [`StandardFault::Short`] cuts the answer
    /// of an IN request to that many bytes. `None`, the default, leaves
    /// it alone. The host's enumeration goes through here too, so a
    /// fault that should spare it is one whose options do not change
    /// the descriptors (no re-attach), or one the model counts.
    fn fail_standard(&mut self, setup: Setup) -> Option<StandardFault> {
        let _ = setup;
        None
    }
}

/// How a model fails a standard request, see
/// [`DeviceModel::fail_standard`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StandardFault {
    /// A STALL, the request without effect.
    Stall,
    /// The answer cut to this many bytes.
    Short(usize),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_isochronous_interval_is_in_frames_or_microframes() {
        let endpoint =
            |interval| Endpoint { address: 0x81, attributes: 1, max_packet: 64, interval };
        assert_eq!(endpoint(1).iso_interval(Speed::Full), Duration::from_millis(1));
        assert_eq!(endpoint(4).iso_interval(Speed::Full), Duration::from_millis(8));
        assert_eq!(endpoint(1).iso_interval(Speed::High), Duration::from_micros(125));
        assert_eq!(endpoint(4).iso_interval(Speed::High), Duration::from_millis(1));
        // A value no descriptor may have still gives an interval.
        assert_eq!(endpoint(0).iso_interval(Speed::Full), Duration::from_millis(1));
    }
}
