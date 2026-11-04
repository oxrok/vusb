// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The device model contract.

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
