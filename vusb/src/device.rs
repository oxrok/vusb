// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A USB device around a model: the standard requests addressed to the
//! device, the configuration state, endpoint halts, and the checks on
//! every transfer the host submits.

use crate::descriptor::{self, truncate};
use crate::model::{
    CONTROL, Descriptors, DeviceModel, Endpoint, RECIPIENT_DEVICE, RECIPIENT_ENDPOINT,
    RECIPIENT_INTERFACE, Reply, STANDARD, Setup, Speed, StandardFault, Violations,
};

const GET_STATUS: u8 = 0;
const CLEAR_FEATURE: u8 = 1;
const SET_FEATURE: u8 = 3;
const SET_ADDRESS: u8 = 5;
const GET_DESCRIPTOR: u8 = 6;
const SET_DESCRIPTOR: u8 = 7;
const GET_CONFIGURATION: u8 = 8;
const SET_CONFIGURATION: u8 = 9;
const GET_INTERFACE: u8 = 10;
const SET_INTERFACE: u8 = 11;

const FEATURE_ENDPOINT_HALT: u16 = 0;
const FEATURE_REMOTE_WAKEUP: u16 = 1;

/// A model on the bus.
pub struct Device {
    model: Box<dyn DeviceModel>,
    descriptors: Descriptors,
    config: u8,
    halted: Vec<u8>,
}

impl Device {
    /// Wraps a model; the device starts unconfigured.
    pub fn new(model: Box<dyn DeviceModel>) -> Device {
        let descriptors = model.descriptors().clone();
        Device { model, descriptors, config: 0, halted: Vec::new() }
    }

    /// The model's name.
    pub fn name(&self) -> &'static str {
        self.model.name()
    }

    /// Puts a fresh model behind the same bus state (configuration,
    /// halts), so a second run starts from the device's initial state
    /// while the host still sees the device it enumerated. When the
    /// fresh model's descriptors differ from those the host enumerated
    /// (a device that renumerated during the run), the caller must
    /// re-attach it, see [`Device::needs_reenumeration`]; the
    /// descriptors are taken over at the next [`Device::reset`].
    pub fn replace_model(&mut self, model: Box<dyn DeviceModel>) {
        self.model = model;
    }

    /// See [`DeviceModel::mark`].
    pub fn mark(&mut self) -> bool {
        self.model.mark()
    }

    /// Whether the model's descriptors differ from those the host
    /// enumerated, so the device has to be attached again.
    pub fn needs_reenumeration(&self) -> bool {
        self.model.descriptors() != &self.descriptors
    }

    /// The model's descriptors, which after a renumeration are not yet
    /// those the host enumerated ([`Device::descriptors`]).
    pub fn model_descriptors(&self) -> &Descriptors {
        self.model.descriptors()
    }

    /// See [`DeviceModel::take_renumeration`].
    pub fn take_renumeration(&mut self) -> bool {
        self.model.take_renumeration()
    }

    /// See [`DeviceModel::in_submitted`].
    pub fn in_submitted(&mut self, ep: u8, max: usize) {
        self.model.in_submitted(ep & 0x0f, max);
    }

    /// See [`DeviceModel::in_unlinked`].
    pub fn in_unlinked(&mut self, ep: u8, max: usize) -> Option<Vec<u8>> {
        self.model.in_unlinked(ep & 0x0f, max)
    }

    /// The descriptors.
    pub fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    /// The active `bConfigurationValue`, 0 when unconfigured.
    pub fn configuration(&self) -> u8 {
        self.config
    }

    /// An endpoint of the active configuration, by address.
    pub fn endpoint(&self, address: u8) -> Option<Endpoint> {
        if address.trailing_zeros() >= 4 {
            return Some(Endpoint {
                address,
                attributes: CONTROL,
                max_packet: u16::from(self.descriptors.device[7]),
                interval: 0,
            });
        }
        let config = self.descriptors.config_by_value(self.config)?;
        Descriptors::endpoints(config).into_iter().find(|e| e.address == address)
    }

    /// A bus reset or a new attach: unconfigured, halts cleared, the
    /// model reset, and its descriptors taken over (they change when a
    /// device renumerates).
    pub fn reset(&mut self) {
        self.config = 0;
        self.halted.clear();
        self.model.reset();
        self.descriptors = self.model.descriptors().clone();
    }

    /// A control transfer on endpoint 0.
    pub fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if !setup.is_in() && data.len() != usize::from(setup.length) {
            v.report(format!(
                "control OUT carries {} bytes, wLength says {}",
                data.len(),
                setup.length
            ));
        }
        let fault = if setup.kind() == STANDARD { self.model.fail_standard(setup) } else { None };
        if fault == Some(StandardFault::Stall) {
            return Reply::Stall;
        }
        let reply = if setup.kind() == STANDARD
            && let Some(reply) = self.standard(setup, v)
        {
            reply
        } else {
            self.model.control(setup, data, v)
        };
        match (reply, fault) {
            (Reply::Data(d), Some(StandardFault::Short(n))) if setup.is_in() => {
                Reply::Data(d.into_iter().take(n.min(usize::from(setup.length))).collect())
            }
            (Reply::Data(d), _) if setup.is_in() => Reply::Data(truncate(&d, setup.length)),
            (other, _) => other,
        }
    }

    /// The standard requests the device answers itself.
    fn standard(&mut self, setup: Setup, v: &mut Violations) -> Option<Reply> {
        let reply = match (setup.recipient(), setup.request, setup.is_in()) {
            (RECIPIENT_DEVICE | RECIPIENT_INTERFACE, GET_STATUS, true) => Reply::Data(vec![0, 0]),
            (RECIPIENT_DEVICE, CLEAR_FEATURE | SET_FEATURE, false) => {
                if setup.value == FEATURE_REMOTE_WAKEUP {
                    Reply::ack()
                } else {
                    v.report(format!("device feature {} is not one the device has", setup.value));
                    Reply::Stall
                }
            }
            (RECIPIENT_DEVICE, SET_ADDRESS, false) => Reply::ack(),
            (RECIPIENT_DEVICE, GET_DESCRIPTOR, true) => self.descriptor(setup, v),
            (RECIPIENT_DEVICE, SET_DESCRIPTOR, false) => {
                v.report("SET_DESCRIPTOR");
                Reply::Stall
            }
            (RECIPIENT_DEVICE, GET_CONFIGURATION, true) => Reply::Data(vec![self.config]),
            (RECIPIENT_DEVICE, SET_CONFIGURATION, false) => {
                let value = setup.value as u8;
                if value == 0 || self.descriptors.config_by_value(value).is_some() {
                    self.config = value;
                    self.halted.clear();
                    Reply::ack()
                } else {
                    v.report(format!("SET_CONFIGURATION {value}: no such configuration"));
                    Reply::Stall
                }
            }
            (RECIPIENT_INTERFACE, GET_INTERFACE, true) => Reply::Data(vec![0]),
            (RECIPIENT_INTERFACE, SET_INTERFACE, false) => {
                let number = (setup.index & 0xff) as u8;
                let alternate = setup.value as u8;
                let known = self.descriptors.config_by_value(self.config).is_some_and(|c| {
                    Descriptors::interfaces(c).iter().any(|i| i[0] == number && i[1] == alternate)
                });
                if known {
                    Reply::ack()
                } else {
                    v.report(format!(
                        "SET_INTERFACE {number} alternate {alternate}: not in the configuration"
                    ));
                    Reply::Stall
                }
            }
            (RECIPIENT_ENDPOINT, GET_STATUS, true) => {
                let address = (setup.index & 0xff) as u8;
                Reply::Data(vec![u8::from(self.halted.contains(&address)), 0])
            }
            (RECIPIENT_ENDPOINT, CLEAR_FEATURE | SET_FEATURE, false) => {
                let address = (setup.index & 0xff) as u8;
                if setup.value != FEATURE_ENDPOINT_HALT {
                    v.report(format!(
                        "endpoint feature {} is not one an endpoint has",
                        setup.value
                    ));
                    return Some(Reply::Stall);
                }
                if self.endpoint(address).is_none() {
                    v.report(format!(
                        "halt request for endpoint {address:#04x}, which the configuration lacks"
                    ));
                    return Some(Reply::Stall);
                }
                self.halted.retain(|&a| a != address);
                if setup.request == SET_FEATURE {
                    self.halted.push(address);
                }
                Reply::ack()
            }
            _ => return None,
        };
        Some(reply)
    }

    fn descriptor(&self, setup: Setup, v: &mut Violations) -> Reply {
        let kind = (setup.value >> 8) as u8;
        let index = usize::from(setup.value & 0xff);
        let bytes = match kind {
            descriptor::DEVICE => Some(self.descriptors.device.clone()),
            descriptor::CONFIGURATION => {
                let c = self.descriptors.configs.get(index).cloned();
                if c.is_none() {
                    v.report(format!(
                        "GET_DESCRIPTOR configuration {index}: the device has {}",
                        self.descriptors.configs.len()
                    ));
                }
                c
            }
            descriptor::STRING => {
                if index == 0 {
                    Some(descriptor::language_table())
                } else {
                    let s = self.descriptors.strings.get(index - 1).map(|s| descriptor::string(s));
                    if s.is_none() {
                        v.report(format!(
                            "GET_DESCRIPTOR string {index}: not one the descriptors name"
                        ));
                    }
                    s
                }
            }
            // A high speed device has a qualifier (what it would be at
            // full speed); a full or low speed one has not, and a host
            // may still ask. No device here has a BOS.
            descriptor::DEVICE_QUALIFIER if self.descriptors.speed == Speed::High => {
                Some(descriptor::device_qualifier(&self.descriptors.device))
            }
            descriptor::DEVICE_QUALIFIER | descriptor::BOS => None,
            other => {
                v.report(format!("GET_DESCRIPTOR of type {other:#04x} to the device"));
                None
            }
        };
        match bytes {
            Some(b) => Reply::Data(truncate(&b, setup.length)),
            None => Reply::Stall,
        }
    }

    fn check_endpoint(&self, address: u8, v: &mut Violations) -> Option<Endpoint> {
        if self.config == 0 {
            v.report(format!("transfer on endpoint {address:#04x} before SET_CONFIGURATION"));
        }
        let Some(ep) = self.endpoint(address) else {
            v.report(format!("transfer on endpoint {address:#04x}, which the configuration lacks"));
            return None;
        };
        Some(ep)
    }

    /// An IN transfer from endpoint number `ep`, up to `max` bytes.
    pub fn transfer_in(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        let address = 0x80 | (ep & 0x0f);
        if self.check_endpoint(address, v).is_none() {
            return Reply::Stall;
        }
        if self.halted.contains(&address) {
            return Reply::Stall;
        }
        self.model.in_transfer(ep & 0x0f, max, v)
    }

    /// An OUT transfer to endpoint number `ep`.
    pub fn transfer_out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        let address = ep & 0x0f;
        if self.check_endpoint(address, v).is_none() {
            return Reply::Stall;
        }
        if self.halted.contains(&address) {
            return Reply::Stall;
        }
        self.model.out(address, data, v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::examples::relay::UsbRelay;

    fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index, length }
    }

    #[test]
    fn enumerates_like_a_host() {
        let mut dev = Device::new(Box::new(UsbRelay::new(2)));
        let mut v = Violations::default();
        let Reply::Data(d) = dev.control(setup(0x80, GET_DESCRIPTOR, 0x0100, 0, 64), &[], &mut v)
        else {
            panic!()
        };
        assert_eq!(d.len(), 18);
        assert_eq!(&d[8..12], &[0xc0, 0x16, 0xdf, 0x05]);
        let Reply::Data(d) = dev.control(setup(0x80, GET_DESCRIPTOR, 0x0200, 0, 9), &[], &mut v)
        else {
            panic!()
        };
        let total = u16::from_le_bytes([d[2], d[3]]);
        let Reply::Data(d) =
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0200, 0, total), &[], &mut v)
        else {
            panic!()
        };
        assert_eq!(d.len(), usize::from(total));
        assert_eq!(
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0300, 0, 255), &[], &mut v),
            Reply::Data(vec![4, 3, 9, 4])
        );
        let Reply::Data(d) =
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0302, 0x0409, 255), &[], &mut v)
        else {
            panic!()
        };
        assert_eq!(
            String::from_utf16_lossy(
                &d[2..].chunks(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect::<Vec<_>>()
            ),
            "USBRelay2"
        );
        assert_eq!(
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0600, 0, 10), &[], &mut v),
            Reply::Stall,
            "a full speed device has no qualifier"
        );
        assert!(v.is_empty(), "{v:?}");
        // Before configuration the interrupt endpoint is not there.
        assert_eq!(dev.transfer_in(1, 64, &mut v), Reply::Stall);
        assert!(!v.is_empty());
        v.drain();
        assert_eq!(dev.control(setup(0x00, SET_CONFIGURATION, 1, 0, 0), &[], &mut v), Reply::ack());
        assert_eq!(dev.configuration(), 1);
        assert_eq!(dev.transfer_in(1, 64, &mut v), Reply::Nak);
        assert_eq!(
            dev.control(setup(0x82, GET_STATUS, 0, 0x81, 2), &[], &mut v),
            Reply::Data(vec![0, 0])
        );
        assert_eq!(dev.control(setup(0x02, SET_FEATURE, 0, 0x81, 0), &[], &mut v), Reply::ack());
        assert_eq!(dev.transfer_in(1, 64, &mut v), Reply::Stall);
        assert_eq!(dev.control(setup(0x02, CLEAR_FEATURE, 0, 0x81, 0), &[], &mut v), Reply::ack());
        assert_eq!(dev.transfer_in(1, 64, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(dev.control(setup(0x00, SET_CONFIGURATION, 7, 0, 0), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
    }

    /// A relay board whose model STALLs string 2 and the configuration,
    /// and cuts string 0.
    struct FailingStrings(UsbRelay);

    impl DeviceModel for FailingStrings {
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
            match (setup.request, setup.value) {
                (GET_DESCRIPTOR, 0x0302) | (SET_CONFIGURATION, _) => Some(StandardFault::Stall),
                (GET_DESCRIPTOR, 0x0300) => Some(StandardFault::Short(2)),
                _ => None,
            }
        }
    }

    #[test]
    fn a_model_can_fail_a_standard_request() {
        let mut dev = Device::new(Box::new(FailingStrings(UsbRelay::new(2))));
        let mut v = Violations::default();
        assert_eq!(
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0302, 0x0409, 255), &[], &mut v),
            Reply::Stall
        );
        assert_eq!(
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0300, 0, 255), &[], &mut v),
            Reply::Data(vec![4, 3])
        );
        // Not longer than wLength either.
        assert_eq!(
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0300, 0, 1), &[], &mut v),
            Reply::Data(vec![4])
        );
        let Reply::Data(d) = dev.control(setup(0x80, GET_DESCRIPTOR, 0x0100, 0, 64), &[], &mut v)
        else {
            panic!()
        };
        assert_eq!(d.len(), 18, "the others as ever");
        // A STALLed request has no effect.
        assert_eq!(dev.control(setup(0x00, SET_CONFIGURATION, 1, 0, 0), &[], &mut v), Reply::Stall);
        assert_eq!(dev.configuration(), 0);
        assert!(v.is_empty(), "{v:?}");
    }

    /// A bare high speed device, which has a qualifier.
    struct Bare(Descriptors);

    impl DeviceModel for Bare {
        fn name(&self) -> &'static str {
            "bare"
        }
        fn descriptors(&self) -> &Descriptors {
            &self.0
        }
        fn control(&mut self, _: Setup, _: &[u8], _: &mut Violations) -> Reply {
            Reply::Stall
        }
        fn out(&mut self, _: u8, _: &[u8], _: &mut Violations) -> Reply {
            Reply::Stall
        }
        fn in_transfer(&mut self, _: u8, _: usize, _: &mut Violations) -> Reply {
            Reply::Nak
        }
        fn reset(&mut self) {}
    }

    #[test]
    fn a_high_speed_device_has_a_qualifier() {
        let desc = crate::shells::ezusb::bare_descriptors(0x04b4, 0x8613, 0xa001);
        let mut dev = Device::new(Box::new(Bare(desc)));
        let mut v = Violations::default();
        let Reply::Data(q) = dev.control(setup(0x80, GET_DESCRIPTOR, 0x0600, 0, 10), &[], &mut v)
        else {
            panic!()
        };
        // bLength, type, bcdUSB 2.00, vendor class, 64 byte endpoint 0,
        // one configuration, reserved.
        assert_eq!(q, vec![10, 6, 0x00, 0x02, 0xff, 0xff, 0xff, 64, 1, 0]);
        assert_eq!(
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0600, 0, 4), &[], &mut v),
            Reply::Data(vec![10, 6, 0x00, 0x02]),
            "cut to wLength"
        );
        assert!(v.is_empty(), "{v:?}");
    }
}
