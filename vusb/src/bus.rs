// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The bus: a host controller around a device, shared by every
//! transport and by in-process tests.
//!
//! A host submits [`Urb`]s and gets [`Completion`]s. What a host
//! controller does between the two lives here, whatever carries the
//! URBs:
//!
//! - A transfer the model answers with NAK stays held, as a URB does in
//!   a host controller, and is offered again at every [`Bus::poll`]
//!   until it completes or the host unlinks it.
//! - Transfers on one endpoint complete in the order they were
//!   submitted: one submitted behind a held one waits behind it, as a
//!   host controller's queue for the endpoint would.
//! - A bulk IN is offered to the device in whole packets, whatever the
//!   URB's length ([`Bus::offered_length`]); an answer longer than the
//!   URB is a babble, completed with the bytes that fit and
//!   [`Status::Babble`].
//! - An unlinked IN transfer that the device holds packets for
//!   completes with them ([`crate::model::DeviceModel::in_unlinked`]).
//! - An armed [`Arm`] is counted and fires once; an unplug drops the
//!   held transfers and every one submitted until the next
//!   [`Bus::reset`], as a device that is gone answers nothing.
//! - A renumeration the model asks for is passed up as a [`Signal`].
//!
//! Every transfer, violation and notice goes to the bus's
//! [`Observer`]s. The convenience methods [`Bus::control`],
//! [`Bus::read`] and [`Bus::write`] submit one URB and poll until it
//! completes, which is what a test wants.

use crate::arm::{Arm, Event};
use crate::device::Device;
use crate::model::{BULK, CONTROL, DeviceModel, Endpoint, Reply, Setup, Violations};
use crate::observer::{Notice, Observer, Outcome, SharedObserver, Transfer};

/// A transfer the host submits.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Urb {
    /// The host's number for it, which its completion carries.
    pub seqnum: u32,
    /// The endpoint number, 0 for a control transfer.
    pub ep: u8,
    /// Device to host; for a control transfer, the setup's direction.
    pub is_in: bool,
    /// The setup packet of a control transfer.
    pub setup: Option<Setup>,
    /// The length asked for (an IN), or of the data (an OUT).
    pub length: usize,
    /// The OUT payload; empty for an IN.
    pub data: Vec<u8>,
    /// Isochronous packets, 0 for the other transfer types. Isochronous
    /// transfers are not supported yet: one is a violation and a STALL.
    pub iso_packets: u32,
}

impl Urb {
    /// A control transfer with `data` as its OUT payload (empty for IN).
    pub fn control(seqnum: u32, setup: Setup, data: Vec<u8>) -> Urb {
        Urb {
            seqnum,
            ep: 0,
            is_in: setup.is_in(),
            setup: Some(setup),
            length: usize::from(setup.length),
            data,
            iso_packets: 0,
        }
    }

    /// An IN transfer of up to `length` bytes from endpoint number `ep`.
    pub fn read(seqnum: u32, ep: u8, length: usize) -> Urb {
        Urb { seqnum, ep, is_in: true, setup: None, length, data: Vec::new(), iso_packets: 0 }
    }

    /// An OUT transfer of `data` to endpoint number `ep`.
    pub fn write(seqnum: u32, ep: u8, data: Vec<u8>) -> Urb {
        Urb { seqnum, ep, is_in: false, setup: None, length: data.len(), data, iso_packets: 0 }
    }

    /// The endpoint address with its direction bit; 0 for control.
    pub fn address(&self) -> u8 {
        if self.ep == 0 {
            0
        } else if self.is_in {
            0x80 | self.ep
        } else {
            self.ep
        }
    }

    /// The queue the transfer belongs to.
    fn key(&self) -> (u8, bool) {
        (self.ep, self.is_in)
    }

    fn dir_word(&self) -> &'static str {
        if self.is_in { "IN" } else { "OUT" }
    }
}

/// How a transfer ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Status {
    /// Completed: the data of an IN, or an OUT taken.
    Ok,
    /// The device answered STALL (`-EPIPE` on Linux).
    Stall,
    /// The device sent more than the URB holds; the bytes that fit are
    /// in the completion (`-EOVERFLOW`).
    Babble,
    /// Unlinked by the host, or given up by [`Bus::control`] and its
    /// kind; the bytes the device held for it, if any, are in the
    /// completion (`-ECONNRESET`).
    Unlinked,
}

/// A completed transfer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Completion {
    /// The URB's number.
    pub seqnum: u32,
    /// How it ended.
    pub status: Status,
    /// Bytes transferred: the data of an IN, or how many of an OUT the
    /// device took.
    pub actual: usize,
    /// The data of an IN, cut to the URB.
    pub data: Vec<u8>,
}

impl Completion {
    /// Completed with data, or an accepted OUT.
    pub fn is_ok(&self) -> bool {
        self.status == Status::Ok
    }
}

/// A renumeration the model asked for: the ids it comes back with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Renumeration {
    /// `idVendor` after it.
    pub vendor: u16,
    /// `idProduct` after it.
    pub product: u16,
}

/// What a step of the bus tells the host beyond completions.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Signal {
    /// The model wants to leave the bus and come back with its new
    /// descriptors; the host takes it off and puts it back
    /// ([`Bus::reset`] is the new attach).
    Renumeration(Renumeration),
    /// An armed event fired. After an unplug the bus answers nothing
    /// until the next [`Bus::reset`].
    Fired(Event),
}

/// What one [`Bus::submit`] or [`Bus::poll`] produced.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Step {
    /// The transfers that completed, in order.
    pub completions: Vec<Completion>,
    /// What else happened, in order.
    pub signals: Vec<Signal>,
}

/// The outcome of an unlink.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Unlink {
    /// The transfer was not held: it completed already, or never was.
    NotHeld,
    /// The held transfer is gone; it never completes.
    Cancelled,
    /// The held transfer completed with what the device held for it
    /// ([`Status::Unlinked`]), before the unlink itself is answered.
    Completed(Completion),
}

/// A host controller around a device.
pub struct Bus {
    device: Device,
    observers: Vec<SharedObserver>,
    held: Vec<Urb>,
    armed: Option<Arm>,
    gone: bool,
    next: u32,
}

impl std::fmt::Debug for Bus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Bus")
            .field("device", &self.device)
            .field("held", &self.held)
            .field("armed", &self.armed)
            .field("gone", &self.gone)
            .finish_non_exhaustive()
    }
}

/// How many polls [`Bus::control`] and its kind give a transfer before
/// unlinking it, as a host's timeout would.
const PATIENCE: u32 = 10_000;

impl Bus {
    /// A bus with `device` on it, just attached.
    pub fn new(device: Device) -> Bus {
        Bus { device, observers: Vec::new(), held: Vec::new(), armed: None, gone: false, next: 1 }
    }

    /// An observer of everything the host does; several are called in
    /// the order given.
    #[must_use]
    pub fn with_observer(mut self, observer: SharedObserver) -> Bus {
        self.observers.push(observer);
        self
    }

    /// Adds an observer.
    pub fn add_observer(&mut self, observer: SharedObserver) {
        self.observers.push(observer);
    }

    /// The device.
    pub fn device(&self) -> &Device {
        &self.device
    }

    /// The device, to change it.
    pub fn device_mut(&mut self) -> &mut Device {
        &mut self.device
    }

    /// Puts a fresh model behind the same bus state, see
    /// [`Device::replace_model`]; disarms.
    pub fn replace_model(&mut self, model: Box<dyn DeviceModel>) {
        self.device.replace_model(model);
        self.armed = None;
    }

    /// Arms an event; one at a time, and a fresh model disarms it.
    pub fn arm(&mut self, arm: Arm) {
        self.armed = Some(arm);
    }

    /// A bus reset or a new attach: the device is reset, the held
    /// transfers are forgotten (the host's controller forgot them too),
    /// and an unplugged device is back.
    pub fn reset(&mut self) {
        self.device.reset();
        self.held.clear();
        self.gone = false;
    }

    /// Whether an unplug fired and no reset followed: the bus answers
    /// nothing.
    pub fn unplugged(&self) -> bool {
        self.gone
    }

    /// Transfers held.
    pub fn held(&self) -> usize {
        self.held.len()
    }

    fn observe(&self, f: impl Fn(&mut dyn Observer)) {
        for o in &self.observers {
            f(&mut *o.lock().expect("observer lock"));
        }
    }

    /// Tells the observers something about the run.
    pub fn comment(&self, message: &str) {
        self.notice(Notice::Comment(message));
    }

    /// Passes a notice to the observers.
    pub fn notice(&self, notice: Notice<'_>) {
        self.observe(|o| o.notice(notice));
    }

    /// The device leaves the bus as a pulled cable does: the held
    /// transfers are dropped, and nothing is answered until the next
    /// [`Bus::reset`]. What an armed unplug does, on request.
    pub fn unplug(&mut self) {
        self.gone = true;
        for u in std::mem::take(&mut self.held) {
            self.comment(&format!("seq {}: dropped, the device is unplugged", u.seqnum));
        }
    }

    /// Reports a violation the host committed outside any transfer.
    pub fn violation(&self, message: &str) {
        self.observe(|o| o.violation(message));
    }

    /// The length of an IN transfer as the device sees it. A bulk IN
    /// asks for whole packets: the host controller takes up to
    /// `wMaxPacketSize` bytes per IN token whatever the URB's length, so
    /// a device cannot tell a URB of 1 byte from one of 64 on a 64 byte
    /// endpoint. libusb passes any length (a driver may read 1 byte
    /// answers), nusb only multiples of the packet size; the length is
    /// rounded up to the packet size for the model and the observers,
    /// so the two read the same. Interrupt and control INs keep their
    /// length. The URB's own length still decides the outcome on the
    /// host: an answer longer than it is a babble.
    pub fn offered_length(&self, ep: u8, length: usize) -> usize {
        match self.device.endpoint(0x80 | ep) {
            Some(e) if e.transfer_type() == BULK && e.max_packet > 0 => {
                let mps = usize::from(e.max_packet);
                length.div_ceil(mps).max(1) * mps
            }
            _ => length,
        }
    }

    /// Submits a transfer: it completes now, or is held, or waits behind
    /// a held transfer of its endpoint.
    pub fn submit(&mut self, urb: Urb) -> Step {
        let mut step = Step::default();
        if self.gone {
            self.comment(&format!(
                "seq {}: ep {:#04x} {} dropped, the device is unplugged",
                urb.seqnum,
                urb.ep,
                urb.dir_word()
            ));
            return step;
        }
        if urb.ep != 0 && urb.is_in {
            // The URB's own length, as the host submitted it (not rounded
            // to the packet size as for the model's in_transfer()).
            self.device.in_submitted(urb.ep, urb.length);
        }
        let queued = urb.ep != 0 && self.held.iter().any(|u| u.key() == urb.key());
        if queued {
            self.comment(&format!(
                "seq {}: ep {:#04x} {} queued behind a held transfer",
                urb.seqnum,
                urb.ep,
                urb.dir_word()
            ));
            self.held.push(urb);
            return step;
        }
        self.offer(urb, true, &mut step);
        step
    }

    /// Offers every held transfer again, the ones behind a transfer
    /// still held excepted. A transport calls it after every submission
    /// and every so often (20 ms is what a kernel's retry feels like).
    pub fn poll(&mut self) -> Step {
        let mut step = Step::default();
        let pending = std::mem::take(&mut self.held);
        // Endpoints whose oldest transfer is still held this round: the
        // transfers behind it wait.
        let mut blocked: Vec<(u8, bool)> = Vec::new();
        for urb in pending {
            if self.gone {
                self.comment(&format!("seq {}: dropped, the device is unplugged", urb.seqnum));
                continue;
            }
            let key = urb.key();
            if urb.ep != 0 && blocked.contains(&key) {
                self.held.push(urb);
                continue;
            }
            if self.offer(urb, false, &mut step) {
                blocked.push(key);
            }
        }
        step
    }

    /// Cancels a held transfer.
    pub fn unlink(&mut self, seqnum: u32) -> Unlink {
        let Some(i) = self.held.iter().position(|u| u.seqnum == seqnum) else {
            self.comment(&format!("unlink seq {seqnum}: already completed"));
            return Unlink::NotHeld;
        };
        // Only the oldest of its endpoint has received anything; nusb
        // cancels the newest first.
        let key = self.held[i].key();
        let oldest = self.held.iter().position(|u| u.key() == key) == Some(i);
        let urb = self.held.remove(i);
        self.comment(&format!("unlink seq {seqnum}: ep {:#04x} {} held", urb.ep, urb.dir_word()));
        if oldest && let Some(c) = self.complete_unlinked(&urb) {
            return Unlink::Completed(c);
        }
        Unlink::Cancelled
    }

    /// Offers a transfer; whether the device answered NAK.
    fn offer(&mut self, urb: Urb, fresh: bool, step: &mut Step) -> bool {
        let (reply, v) = self.perform(&urb);
        let nak = reply == Reply::Nak;
        let event = self.fired(&urb, &reply);
        self.finish(urb, reply, v, fresh, step);
        self.check_renumeration(step);
        self.event_fired(event, step);
        nak
    }

    fn perform(&mut self, urb: &Urb) -> (Reply, Violations) {
        let mut v = Violations::default();
        if urb.iso_packets > 0 {
            v.report(format!("isochronous URB with {} packets; not supported", urb.iso_packets));
            return (Reply::Stall, v);
        }
        let reply = match urb.setup {
            Some(setup) => self.device.control(setup, &urb.data, &mut v),
            None if urb.is_in => {
                let length = self.offered_length(urb.ep, urb.length);
                self.device.transfer_in(urb.ep, length, &mut v)
            }
            None => self.device.transfer_out(urb.ep, &urb.data, &mut v),
        };
        (reply, v)
    }

    /// After every offer: counts it for the armed event, asks the model
    /// for its mark, and returns the event when it fires (once).
    fn fired(&mut self, urb: &Urb, reply: &Reply) -> Option<Event> {
        let point = self.device.mark();
        let arm = self.armed.as_mut()?;
        if arm.fires(urb.ep, urb.is_in, reply, point) {
            let event = arm.event.clone();
            self.armed = None;
            return Some(event);
        }
        None
    }

    fn event_fired(&mut self, event: Option<Event>, step: &mut Step) {
        let Some(event) = event else { return };
        self.observe(|o| o.notice(Notice::Fired(&event)));
        if event == Event::Unplug {
            self.unplug();
        }
        step.signals.push(Signal::Fired(event));
    }

    /// The observers get the notice at once, right after the transfer
    /// that caused it (a tool that ends right after the transfer would
    /// otherwise see it or not depending on timing).
    fn check_renumeration(&mut self, step: &mut Step) {
        if !self.device.take_renumeration() {
            return;
        }
        let m = self.device.model_descriptors();
        let r = Renumeration { vendor: m.vendor_id(), product: m.product_id() };
        self.comment("the model asks to renumerate");
        self.observe(|o| o.notice(Notice::Renumeration { vendor: r.vendor, product: r.product }));
        step.signals.push(Signal::Renumeration(r));
    }

    /// A held IN transfer just unlinked: when the model says it holds
    /// data, reports it and completes the URB with it.
    fn complete_unlinked(&mut self, urb: &Urb) -> Option<Completion> {
        if !urb.is_in || urb.ep == 0 {
            return None;
        }
        let offered = self.offered_length(urb.ep, urb.length);
        let d = self.device.in_unlinked(urb.ep, offered)?;
        let transfer = Transfer {
            seqnum: urb.seqnum,
            endpoint: urb.address(),
            kind: self.device.endpoint(urb.address()).map(Endpoint::transfer_type),
            setup: None,
            is_in: true,
            requested: urb.length,
            offered,
            data: Vec::new(),
            reply: Reply::Data(d.clone()),
            outcome: Outcome::Unlinked,
        };
        self.observe(|o| o.transfer(&transfer));
        let d = &d[..d.len().min(urb.length)];
        Some(Completion {
            seqnum: urb.seqnum,
            status: Status::Unlinked,
            actual: d.len(),
            data: d.to_vec(),
        })
    }

    /// Reports the outcome and completes the URB, or holds it; `fresh`
    /// says the URB was just submitted rather than retried (a retried
    /// URB the device still NAKs is not reported again).
    fn finish(&mut self, urb: Urb, reply: Reply, mut v: Violations, fresh: bool, step: &mut Step) {
        let control = urb.setup.is_some();
        let kind = if control {
            Some(CONTROL)
        } else {
            self.device.endpoint(urb.address()).map(Endpoint::transfer_type)
        };
        let offered = if !control && urb.is_in {
            self.offered_length(urb.ep, urb.length)
        } else {
            urb.length
        };
        let outcome = match &reply {
            Reply::Nak => Outcome::Held,
            Reply::Data(d) if !control && urb.is_in && d.len() > urb.length => Outcome::Babble,
            _ => Outcome::Completed,
        };
        let violations = v.drain();
        self.observe(|o| {
            for message in &violations {
                o.violation(message);
            }
        });
        if fresh || outcome != Outcome::Held {
            let transfer = Transfer {
                seqnum: urb.seqnum,
                endpoint: urb.address(),
                kind,
                setup: urb.setup,
                is_in: urb.is_in,
                requested: urb.length,
                offered,
                data: urb.data.clone(),
                reply: reply.clone(),
                outcome,
            };
            self.observe(|o| o.transfer(&transfer));
        }
        let completion = match reply {
            Reply::Nak => {
                self.held.push(urb);
                return;
            }
            Reply::Stall => Completion {
                seqnum: urb.seqnum,
                status: Status::Stall,
                actual: 0,
                data: Vec::new(),
            },
            Reply::Data(d) if outcome == Outcome::Babble => {
                // More than the URB holds: the HCD ends it with -EOVERFLOW
                // and the bytes that fit, as for a babbling device.
                let d = &d[..urb.length];
                Completion {
                    seqnum: urb.seqnum,
                    status: Status::Babble,
                    actual: d.len(),
                    data: d.to_vec(),
                }
            }
            Reply::Data(d) if urb.is_in => {
                // A control IN longer than wLength is cut, as the model
                // trait says.
                let d = &d[..d.len().min(urb.length)];
                Completion {
                    seqnum: urb.seqnum,
                    status: Status::Ok,
                    actual: d.len(),
                    data: d.to_vec(),
                }
            }
            Reply::Data(d) => {
                let taken = if d.is_empty() { urb.data.len() } else { d.len().min(urb.data.len()) };
                Completion {
                    seqnum: urb.seqnum,
                    status: Status::Ok,
                    actual: taken,
                    data: Vec::new(),
                }
            }
        };
        step.completions.push(completion);
    }

    fn next_seqnum(&mut self) -> u32 {
        let n = self.next;
        self.next = self.next.wrapping_add(1).max(1);
        n
    }

    /// Submits a transfer and polls until it completes, up to a host's
    /// patience; then unlinks it ([`Status::Unlinked`]). The signals of
    /// those steps are dropped: a test that wants them uses
    /// [`Bus::submit`] and [`Bus::poll`].
    fn run(&mut self, urb: Urb) -> Completion {
        let seqnum = urb.seqnum;
        let mut step = self.submit(urb);
        for _ in 0..PATIENCE {
            if let Some(i) = step.completions.iter().position(|c| c.seqnum == seqnum) {
                return step.completions.swap_remove(i);
            }
            if self.gone {
                break;
            }
            step = self.poll();
        }
        match self.unlink(seqnum) {
            Unlink::Completed(c) => c,
            Unlink::Cancelled | Unlink::NotHeld => {
                Completion { seqnum, status: Status::Unlinked, actual: 0, data: Vec::new() }
            }
        }
    }

    /// A control transfer, run to completion.
    pub fn control(&mut self, setup: Setup, data: &[u8]) -> Completion {
        let seqnum = self.next_seqnum();
        self.run(Urb::control(seqnum, setup, data.to_vec()))
    }

    /// An IN transfer of up to `length` bytes from endpoint number `ep`,
    /// run to completion.
    pub fn read(&mut self, ep: u8, length: usize) -> Completion {
        let seqnum = self.next_seqnum();
        self.run(Urb::read(seqnum, ep, length))
    }

    /// An OUT transfer of `data` to endpoint number `ep`, run to
    /// completion.
    pub fn write(&mut self, ep: u8, data: &[u8]) -> Completion {
        let seqnum = self.next_seqnum();
        self.run(Urb::write(seqnum, ep, data.to_vec()))
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::arm::When;
    use crate::descriptor::{self, DeviceDescriptor};
    use crate::examples::relay::UsbRelay;
    use crate::model::Descriptors;
    use crate::observer::Recorder;

    fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index, length }
    }

    fn recorded(device: Device) -> (Bus, Arc<Mutex<Recorder>>) {
        let rec = Arc::new(Mutex::new(Recorder::default()));
        let shared: SharedObserver = rec.clone();
        (Bus::new(device).with_observer(shared), rec)
    }

    /// A vendor device with a bulk IN on 0x82 (64 byte packets) and an
    /// interrupt IN on 0x81 (8 byte packets).
    fn vendor_descriptors() -> Descriptors {
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 2, [0xff, 0, 0], 0));
        body.extend_from_slice(&descriptor::endpoint(0x81, 3, 8, 1));
        body.extend_from_slice(&descriptor::endpoint(0x82, 2, 64, 0));
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: 0x1234,
            product: 0x5678,
            device: 0x0100,
            strings: [0, 0, 0],
            configurations: 1,
        };
        Descriptors {
            speed: crate::model::Speed::Full,
            device: device.bytes(),
            configs: vec![descriptor::configuration(1, 0, 0x80, 50, &body)],
            strings: Vec::new(),
        }
    }

    /// Answers each IN with `answer` bytes and remembers the lengths it
    /// was given.
    struct Babbler {
        descriptors: Descriptors,
        answer: usize,
        asked: Vec<(&'static str, u8, usize)>,
    }

    impl DeviceModel for Babbler {
        fn name(&self) -> &'static str {
            "babbler"
        }
        fn descriptors(&self) -> &Descriptors {
            &self.descriptors
        }
        fn control(&mut self, _: Setup, _: &[u8], _: &mut Violations) -> Reply {
            Reply::ack()
        }
        fn out(&mut self, _: u8, _: &[u8], _: &mut Violations) -> Reply {
            Reply::ack()
        }
        fn in_transfer(&mut self, ep: u8, max: usize, _: &mut Violations) -> Reply {
            self.asked.push(("transfer", ep, max));
            Reply::Data(vec![0xa5; self.answer])
        }
        fn in_submitted(&mut self, ep: u8, max: usize) {
            self.asked.push(("submitted", ep, max));
        }
        fn reset(&mut self) {}
    }

    /// Holds each bulk IN, and has 64 bytes for one that is unlinked.
    struct Holder(Descriptors);

    impl DeviceModel for Holder {
        fn name(&self) -> &'static str {
            "holder"
        }
        fn descriptors(&self) -> &Descriptors {
            &self.0
        }
        fn control(&mut self, _: Setup, _: &[u8], _: &mut Violations) -> Reply {
            Reply::ack()
        }
        fn out(&mut self, _: u8, _: &[u8], _: &mut Violations) -> Reply {
            Reply::ack()
        }
        fn in_transfer(&mut self, _: u8, _: usize, _: &mut Violations) -> Reply {
            Reply::Nak
        }
        fn in_unlinked(&mut self, _: u8, _: usize) -> Option<Vec<u8>> {
            Some(vec![0x5a; 64])
        }
        fn reset(&mut self) {}
    }

    #[test]
    fn drives_a_relay_board_in_process() {
        let (mut bus, rec) = recorded(Device::new(Box::new(UsbRelay::new(2))));
        let c = bus.control(setup(0x80, 6, 0x0100, 0, 18), &[]);
        assert_eq!((c.status, c.actual, c.data.len()), (Status::Ok, 18, 18));
        assert!(bus.control(setup(0x00, 9, 1, 0, 0), &[]).is_ok());
        assert_eq!(bus.control(setup(0xa1, 1, 0x0300, 0, 8), &[]).data, b"ABCDE\0\0\0");
        let c = bus.control(setup(0x21, 9, 0x0300, 0, 8), &[0xff, 2, 0, 0, 0, 0, 0, 0]);
        assert_eq!((c.status, c.actual), (Status::Ok, 8));
        assert_eq!(bus.control(setup(0xa1, 1, 0x0300, 0, 8), &[]).data, b"ABCDE\0\0\x02");
        // A fresh model behind the configured device: relays off again.
        bus.replace_model(Box::new(UsbRelay::new(2)));
        assert_eq!(bus.control(setup(0xa1, 1, 0x0300, 0, 8), &[]).data, b"ABCDE\0\0\0");
        // The board never sends an input report: a read is held until
        // the host gives up.
        let c = bus.read(1, 64);
        assert_eq!((c.status, c.actual), (Status::Unlinked, 0));
        assert_eq!(bus.held(), 0);
        assert_eq!(bus.control(setup(0x80, 6, 0x0600, 0, 10), &[]).status, Status::Stall);
        let rec = rec.lock().unwrap();
        assert!(rec.violations.is_empty(), "{:?}", rec.violations);
        let first = &rec.transfers[0];
        assert_eq!((first.seqnum, first.endpoint, first.kind), (1, 0, Some(CONTROL)));
        assert_eq!(first.setup, Some(setup(0x80, 6, 0x0100, 0, 18)));
        let held: Vec<(u32, u8)> = rec
            .transfers
            .iter()
            .filter(|t| t.outcome == Outcome::Held)
            .map(|t| (t.seqnum, t.endpoint))
            .collect();
        assert_eq!(held, [(7, 0x81)], "reported held once, not at every poll");
        assert!(
            rec.notices.iter().any(|n| n == "unlink seq 7: ep 0x01 IN held"),
            "{:?}",
            rec.notices
        );
    }

    #[test]
    fn transfers_of_an_endpoint_complete_in_order() {
        let (mut bus, rec) = recorded(Device::new(Box::new(Holder(vendor_descriptors()))));
        assert!(bus.control(setup(0x00, 9, 1, 0, 0), &[]).is_ok());
        assert_eq!(bus.submit(Urb::read(2, 2, 64)), Step::default());
        assert_eq!(bus.submit(Urb::read(3, 2, 64)), Step::default());
        assert_eq!(bus.held(), 2);
        assert_eq!(bus.poll(), Step::default(), "both still held");
        // The younger goes first and comes back with nothing.
        assert_eq!(bus.unlink(3), Unlink::Cancelled);
        // The oldest completes with the data.
        let Unlink::Completed(c) = bus.unlink(2) else { panic!() };
        assert_eq!((c.seqnum, c.status, c.data), (2, Status::Unlinked, vec![0x5a; 64]));
        assert_eq!(bus.unlink(2), Unlink::NotHeld);
        let rec = rec.lock().unwrap();
        let outcomes: Vec<(u32, Outcome)> = rec
            .transfers
            .iter()
            .filter(|t| t.endpoint == 0x82)
            .map(|t| (t.seqnum, t.outcome))
            .collect();
        assert_eq!(outcomes, [(2, Outcome::Held), (2, Outcome::Unlinked)], "3 was never offered");
        assert!(rec.notices.iter().any(|n| n.starts_with("seq 3:") && n.contains("queued")));
    }

    #[test]
    fn an_answer_longer_than_the_urb_is_a_babble() {
        let model = Babbler { descriptors: vendor_descriptors(), answer: 6, asked: Vec::new() };
        let (mut bus, rec) = recorded(Device::new(Box::new(model)));
        assert!(bus.control(setup(0x00, 9, 1, 0, 0), &[]).is_ok());
        // Bulk: the model is asked for a whole packet, the URB of 4 bytes
        // babbles with the 4 that fit.
        let c = bus.read(2, 4);
        assert_eq!((c.status, c.data), (Status::Babble, vec![0xa5; 4]));
        // A URB that holds the answer completes.
        let c = bus.read(2, 6);
        assert_eq!((c.status, c.data), (Status::Ok, vec![0xa5; 6]));
        // Interrupt: the length is not rounded, and the same holds.
        let c = bus.read(1, 2);
        assert_eq!((c.status, c.data), (Status::Babble, vec![0xa5; 2]));
        let rec = rec.lock().unwrap();
        let seen: Vec<(usize, usize, Outcome)> = rec
            .transfers
            .iter()
            .filter(|t| t.endpoint != 0)
            .map(|t| (t.requested, t.offered, t.outcome))
            .collect();
        assert_eq!(
            seen,
            [(4, 64, Outcome::Babble), (6, 64, Outcome::Completed), (2, 2, Outcome::Babble)]
        );
        assert_eq!(rec.transfers[1].reply, Reply::Data(vec![0xa5; 6]), "the reply before the cut");
    }

    #[test]
    fn an_isochronous_urb_is_a_violation() {
        let (mut bus, rec) = recorded(Device::new(Box::new(Holder(vendor_descriptors()))));
        assert!(bus.control(setup(0x00, 9, 1, 0, 0), &[]).is_ok());
        let urb = Urb { iso_packets: 2, ..Urb::read(9, 2, 128) };
        let step = bus.submit(urb);
        assert_eq!(step.completions.len(), 1);
        assert_eq!(step.completions[0].status, Status::Stall);
        assert_eq!(rec.lock().unwrap().violations.len(), 1);
    }

    #[test]
    fn armed_events_fire_once_at_their_point() {
        let (mut bus, rec) = recorded(Device::new(Box::new(UsbRelay::new(2))));
        bus.arm(Arm::new(Event::Unplug, When::Out { ep: 0, n: 2 }));
        assert!(bus.control(setup(0x80, 6, 0x0100, 0, 18), &[]).is_ok(), "an IN does not count");
        let step = bus.submit(Urb::control(50, setup(0x00, 9, 1, 0, 0), Vec::new()));
        assert_eq!(step.signals, vec![]);
        // Held reads go when the device is unplugged.
        assert_eq!(bus.submit(Urb::read(51, 1, 64)), Step::default());
        let step = bus.submit(Urb::control(52, setup(0x00, 9, 1, 0, 0), Vec::new()));
        assert_eq!(step.completions.len(), 1);
        assert_eq!(step.signals, [Signal::Fired(Event::Unplug)]);
        assert!(bus.unplugged());
        assert_eq!(bus.held(), 0);
        // Gone: nothing is answered until the next attach.
        assert_eq!(
            bus.submit(Urb::control(53, setup(0x80, 6, 0x0100, 0, 18), Vec::new())),
            Step::default()
        );
        bus.reset();
        assert!(!bus.unplugged());
        assert!(bus.control(setup(0x80, 6, 0x0100, 0, 18), &[]).is_ok());
        // A mark leaves the device alone; a fresh model disarms.
        bus.arm(Arm::parse("mark snapshot in=0:1").unwrap());
        bus.replace_model(Box::new(UsbRelay::new(2)));
        let step = bus.submit(Urb::control(60, setup(0x80, 6, 0x0100, 0, 18), Vec::new()));
        assert!(step.signals.is_empty(), "disarmed");
        bus.arm(Arm::parse("mark snapshot in=0:1").unwrap());
        let step = bus.submit(Urb::control(61, setup(0x80, 6, 0x0100, 0, 18), Vec::new()));
        assert_eq!(step.signals, [Signal::Fired(Event::Mark("snapshot".into()))]);
        assert!(!bus.unplugged());
        let rec = rec.lock().unwrap();
        assert!(rec.notices.iter().any(|n| n == "event unplug"), "{:?}", rec.notices);
        assert!(rec.notices.iter().any(|n| n == "seq 51: dropped, the device is unplugged"));
        assert!(rec.notices.iter().any(|n| n == "event snapshot"));
    }

    #[test]
    fn the_model_asks_to_renumerate() {
        use crate::shells::ezusb::{self, CPUCS, Chip, Loader, REQ_FIRMWARE_LOAD};

        /// A bare FX2 whose firmware, once released, is a relay board.
        struct Fx2 {
            loader: Loader,
            descriptors: Descriptors,
            renumerate: bool,
        }
        impl DeviceModel for Fx2 {
            fn name(&self) -> &'static str {
                "fx2"
            }
            fn descriptors(&self) -> &Descriptors {
                &self.descriptors
            }
            fn control(&mut self, s: Setup, d: &[u8], v: &mut Violations) -> Reply {
                match self.loader.control(s, d, v) {
                    Some((reply, released)) => {
                        if released {
                            self.descriptors = UsbRelay::new(2).descriptors().clone();
                            self.renumerate = true;
                        }
                        reply
                    }
                    None => Reply::Stall,
                }
            }
            fn out(&mut self, _: u8, _: &[u8], _: &mut Violations) -> Reply {
                Reply::Stall
            }
            fn in_transfer(&mut self, _: u8, _: usize, _: &mut Violations) -> Reply {
                Reply::Nak
            }
            fn reset(&mut self) {}
            fn take_renumeration(&mut self) -> bool {
                std::mem::take(&mut self.renumerate)
            }
        }
        let model = Fx2 {
            loader: Loader::new(Chip::Fx2lp),
            descriptors: ezusb::bare_descriptors(0x04b4, 0x8613, 0xa001),
            renumerate: false,
        };
        let (mut bus, rec) = recorded(Device::new(Box::new(model)));
        assert_eq!(bus.device().descriptors().vendor_id(), 0x04b4);
        let load = |value: u16, len: u16| setup(0x40, REQ_FIRMWARE_LOAD, value, 0, len);
        assert_eq!(bus.submit(Urb::control(1, load(CPUCS, 1), vec![1])).signals, vec![]);
        assert_eq!(bus.submit(Urb::control(2, load(0, 4), vec![1, 2, 3, 4])).signals, vec![]);
        let step = bus.submit(Urb::control(3, load(CPUCS, 1), vec![0]));
        assert_eq!(
            step.signals,
            [Signal::Renumeration(Renumeration { vendor: 0x16c0, product: 0x05df })]
        );
        assert!(bus.device().needs_reenumeration());
        // The new attach takes the new descriptors over.
        bus.reset();
        assert_eq!(bus.device().descriptors().vendor_id(), 0x16c0);
        assert!(!bus.device().needs_reenumeration());
        let rec = rec.lock().unwrap();
        assert!(rec.notices.iter().any(|n| n == "renumeration 16c0:05df"), "{:?}", rec.notices);
    }
}
