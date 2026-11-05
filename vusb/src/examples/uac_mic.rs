// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A USB microphone that hears a pure tone: a USB Audio Class 1 device
//! with one isochronous IN endpoint, the shortest model with a stream.
//! On Linux `snd-usb-audio` makes it a sound card, and `arecord`
//! records the tone.
//!
//! Mono, 16 bit, 48 kHz: 48 samples, 96 bytes, in every 1 ms frame.
//! The model keeps no clock. Each [`DeviceModel::iso_in`] is one frame,
//! so the tone's phase is a count of the samples sent: a recording has
//! exactly the samples the model produced, however fast the host asked.

use std::f64::consts::TAU;

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, Reply, Setup, Speed, Violations};
use crate::options::parse_number;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "uac-mic";
/// One line about the device and its options, for a host that lists its models.
pub const ABOUT: &str = "a USB Audio microphone that hears a tone (48 kHz mono, isochronous IN); \
     tone=HZ (20 to 20000, default 1000), silent=N (no packet in every Nth frame)";

/// The sample rate.
pub const RATE: u32 = 48_000;
/// Samples in one 1 ms frame.
pub const SAMPLES_PER_FRAME: usize = 48;
/// The isochronous IN endpoint's number.
pub const EP_AUDIO: u8 = 1;
/// The tone's amplitude, half of full scale.
const AMPLITUDE: f64 = 16384.0;

const AUDIO: u8 = 1;
const AUDIO_CONTROL: u8 = 1;
const AUDIO_STREAMING: u8 = 2;
const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

/// The device.
#[derive(Clone, Debug)]
pub struct UacMic {
    descriptors: Descriptors,
    tone: u32,
    /// No packet in every this many frames: a device that drops out.
    silent: Option<u32>,
    /// Samples sent since power on.
    sample: u64,
    frame: u32,
}

impl UacMic {
    /// A microphone hearing a tone of `tone` Hz.
    pub fn new(tone: u32) -> UacMic {
        UacMic { descriptors: descriptors(), tone, silent: None, sample: 0, frame: 0 }
    }

    /// The samples of the next frame, as the device sends them:
    /// little endian, 16 bit, signed.
    fn frame(&mut self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(SAMPLES_PER_FRAME * 2);
        for _ in 0..SAMPLES_PER_FRAME {
            bytes.extend_from_slice(&sample(self.tone, self.sample).to_le_bytes());
            self.sample += 1;
        }
        bytes
    }
}

/// Sample `n` of a tone of `tone` Hz at [`RATE`].
pub fn sample(tone: u32, n: u64) -> i16 {
    // The phase in whole samples of one second, so it never loses
    // precision however long the device runs.
    let phase = (n * u64::from(tone)) % u64::from(RATE);
    let value = AMPLITUDE * (TAU * phase as f64 / f64::from(RATE)).sin();
    value.round() as i16
}

/// The descriptors: an audio control interface with a microphone
/// terminal wired to a USB streaming terminal, and an audio streaming
/// interface whose alternate setting 1 has the endpoint.
fn descriptors() -> Descriptors {
    let mut body = Vec::new();
    // Audio control: header, input terminal (microphone), output
    // terminal (USB streaming) fed by it.
    body.extend_from_slice(&descriptor::interface(0, 0, 0, [AUDIO, AUDIO_CONTROL, 0], 0));
    body.extend_from_slice(&[9, CS_INTERFACE, 0x01, 0x00, 0x01, 30, 0, 1, 1]);
    body.extend_from_slice(&[12, CS_INTERFACE, 0x02, 1, 0x01, 0x02, 0, 1, 0, 0, 0, 0]);
    body.extend_from_slice(&[9, CS_INTERFACE, 0x03, 2, 0x01, 0x01, 0, 1, 0]);
    // Audio streaming: no endpoint until the host selects alternate 1.
    body.extend_from_slice(&descriptor::interface(1, 0, 0, [AUDIO, AUDIO_STREAMING, 0], 0));
    body.extend_from_slice(&descriptor::interface(1, 1, 1, [AUDIO, AUDIO_STREAMING, 0], 0));
    // General: from terminal 2, one frame of delay, PCM.
    body.extend_from_slice(&[7, CS_INTERFACE, 0x01, 2, 1, 0x01, 0x00]);
    // Format type I: one channel, two bytes a sample, 16 bits, one rate.
    let rate = RATE.to_le_bytes();
    body.extend_from_slice(&[11, CS_INTERFACE, 0x02, 1, 1, 2, 16, 1, rate[0], rate[1], rate[2]]);
    // The endpoint, in an audio device nine bytes long: isochronous,
    // asynchronous, a packet every frame.
    let max = (SAMPLES_PER_FRAME as u16 * 2).to_le_bytes();
    body.extend_from_slice(&[
        9,
        descriptor::ENDPOINT,
        0x80 | EP_AUDIO,
        0x05,
        max[0],
        max[1],
        1,
        0,
        0,
    ]);
    // Its class descriptor: no sampling frequency control.
    body.extend_from_slice(&[7, CS_ENDPOINT, 0x01, 0x00, 0, 0, 0]);
    let device = DeviceDescriptor {
        usb: 0x0200,
        class: [0, 0, 0],
        max_packet0: 64,
        // pid.codes' test ids.
        vendor: 0x1209,
        product: 0x0003,
        device: 0x0100,
        strings: [1, 2, 0],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::Full,
        device: device.bytes(),
        configs: vec![descriptor::configuration(1, 0, 0x80, 100, &body)],
        strings: vec!["vusb".to_owned(), "Tone microphone".to_owned()],
    }
}

impl DeviceModel for UacMic {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        // The device has no feature unit and no sampling frequency
        // control, so no class request is its own.
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the microphone answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the microphone has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, ep: u8, _max: usize, v: &mut Violations) -> Reply {
        v.report(format!("IN transfer from endpoint {ep}, which is isochronous"));
        Reply::Stall
    }

    fn iso_in(&mut self, ep: u8, max: usize, v: &mut Violations) -> Option<Vec<u8>> {
        if ep != EP_AUDIO {
            v.report(format!("isochronous IN on endpoint {ep}; the audio is on {EP_AUDIO}"));
            return None;
        }
        if max < SAMPLES_PER_FRAME * 2 {
            v.report(format!(
                "a packet of {max} bytes; a frame of audio is {}",
                SAMPLES_PER_FRAME * 2
            ));
        }
        self.frame += 1;
        if self.silent.is_some_and(|n| self.frame.is_multiple_of(n)) {
            return None;
        }
        Some(self.frame())
    }

    fn reset(&mut self) {
        self.sample = 0;
        self.frame = 0;
    }
}

/// The device, from `tone=HZ` and `silent=N`.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut mic = UacMic::new(1000);
    for (k, v) in options {
        match k.as_str() {
            "tone" => mic.tone = parse_number(k, v, 20..=20_000)?,
            "silent" => mic.silent = Some(parse_number(k, v, 2..=1_000_000)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(mic))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{Bus, Status};
    use crate::device::Device;
    use crate::host::enumerate;
    use crate::model::ISOCHRONOUS;

    #[test]
    fn records_a_tone_frame_by_frame() {
        let mut bus = Bus::new(Device::new(build(&[]).unwrap()));
        let e = enumerate(&mut bus).unwrap();
        assert_eq!(e.product, "Tone microphone");
        let eps = e.endpoints();
        assert_eq!(eps.len(), 1);
        assert_eq!(
            (eps[0].address, eps[0].transfer_type(), eps[0].max_packet),
            (0x81, ISOCHRONOUS, 96)
        );
        // The host selects the streaming alternate setting, then reads
        // ten frames: ten periods of a 1 kHz tone at 48 kHz.
        let select = Setup { request_type: 0x01, request: 11, value: 1, index: 1, length: 0 };
        assert!(bus.control(select, &[]).is_ok());
        let c = bus.iso_read(EP_AUDIO, &[96; 10]);
        assert_eq!((c.status, c.actual), (Status::Ok, 960));
        let samples: Vec<i16> =
            c.data.as_chunks::<2>().0.iter().map(|b| i16::from_le_bytes(*b)).collect();
        assert_eq!(samples[0], 0);
        assert_eq!(samples[12], 16384, "the peak, a quarter period in");
        assert_eq!(samples[36], -16384);
        assert_eq!(&samples[..48], &samples[48..96], "one period a frame");
        let crossings = samples.windows(2).filter(|w| w[0] < 0 && w[1] >= 0).count();
        assert_eq!(crossings, 9, "ten periods");
    }

    #[test]
    fn a_dropout_is_a_missed_packet_and_the_tone_goes_on() {
        let options =
            [("silent".to_owned(), "3".to_owned()), ("tone".to_owned(), "500".to_owned())];
        let mut bus = Bus::new(Device::new(build(&options).unwrap()));
        enumerate(&mut bus).unwrap();
        let c = bus.iso_read(EP_AUDIO, &[96; 4]);
        let statuses: Vec<Status> = c.iso.iter().map(|p| p.status).collect();
        assert_eq!(statuses, [Status::Ok, Status::Ok, Status::Missed, Status::Ok]);
        assert_eq!(c.actual, 96 * 3);
        // 500 Hz: the third frame sent continues where the second ended.
        let at = |n: usize| i16::from_le_bytes([c.data[n * 2], c.data[n * 2 + 1]]);
        assert_eq!(at(96), sample(500, 96));
        assert_eq!(at(143), sample(500, 143));
        assert!(build(&[("tone".to_owned(), "5".to_owned())]).is_err());
    }
}
