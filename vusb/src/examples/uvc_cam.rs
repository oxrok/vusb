// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A camera that sees a test pattern: a USB Video Class 1.0 device with
//! one uncompressed format, the shortest model that negotiates a stream
//! before it sends one. On Linux `uvcvideo` makes it a video device, and
//! `v4l2-ctl` or any camera program captures from it.
//!
//! 160 by 120 pixels, YUY2, 25 frames a second: a frame is 38400 bytes,
//! sent as forty packets of three lines, one in every 1 ms frame of the
//! bus. The picture is eight colour bars under a strip that counts the
//! frames in binary, so a capture says which frames it holds and each
//! one is known byte for byte.
//!
//! The model keeps no clock. Each [`DeviceModel::iso_in`] is the next
//! packet, so the frames a host captures are those the model produced,
//! however fast it asked.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{Descriptors, DeviceModel, ISOCHRONOUS, Reply, Setup, Speed, Violations};
use crate::options::parse_number;
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "uvc-cam";
/// One line about the device and its options, for a host that lists its models.
pub const ABOUT: &str = "a USB Video camera that sees a test pattern (160x120 YUY2, 25 fps, \
     isochronous IN); drop=N (every Nth packet is lost)";

/// The picture's width, in pixels.
pub const WIDTH: usize = 160;
/// The picture's height, in lines.
pub const HEIGHT: usize = 120;
/// Bytes in one line: YUY2 has two bytes a pixel.
pub const LINE: usize = WIDTH * 2;
/// Bytes in one frame.
pub const FRAME: usize = LINE * HEIGHT;
/// The lines at the top that show the frame number.
pub const COUNTER_LINES: usize = 8;
/// The bits of the frame number the strip shows, the highest at the left.
pub const COUNTER_BITS: usize = 16;
/// The isochronous IN endpoint's number.
pub const EP_VIDEO: u8 = 1;
/// The header before the picture's bytes in every packet.
pub const HEADER: usize = 2;
/// The bytes of one packet: its header and three lines.
pub const PACKET: usize = HEADER + LINES_PER_PACKET * LINE;
/// The packets of one frame.
pub const PACKETS_PER_FRAME: usize = HEIGHT / LINES_PER_PACKET;

const LINES_PER_PACKET: usize = 3;
/// The time of one frame in units of 100 ns: forty packets, 40 ms.
const FRAME_INTERVAL: u32 = 400_000;

/// The colour bars, left to right, as (Y, U, V): white, yellow, cyan,
/// green, magenta, red, blue and black at 75 %, in BT.601.
const BARS: [(u8, u8, u8); 8] = [
    (180, 128, 128),
    (162, 44, 142),
    (131, 156, 44),
    (112, 72, 58),
    (84, 184, 198),
    (65, 100, 212),
    (35, 212, 114),
    (16, 128, 128),
];
/// A one in the counter strip.
const WHITE: (u8, u8, u8) = (235, 128, 128);
/// A zero in the counter strip.
const BLACK: (u8, u8, u8) = (16, 128, 128);

const VIDEO: u8 = 0x0e;
const VIDEO_CONTROL: u8 = 1;
const VIDEO_STREAMING: u8 = 2;
const VIDEO_COLLECTION: u8 = 3;
const INTERFACE_ASSOCIATION: u8 = 0x0b;
const CS_INTERFACE: u8 = 0x24;
/// The streaming interface's number.
const STREAMING: u16 = 1;

// The requests of a video control, and the two controls of the
// streaming interface.
const SET_CUR: u8 = 0x01;
const GET_CUR: u8 = 0x81;
const GET_MIN: u8 = 0x82;
const GET_MAX: u8 = 0x83;
const GET_LEN: u8 = 0x85;
const GET_INFO: u8 = 0x86;
const GET_DEF: u8 = 0x87;
const PROBE: u8 = 1;
const COMMIT: u8 = 2;
/// The length of the probe and commit controls in UVC 1.0.
const SETTING: usize = 26;

// The bits of a payload header's second byte.
const FRAME_ID: u8 = 0x01;
const END_OF_FRAME: u8 = 0x02;
const END_OF_HEADER: u8 = 0x80;

/// The device.
#[derive(Clone, Debug)]
pub struct UvcCam {
    descriptors: Descriptors,
    /// Every this many packets one is lost.
    drop: Option<u32>,
    /// Whether the host committed the format since the last reset.
    committed: bool,
    /// The frame being sent, counted from power on.
    frame: u32,
    /// The packet of that frame the host gets next.
    packet: usize,
    /// Packets since power on, the lost ones included.
    packets: u32,
}

impl UvcCam {
    /// A camera that loses no packet.
    pub fn new() -> UvcCam {
        UvcCam {
            descriptors: descriptors(),
            drop: None,
            committed: false,
            frame: 0,
            packet: 0,
            packets: 0,
        }
    }

    /// A stream starts at the top of a frame: the one a stopped stream
    /// left unfinished is never completed.
    fn commit(&mut self) {
        self.committed = true;
        if self.packet != 0 {
            self.packet = 0;
            self.frame += 1;
        }
    }
}

impl Default for UvcCam {
    fn default() -> UvcCam {
        UvcCam::new()
    }
}

/// Line `row` of frame number `frame`, as the camera sends it: two
/// pixels in four bytes, Y U Y V.
pub fn line(frame: u32, row: usize) -> [u8; LINE] {
    let mut bytes = [0; LINE];
    for (pair, four) in bytes.as_chunks_mut::<4>().0.iter_mut().enumerate() {
        let x = pair * 2;
        let (y, u, v) = if row < COUNTER_LINES {
            let bit = COUNTER_BITS - 1 - x * COUNTER_BITS / WIDTH;
            if frame >> bit & 1 == 1 { WHITE } else { BLACK }
        } else {
            BARS[x * BARS.len() / WIDTH]
        };
        *four = [y, u, y, v];
    }
    bytes
}

/// The picture of frame number `frame`.
pub fn frame(frame: u32) -> Vec<u8> {
    (0..HEIGHT).flat_map(|row| line(frame, row)).collect()
}

/// What the probe and commit controls hold: the one format, frame size
/// and frame rate the camera has, and the sizes that follow from them.
fn setting() -> [u8; SETTING] {
    let mut s = [0; SETTING];
    // bFormatIndex and bFrameIndex.
    s[2] = 1;
    s[3] = 1;
    s[4..8].copy_from_slice(&FRAME_INTERVAL.to_le_bytes());
    // dwMaxVideoFrameSize and dwMaxPayloadTransferSize.
    s[18..22].copy_from_slice(&(FRAME as u32).to_le_bytes());
    s[22..26].copy_from_slice(&(PACKET as u32).to_le_bytes());
    s
}

/// The descriptors: a video control interface with a camera terminal
/// wired to a USB streaming terminal, and a video streaming interface
/// with one format and one frame size, whose alternate setting 1 has
/// the endpoint.
fn descriptors() -> Descriptors {
    let mut body = Vec::new();
    // The two interfaces are one function.
    body.extend_from_slice(&[8, INTERFACE_ASSOCIATION, 0, 2, VIDEO, VIDEO_COLLECTION, 0, 2]);
    // Video control: header (UVC 1.0, 39 bytes with its units, a 1 MHz
    // clock, one streaming interface), a camera terminal with no
    // controls, an output terminal (USB streaming) fed by it.
    body.extend_from_slice(&descriptor::interface(0, 0, 0, [VIDEO, VIDEO_CONTROL, 0], 2));
    body.extend_from_slice(&[13, CS_INTERFACE, 0x01, 0x00, 0x01, 39, 0, 0x40, 0x42, 0x0f, 0, 1, 1]);
    body.extend_from_slice(&[
        17,
        CS_INTERFACE,
        0x02,
        1,
        0x01,
        0x02,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        2,
        0,
        0,
    ]);
    body.extend_from_slice(&[9, CS_INTERFACE, 0x03, 2, 0x01, 0x01, 0, 1, 0]);
    // Video streaming: no endpoint until the host selects alternate 1.
    body.extend_from_slice(&descriptor::interface(1, 0, 0, [VIDEO, VIDEO_STREAMING, 0], 0));
    // Input header: one format, 71 bytes with it and its frame, the
    // endpoint, the terminal it feeds.
    body.extend_from_slice(&[
        14,
        CS_INTERFACE,
        0x01,
        1,
        71,
        0,
        0x80 | EP_VIDEO,
        0,
        2,
        0,
        0,
        0,
        1,
        0,
    ]);
    // Format: uncompressed, one frame size, YUY2 by its GUID, 16 bits a
    // pixel.
    body.extend_from_slice(&[27, CS_INTERFACE, 0x04, 1, 1]);
    body.extend_from_slice(b"YUY2");
    body.extend_from_slice(&[
        0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
    ]);
    body.extend_from_slice(&[16, 1, 0, 0, 0, 0]);
    // Frame: its size, its bit rate, its bytes, and one frame interval.
    let bit_rate = (FRAME as u32 * 8 * 25).to_le_bytes();
    body.extend_from_slice(&[30, CS_INTERFACE, 0x05, 1, 0]);
    body.extend_from_slice(&(WIDTH as u16).to_le_bytes());
    body.extend_from_slice(&(HEIGHT as u16).to_le_bytes());
    body.extend_from_slice(&bit_rate);
    body.extend_from_slice(&bit_rate);
    body.extend_from_slice(&(FRAME as u32).to_le_bytes());
    body.extend_from_slice(&FRAME_INTERVAL.to_le_bytes());
    body.push(1);
    body.extend_from_slice(&FRAME_INTERVAL.to_le_bytes());
    // The endpoint: isochronous, asynchronous, a packet every frame.
    body.extend_from_slice(&descriptor::interface(1, 1, 1, [VIDEO, VIDEO_STREAMING, 0], 0));
    body.extend_from_slice(&descriptor::endpoint(
        0x80 | EP_VIDEO,
        ISOCHRONOUS | 0x04,
        PACKET as u16,
        1,
    ));
    let device = DeviceDescriptor {
        usb: 0x0200,
        // A device whose functions are interface associations.
        class: [0xef, 2, 1],
        max_packet0: 64,
        // pid.codes' test ids.
        vendor: 0x1209,
        product: 0x0004,
        device: 0x0100,
        strings: [1, 2, 0],
        configurations: 1,
    };
    Descriptors {
        speed: Speed::Full,
        device: device.bytes(),
        configs: vec![descriptor::configuration(1, 0, 0x80, 100, &body)],
        strings: vec!["vusb".to_owned(), "Test pattern camera".to_owned()],
    }
}

impl DeviceModel for UvcCam {
    fn name(&self) -> &'static str {
        NAME
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        // The camera has two controls, both of the streaming interface:
        // the host probes a format, then commits it. With one format
        // every question has the same answer.
        let control = (setup.value >> 8) as u8;
        let class_interface = setup.request_type & 0x7f == 0x21;
        if class_interface && setup.index == STREAMING && matches!(control, PROBE | COMMIT) {
            match setup.request {
                GET_CUR | GET_MIN | GET_MAX | GET_DEF => return Reply::Data(setting().to_vec()),
                GET_LEN => return Reply::Data(vec![SETTING as u8, 0]),
                // The control can be read and written.
                GET_INFO => return Reply::Data(vec![0x03]),
                SET_CUR if data.len() != SETTING => {
                    v.report(format!(
                        "a format of {} bytes; the control has {SETTING}",
                        data.len()
                    ));
                    return Reply::Stall;
                }
                // A probe names what the host would like, and the next
                // GET_CUR says what it gets.
                SET_CUR if control == PROBE => return Reply::ack(),
                SET_CUR => {
                    if data[2..4] != [1, 1] {
                        v.report(format!(
                            "commit of format {} frame {}; the camera has format 1 frame 1",
                            data[2], data[3]
                        ));
                        return Reply::Stall;
                    }
                    self.commit();
                    return Reply::ack();
                }
                _ => {}
            }
        }
        v.report(format!(
            "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the camera answers",
            setup.request_type, setup.request, setup.value, setup.index
        ));
        Reply::Stall
    }

    fn out(&mut self, ep: u8, _data: &[u8], v: &mut Violations) -> Reply {
        v.report(format!("OUT transfer to endpoint {ep}; the camera has none"));
        Reply::Stall
    }

    fn in_transfer(&mut self, ep: u8, _max: usize, v: &mut Violations) -> Reply {
        v.report(format!("IN transfer from endpoint {ep}, which is isochronous"));
        Reply::Stall
    }

    fn iso_in(&mut self, ep: u8, max: usize, v: &mut Violations) -> Option<Vec<u8>> {
        if ep != EP_VIDEO {
            v.report(format!("isochronous IN on endpoint {ep}; the video is on {EP_VIDEO}"));
            return None;
        }
        if max < PACKET {
            v.report(format!("a packet of {max} bytes; the camera sends {PACKET}"));
        }
        if !self.committed {
            v.report("a stream before the host committed a format");
            self.committed = true;
        }
        let (frame, packet) = (self.frame, self.packet);
        self.packet += 1;
        if self.packet == PACKETS_PER_FRAME {
            self.packet = 0;
            self.frame += 1;
        }
        self.packets += 1;
        if self.drop.is_some_and(|n| self.packets.is_multiple_of(n)) {
            return None;
        }
        // The header: its length, then the frame's parity, which is how
        // a host tells one frame from the next, and the mark on the
        // frame's last packet.
        let mut info = END_OF_HEADER;
        if frame & 1 == 1 {
            info |= FRAME_ID;
        }
        if packet + 1 == PACKETS_PER_FRAME {
            info |= END_OF_FRAME;
        }
        let mut bytes = Vec::with_capacity(PACKET);
        bytes.extend_from_slice(&[HEADER as u8, info]);
        let first = packet * LINES_PER_PACKET;
        for row in first..first + LINES_PER_PACKET {
            bytes.extend_from_slice(&line(frame, row));
        }
        Some(bytes)
    }

    fn reset(&mut self) {
        self.committed = false;
        self.frame = 0;
        self.packet = 0;
        self.packets = 0;
    }
}

/// The device, from `drop=N`.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    let mut cam = UvcCam::new();
    for (k, v) in options {
        match k.as_str() {
            "drop" => cam.drop = Some(parse_number(k, v, 2..=1_000_000)?),
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    Ok(Box::new(cam))
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::bus::{Bus, Completion, Status};
    use crate::device::Device;
    use crate::host::enumerate;
    use crate::observer::{Recorder, SharedObserver};

    fn camera(options: &[(&str, &str)]) -> (Bus, Arc<Mutex<Recorder>>) {
        let options: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        let rec = Arc::new(Mutex::new(Recorder::default()));
        let shared: SharedObserver = rec.clone();
        (Bus::new(Device::new(build(&options).unwrap())).with_observer(shared), rec)
    }

    fn get(bus: &mut Bus, request: u8, control: u8, length: u16) -> Completion {
        let value = u16::from(control) << 8;
        let setup = Setup { request_type: 0xa1, request, value, index: STREAMING, length };
        bus.control(setup, &[])
    }

    fn set(bus: &mut Bus, control: u8, data: &[u8]) -> Completion {
        let setup = Setup {
            request_type: 0x21,
            request: SET_CUR,
            value: u16::from(control) << 8,
            index: STREAMING,
            length: data.len() as u16,
        };
        bus.control(setup, data)
    }

    /// What a host does before it streams: asks for the default format,
    /// probes it, reads what it got, commits that, and selects the
    /// alternate setting with the endpoint.
    fn negotiate(bus: &mut Bus) {
        let default = get(bus, GET_DEF, PROBE, 26);
        assert!(default.is_ok());
        assert!(set(bus, PROBE, &default.data).is_ok());
        let got = get(bus, GET_CUR, PROBE, 26);
        assert!(set(bus, COMMIT, &got.data).is_ok());
        let select = Setup { request_type: 0x01, request: 11, value: 1, index: 1, length: 0 };
        assert!(bus.control(select, &[]).is_ok());
    }

    /// The pictures in a stream of packets, split where a header says
    /// a frame ends, each with the frame parity its packets carried.
    fn pictures(c: &Completion) -> Vec<(u8, Vec<u8>)> {
        let mut pictures = Vec::new();
        let mut picture = Vec::new();
        let mut at = 0;
        for packet in &c.iso {
            let bytes = &c.data[at..at + packet.actual];
            at += packet.actual;
            let [length, info, payload @ ..] = bytes else { continue };
            assert_eq!((usize::from(*length), info & END_OF_HEADER), (HEADER, END_OF_HEADER));
            picture.extend_from_slice(payload);
            if info & END_OF_FRAME != 0 {
                pictures.push((info & FRAME_ID, std::mem::take(&mut picture)));
            }
        }
        pictures
    }

    #[test]
    fn a_host_finds_a_camera_with_one_format() {
        let (mut bus, rec) = camera(&[]);
        let e = enumerate(&mut bus).unwrap();
        assert_eq!(e.product, "Test pattern camera");
        let eps = e.endpoints();
        assert_eq!(eps.len(), 1);
        assert_eq!(
            (eps[0].address, eps[0].transfer_type(), usize::from(eps[0].max_packet)),
            (0x81, ISOCHRONOUS, PACKET)
        );
        // The class descriptors say their own total lengths: what follows
        // the video control header up to the streaming interface, and
        // what follows the input header up to the endpoint's interface.
        let config = &bus.device().descriptors().configs[0];
        let records: Vec<&[u8]> = descriptor::records(config).collect();
        let class: Vec<&[u8]> = records.iter().copied().filter(|r| r[1] == CS_INTERFACE).collect();
        let total = |r: &[u8], at: usize| usize::from(u16::from_le_bytes([r[at], r[at + 1]]));
        assert_eq!(total(class[0], 5), class[..3].iter().map(|r| r.len()).sum::<usize>());
        assert_eq!(total(class[3], 4), class[3..].iter().map(|r| r.len()).sum::<usize>());
        assert_eq!(class.len(), 6);
        // The controls: 26 bytes, readable and writable, one answer.
        assert_eq!(get(&mut bus, GET_LEN, PROBE, 2).data, [26, 0]);
        assert_eq!(get(&mut bus, GET_INFO, COMMIT, 1).data, [3]);
        let s = get(&mut bus, GET_MAX, PROBE, 26).data;
        assert_eq!(s, get(&mut bus, GET_MIN, PROBE, 26).data);
        assert_eq!((s[2], s[3]), (1, 1));
        assert_eq!(u32::from_le_bytes([s[4], s[5], s[6], s[7]]), 400_000, "25 frames a second");
        assert_eq!(u32::from_le_bytes([s[18], s[19], s[20], s[21]]), 38_400);
        assert_eq!(u32::from_le_bytes([s[22], s[23], s[24], s[25]]), 962);
        let rec = rec.lock().unwrap();
        assert!(rec.violations.is_empty(), "{:?}", rec.violations);
    }

    #[test]
    fn streams_the_pattern_frame_by_frame() {
        let (mut bus, rec) = camera(&[]);
        enumerate(&mut bus).unwrap();
        negotiate(&mut bus);
        // Three frames and one packet more.
        let c = bus.iso_read(EP_VIDEO, &[PACKET; 3 * PACKETS_PER_FRAME + 1]);
        assert_eq!(c.status, Status::Ok);
        assert_eq!(c.actual, (3 * PACKETS_PER_FRAME + 1) * PACKET);
        let got = pictures(&c);
        assert_eq!(got.len(), 3);
        for (n, (parity, picture)) in got.iter().enumerate() {
            assert_eq!(u32::from(*parity), n as u32 & 1, "the parity turns with every frame");
            assert!(*picture == frame(n as u32), "frame {n}");
        }
        // The pattern: bars below the strip, the same in every frame;
        // the strip is the frame number, its lowest bit at the right.
        let pixel = |f: &[u8], x: usize, y: usize| {
            let at = y * LINE + x / 2 * 4;
            (f[at + x % 2 * 2], f[at + 1], f[at + 3])
        };
        let (zero, five) = (frame(0), frame(5));
        assert_eq!(zero.len(), FRAME);
        assert_eq!(zero[COUNTER_LINES * LINE..], five[COUNTER_LINES * LINE..]);
        assert_eq!(pixel(&zero, 0, 8), (180, 128, 128), "white");
        assert_eq!(pixel(&zero, 19, 119), (180, 128, 128));
        assert_eq!(pixel(&zero, 20, 8), (162, 44, 142), "yellow");
        assert_eq!(pixel(&zero, 119, 60), (65, 100, 212), "red");
        assert_eq!(pixel(&zero, 159, 119), (16, 128, 128), "black");
        let bits = |f: &[u8], y: usize| -> Vec<u8> {
            (0..COUNTER_BITS).map(|b| u8::from(pixel(f, b * 10 + 5, y) == WHITE)).collect()
        };
        assert_eq!(bits(&zero, 0), [0; 16]);
        assert_eq!(bits(&five, 0), [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1]);
        assert_eq!(bits(&five, 7), bits(&five, 0));
        assert_eq!(bits(&frame(0x8001), 3)[..2], [1, 0]);
        let rec = rec.lock().unwrap();
        assert!(rec.violations.is_empty(), "{:?}", rec.violations);
    }

    #[test]
    fn a_new_stream_starts_at_the_top_of_the_next_frame() {
        let (mut bus, rec) = camera(&[]);
        enumerate(&mut bus).unwrap();
        negotiate(&mut bus);
        // The host stops ten packets into frame 0, and starts again.
        bus.iso_read(EP_VIDEO, &[PACKET; 10]);
        negotiate(&mut bus);
        let c = bus.iso_read(EP_VIDEO, &[PACKET; PACKETS_PER_FRAME]);
        let got = pictures(&c);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].0, 1);
        assert!(got[0].1 == frame(1), "frame 1 from its top");
        let rec = rec.lock().unwrap();
        assert!(rec.violations.is_empty(), "{:?}", rec.violations);
    }

    #[test]
    fn a_lost_packet_is_missed_and_its_lines_with_it() {
        let (mut bus, _) = camera(&[("drop", "7")]);
        enumerate(&mut bus).unwrap();
        negotiate(&mut bus);
        let c = bus.iso_read(EP_VIDEO, &[PACKET; PACKETS_PER_FRAME]);
        let missed: Vec<usize> =
            (0..PACKETS_PER_FRAME).filter(|&n| c.iso[n].status == Status::Missed).collect();
        assert_eq!(missed, [6, 13, 20, 27, 34]);
        // The frame still ends where it should, five packets short: the
        // lines of a lost packet are not sent later.
        let got = pictures(&c);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].1.len(), FRAME - 5 * (PACKET - HEADER));
        assert_eq!(got[0].1[..6 * (PACKET - HEADER)], frame(0)[..6 * (PACKET - HEADER)]);
        assert!(build(&[("drop".to_owned(), "1".to_owned())]).is_err());
    }

    #[test]
    fn what_a_host_may_not_do() {
        let (mut bus, rec) = camera(&[]);
        enumerate(&mut bus).unwrap();
        let violations = |rec: &Arc<Mutex<Recorder>>| rec.lock().unwrap().violations.len();
        // A stream nobody negotiated: said once, and the stream goes on.
        let c = bus.iso_read(EP_VIDEO, &[PACKET; 2]);
        assert_eq!(c.actual, 2 * PACKET);
        assert_eq!(violations(&rec), 1);
        // A commit of a format the camera lacks, a format of the wrong
        // length, and a control it does not have.
        let mut other = setting();
        other[2] = 2;
        assert_eq!(set(&mut bus, COMMIT, &other).status, Status::Stall);
        assert_eq!(set(&mut bus, PROBE, &other[..20]).status, Status::Stall);
        assert_eq!(get(&mut bus, GET_CUR, 3, 26).status, Status::Stall);
        assert_eq!(violations(&rec), 4);
        // A probe of it is no fault: the answer is the format it has.
        assert!(set(&mut bus, PROBE, &other).is_ok());
        assert_eq!(get(&mut bus, GET_CUR, PROBE, 26).data, setting());
        assert_eq!(violations(&rec), 4);
    }
}
