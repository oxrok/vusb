// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The mass storage shell: a disk the kernel's `usb-storage` binds,
//! which the machine sees as `/dev/sdX`.
//!
//! A model implements [`Medium`], blocks to read and write, and never
//! sees a URB. Two come with the shell: [`Memory`], a disk in memory,
//! and [`Image`], a disk that is a file. [`Msc`] is the [`DeviceModel`]
//! around one: the descriptors (one interface of class 8, SCSI
//! transparent, bulk only, with a bulk IN and a bulk OUT endpoint), the
//! two class requests, the Bulk-Only Transport and the SCSI commands a
//! host's disk driver sends.
//!
//! ```
//! use vusb::shells::msc::{Faults, Identity, Memory, Msc};
//!
//! // 64 blocks of 512 bytes, with something in the first.
//! let mut disk = vec![0; 64 * 512];
//! disk[..5].copy_from_slice(b"hello");
//! let id = Identity::new(0x1234, 0x5678, "Maker", "Stick");
//! let model = Msc::new("stick", &id, Faults::default(), Memory::from_bytes(disk));
//!
//! // In a test: the bus is the host. A command is three transfers: the
//! // command block wrapper, the data, the status.
//! use vusb::bus::Bus;
//! use vusb::device::Device;
//! use vusb::host::enumerate;
//! use vusb::shells::msc::{EP_IN, EP_OUT};
//! let mut bus = Bus::new(Device::new(Box::new(model)));
//! enumerate(&mut bus).unwrap();
//!
//! // READ(10) of block 0, 512 bytes to the host, tag 7.
//! let mut cbw = Vec::new();
//! cbw.extend_from_slice(b"USBC");
//! cbw.extend_from_slice(&7u32.to_le_bytes());
//! cbw.extend_from_slice(&512u32.to_le_bytes());
//! cbw.extend_from_slice(&[0x80, 0, 10]);
//! cbw.extend_from_slice(&[0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]);
//! bus.write(EP_OUT, &cbw);
//! assert_eq!(&bus.read(EP_IN, 512).data[..5], b"hello");
//! let status = bus.read(EP_IN, 13).data;
//! assert_eq!(&status[..4], b"USBS");
//! assert_eq!(status[12], 0, "passed");
//! ```
//!
//! # The transport
//!
//! A command is a command block wrapper (31 bytes to the bulk OUT
//! endpoint), a data phase in the direction and of the length the
//! wrapper names, and a status wrapper (13 bytes from the bulk IN
//! endpoint). The shell keeps to what the wrapper says, whatever the
//! command turns out to need:
//!
//! - The data phase is always as long as the host said. A command that
//!   has less to send fills the rest with zeros, and one that fails
//!   takes and drops what the host sends; the status wrapper's residue
//!   says how much was not real. The specification allows a STALL
//!   there instead; filling needs no halt to be cleared, so a host sees
//!   the same thing over every transport.
//! - A host that asks for less than the command has, or for the wrong
//!   direction, gets a phase error (status 2) and a violation.
//! - A wrapper that is not one (not 31 bytes, not the signature) STALLs
//!   both endpoints until the host's reset recovery, the class request
//!   `0xff`, as the specification has it.
//!
//! How the host cuts the data into transfers means nothing: a write may
//! arrive in one transfer or packet by packet, and a read is handed out
//! in whatever lengths the host asks for.
//!
//! # The commands
//!
//! `TEST UNIT READY`, `REQUEST SENSE`, `INQUIRY` (with the vital product
//! data pages 0x00, 0x80 and 0x83), `MODE SENSE` (6 and 10, the caching
//! page), `START STOP UNIT`, `PREVENT ALLOW MEDIUM REMOVAL`, `READ FORMAT
//! CAPACITIES`, `READ CAPACITY` (10 and 16), `READ` and `WRITE` (6, 10,
//! 12 and 16), `VERIFY` (10), `SYNCHRONIZE CACHE` (10 and 16) and
//! `REPORT LUNS`. Any other fails with ILLEGAL REQUEST, invalid command
//! operation code, which is no violation: a host may ask.
//!
//! Every model takes the [`Faults`] options, all off by default.

use std::fs::{File, OpenOptions};
use std::io::{self, Read as _, Seek as _, SeekFrom, Write as _};
use std::path::Path;

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{
    BULK, CLASS, Descriptors, DeviceModel, RECIPIENT_INTERFACE, Reply, Setup, Speed, Violations,
};
use crate::{Error, Result};

/// The bulk IN endpoint's number (`0x81`): data to the host and the
/// status wrappers, which an armed event's `in=1:N` counts
/// ([`crate::arm`]).
pub const EP_IN: u8 = 1;
/// The bulk OUT endpoint's number (`0x02`): the command wrappers and
/// data from the host, which `out=2:N` counts.
pub const EP_OUT: u8 = 2;

/// The block size of [`Memory`] and [`Image`], and a [`Medium`]'s
/// unless it says otherwise.
pub const BLOCK: u32 = 512;

const INTERFACE: u8 = 0;

const GET_MAX_LUN: u8 = 0xfe;
const BULK_ONLY_RESET: u8 = 0xff;

const CBW_SIGNATURE: [u8; 4] = *b"USBC";
const CSW_SIGNATURE: [u8; 4] = *b"USBS";
const CBW_LENGTH: usize = 31;

const PASSED: u8 = 0;
const FAILED: u8 = 1;
const PHASE_ERROR: u8 = 2;

const TEST_UNIT_READY: u8 = 0x00;
const REQUEST_SENSE: u8 = 0x03;
const READ_6: u8 = 0x08;
const WRITE_6: u8 = 0x0a;
const INQUIRY: u8 = 0x12;
const MODE_SENSE_6: u8 = 0x1a;
const START_STOP_UNIT: u8 = 0x1b;
const PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1e;
const READ_FORMAT_CAPACITIES: u8 = 0x23;
const READ_CAPACITY_10: u8 = 0x25;
const READ_10: u8 = 0x28;
const WRITE_10: u8 = 0x2a;
const VERIFY_10: u8 = 0x2f;
const SYNCHRONIZE_CACHE_10: u8 = 0x35;
const MODE_SENSE_10: u8 = 0x5a;
const READ_16: u8 = 0x88;
const WRITE_16: u8 = 0x8a;
const SYNCHRONIZE_CACHE_16: u8 = 0x91;
const SERVICE_ACTION_IN_16: u8 = 0x9e;
const REPORT_LUNS: u8 = 0xa0;
const READ_12: u8 = 0xa8;
const WRITE_12: u8 = 0xaa;

/// The service action of `SERVICE ACTION IN (16)` that is `READ
/// CAPACITY (16)`.
const READ_CAPACITY_16: u8 = 0x10;

const PAGE_CACHING: u8 = 0x08;
const PAGE_ALL: u8 = 0x3f;

/// Sense data: why the last command failed, as `REQUEST SENSE` tells
/// it. The sense key, the additional sense code and its qualifier.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sense {
    /// The sense key: 2 not ready, 3 medium error, 5 illegal request,
    /// 7 data protect.
    pub key: u8,
    /// The additional sense code.
    pub code: u8,
    /// The additional sense code qualifier.
    pub qualifier: u8,
}

impl Sense {
    /// Nothing to report.
    pub const NONE: Sense = Sense::new(0, 0x00, 0x00);
    /// The unit is becoming ready.
    pub const BECOMING_READY: Sense = Sense::new(2, 0x04, 0x01);
    /// A block could not be read.
    pub const READ_ERROR: Sense = Sense::new(3, 0x11, 0x00);
    /// A block could not be written.
    pub const WRITE_ERROR: Sense = Sense::new(3, 0x0c, 0x00);
    /// A command the device does not have.
    pub const INVALID_COMMAND: Sense = Sense::new(5, 0x20, 0x00);
    /// Blocks beyond the end of the medium.
    pub const OUT_OF_RANGE: Sense = Sense::new(5, 0x21, 0x00);
    /// A field of the command block the device cannot use.
    pub const INVALID_FIELD: Sense = Sense::new(5, 0x24, 0x00);
    /// A logical unit the device does not have.
    pub const NO_SUCH_UNIT: Sense = Sense::new(5, 0x25, 0x00);
    /// The medium is write protected.
    pub const WRITE_PROTECTED: Sense = Sense::new(7, 0x27, 0x00);

    const fn new(key: u8, code: u8, qualifier: u8) -> Sense {
        Sense { key, code, qualifier }
    }

    /// The 18 bytes of fixed format sense data.
    fn bytes(self) -> Vec<u8> {
        let mut b = vec![0; 18];
        b[0] = 0x70;
        b[2] = self.key;
        b[7] = 10;
        b[12] = self.code;
        b[13] = self.qualifier;
        b
    }
}

/// What a disk is made of: blocks, read and written whole.
///
/// The shell checks every range against [`Medium::blocks`] first, so
/// `read` and `write` only see blocks the medium has, and `write` is
/// never called on a medium that says it is read only. An error from
/// either fails the command with a medium error, as a bad sector does.
pub trait Medium: Send {
    /// How many blocks there are.
    fn blocks(&self) -> u64;

    /// The size of a block in bytes, [`BLOCK`] unless the medium says
    /// otherwise; a power of two, 512 at least.
    fn block_size(&self) -> u32 {
        BLOCK
    }

    /// Whether the medium is write protected: the host is told so, and
    /// its writes fail.
    fn read_only(&self) -> bool {
        false
    }

    /// Reads whole blocks from block `lba` on: `buffer` is a multiple
    /// of the block size long.
    ///
    /// # Errors
    ///
    /// When the blocks cannot be read.
    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> io::Result<()>;

    /// Writes whole blocks from block `lba` on: `data` is a multiple of
    /// the block size long.
    ///
    /// # Errors
    ///
    /// When the blocks cannot be written.
    fn write(&mut self, lba: u64, data: &[u8]) -> io::Result<()>;

    /// The host asks that what it wrote be safe (`SYNCHRONIZE CACHE`).
    /// By default there is nothing to do.
    ///
    /// # Errors
    ///
    /// When it cannot be made so.
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl<M: Medium + ?Sized> Medium for Box<M> {
    fn blocks(&self) -> u64 {
        (**self).blocks()
    }

    fn block_size(&self) -> u32 {
        (**self).block_size()
    }

    fn read_only(&self) -> bool {
        (**self).read_only()
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> io::Result<()> {
        (**self).read(lba, buffer)
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> io::Result<()> {
        (**self).write(lba, data)
    }

    fn flush(&mut self) -> io::Result<()> {
        (**self).flush()
    }
}

/// A disk in memory, of [`BLOCK`] byte blocks: what a test wants. It is
/// gone with the model, so a session's `reset` starts from a new one.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Memory {
    data: Vec<u8>,
    read_only: bool,
}

impl Memory {
    /// A disk of `blocks` blocks, all zeros.
    pub fn new(blocks: usize) -> Memory {
        Memory { data: vec![0; blocks * BLOCK as usize], read_only: false }
    }

    /// A disk that holds `data`, filled with zeros up to a whole block.
    pub fn from_bytes(mut data: Vec<u8>) -> Memory {
        let whole = data.len().div_ceil(BLOCK as usize) * BLOCK as usize;
        data.resize(whole, 0);
        Memory { data, read_only: false }
    }

    /// The same disk, write protected.
    #[must_use]
    pub fn read_only(mut self) -> Memory {
        self.read_only = true;
        self
    }

    /// What the disk holds.
    pub fn bytes(&self) -> &[u8] {
        &self.data
    }

    fn range(&self, lba: u64, length: usize) -> io::Result<std::ops::Range<usize>> {
        let start = usize::try_from(lba).ok().and_then(|l| l.checked_mul(BLOCK as usize));
        start
            .and_then(|s| Some(s..s.checked_add(length)?))
            .filter(|r| r.end <= self.data.len())
            .ok_or_else(|| io::Error::new(io::ErrorKind::UnexpectedEof, "beyond the disk"))
    }
}

impl Medium for Memory {
    fn blocks(&self) -> u64 {
        (self.data.len() / BLOCK as usize) as u64
    }

    fn read_only(&self) -> bool {
        self.read_only
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> io::Result<()> {
        let range = self.range(lba, buffer.len())?;
        buffer.copy_from_slice(&self.data[range]);
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> io::Result<()> {
        let range = self.range(lba, data.len())?;
        self.data[range].copy_from_slice(data);
        Ok(())
    }
}

/// A disk that is a file, of [`BLOCK`] byte blocks: a disk image made
/// beforehand, with `truncate -s 64M disk.img` or `mkfs`, or taken from
/// a real disk. What the host writes is in the file, so it outlives the
/// model, and a session's `reset` finds it as the host left it.
///
/// The file keeps its size: it is the disk's. Bytes after the last
/// whole block are not part of it. Reads and writes go to the file as
/// they come, which is quick enough that the model still never waits;
/// `SYNCHRONIZE CACHE` is an `fsync`.
#[derive(Debug)]
pub struct Image {
    file: File,
    blocks: u64,
    read_only: bool,
}

impl Image {
    /// Opens a disk image to read and write.
    ///
    /// # Errors
    ///
    /// When the file cannot be opened so, or holds no whole block.
    pub fn open(path: impl AsRef<Path>) -> Result<Image> {
        Image::with(path.as_ref(), false)
    }

    /// Opens a disk image write protected: the file is opened to read
    /// only, and the host is told the medium is.
    ///
    /// # Errors
    ///
    /// When the file cannot be opened, or holds no whole block.
    pub fn open_read_only(path: impl AsRef<Path>) -> Result<Image> {
        Image::with(path.as_ref(), true)
    }

    fn with(path: &Path, read_only: bool) -> Result<Image> {
        let named =
            |e: io::Error| Error::Io(io::Error::new(e.kind(), format!("{}: {e}", path.display())));
        let file = OpenOptions::new().read(true).write(!read_only).open(path).map_err(named)?;
        let blocks = file.metadata().map_err(named)?.len() / u64::from(BLOCK);
        if blocks == 0 {
            return Err(Error::Invalid(format!(
                "{}: a disk image holds one block of {BLOCK} bytes at least",
                path.display()
            )));
        }
        Ok(Image { file, blocks, read_only })
    }
}

impl Medium for Image {
    fn blocks(&self) -> u64 {
        self.blocks
    }

    fn read_only(&self) -> bool {
        self.read_only
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> io::Result<()> {
        self.file.seek(SeekFrom::Start(lba * u64::from(BLOCK)))?;
        self.file.read_exact(buffer)
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> io::Result<()> {
        self.file.seek(SeekFrom::Start(lba * u64::from(BLOCK)))?;
        self.file.write_all(data)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.file.sync_data()
    }
}

/// The ids and strings of a disk; the rest of the descriptors is the
/// shell's.
#[derive(Clone, Debug)]
pub struct Identity {
    /// `idVendor`.
    pub vendor: u16,
    /// `idProduct`.
    pub product: u16,
    /// `iManufacturer`'s string; its first 8 characters are the vendor
    /// of `INQUIRY`.
    pub manufacturer: String,
    /// `iProduct`'s string; its first 16 characters are the product of
    /// `INQUIRY`.
    pub name: String,
    /// `iSerialNumber`'s string. The class asks for 12 hexadecimal
    /// digits at least, which the default is.
    pub serial: String,
    /// High speed by default, with 512 byte bulk endpoints, as disks
    /// are; full speed has 64.
    pub speed: Speed,
    /// Whether `INQUIRY` says the medium is removable, as a stick does.
    /// Not by default, as a disk in an enclosure: Linux asks a
    /// removable disk whether it is still ready every two seconds,
    /// which a log made to be compared is better without.
    pub removable: bool,
}

impl Identity {
    /// A high speed disk with serial number `000000000001`, not
    /// removable.
    pub fn new(vendor: u16, product: u16, manufacturer: &str, name: &str) -> Identity {
        Identity {
            vendor,
            product,
            manufacturer: manufacturer.to_owned(),
            name: name.to_owned(),
            serial: "000000000001".to_owned(),
            speed: Speed::High,
            removable: false,
        }
    }
}

/// The descriptors of a disk.
pub fn descriptors(id: &Identity) -> Descriptors {
    let packet = if id.speed == Speed::High { 512 } else { 64 };
    let mut body = Vec::new();
    // SCSI transparent command set, bulk only transport.
    body.extend_from_slice(&descriptor::interface(INTERFACE, 0, 2, [0x08, 0x06, 0x50], 0));
    body.extend_from_slice(&descriptor::endpoint(0x80 | EP_IN, BULK, packet, 0));
    body.extend_from_slice(&descriptor::endpoint(EP_OUT, BULK, packet, 0));
    let config = descriptor::configuration(1, 0, 0x80, 200, &body);
    let device = DeviceDescriptor {
        usb: 0x0200,
        class: [0, 0, 0],
        max_packet0: 64,
        vendor: id.vendor,
        product: id.product,
        device: 0x0100,
        strings: [1, 2, 3],
        configurations: 1,
    };
    Descriptors {
        speed: id.speed,
        device: device.bytes(),
        configs: vec![config],
        strings: vec![id.manufacturer.clone(), id.name.clone(), id.serial.clone()],
    }
}

/// What every model on this shell takes as options, all off by default;
/// a model's `build` hands each option to [`Faults::option`] first.
///
/// - `read-error=LBA` a read that touches block LBA fails with a medium
///   error, unrecovered read error, as a bad sector does (repeatable).
///   The blocks before it in the same command are read.
/// - `write-error=LBA` a write that touches block LBA fails with a
///   medium error, write error (repeatable). The blocks before it in
///   the same command are written.
/// - `not-ready=N` the first N `TEST UNIT READY` of a power on answer
///   NOT READY, becoming ready, as a disk that spins up, and so does
///   whatever needs the medium meanwhile. Linux asks again every
///   second.
#[derive(Clone, Debug, Default)]
pub struct Faults {
    read_errors: Vec<u64>,
    write_errors: Vec<u64>,
    not_ready: u32,
}

impl Faults {
    /// Takes an option of the shell: whether it was one.
    ///
    /// # Errors
    ///
    /// On a value it cannot use.
    pub fn option(&mut self, key: &str, value: &str) -> Result<bool> {
        let number = || crate::options::parse_number(key, value, 0..=u32::MAX);
        match key {
            "read-error" => self.read_errors.push(u64::from(number()?)),
            "write-error" => self.write_errors.push(u64::from(number()?)),
            "not-ready" => self.not_ready = number()?,
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// The first of `bad` among the `count` blocks from `lba` on.
fn first_bad(bad: &[u64], lba: u64, count: u64) -> Option<u64> {
    bad.iter().copied().filter(|b| (lba..lba + count).contains(b)).min()
}

/// A command block wrapper, parsed.
#[derive(Clone, Copy, Debug)]
struct Wrapper {
    tag: u32,
    /// `dCBWDataTransferLength`: what the host means to move.
    length: u32,
    /// The data phase is to the host.
    to_host: bool,
    lun: u8,
    block: [u8; 16],
    block_length: usize,
}

/// What a command comes to, before the transport has its say.
#[derive(Debug)]
enum Outcome {
    /// Done, with nothing to move.
    Good,
    /// These bytes for the host.
    Data(Vec<u8>),
    /// These blocks for the host.
    Read { lba: u64, count: u64 },
    /// These blocks from the host.
    Write { lba: u64, count: u64 },
    /// Failed, with this to say for `REQUEST SENSE`.
    Failed(Sense),
}

/// Blocks being read for the host.
#[derive(Clone, Copy, Debug)]
struct Reading {
    next: u64,
    left: u64,
}

/// Blocks being taken from the host.
#[derive(Debug)]
struct Writing {
    next: u64,
    /// Bytes of the command's blocks not yet taken.
    left: u64,
    /// The bytes of a block that is not whole yet.
    partial: Vec<u8>,
}

/// Where a command stands.
#[derive(Debug, Default)]
enum Phase {
    /// Waiting for a command block wrapper.
    #[default]
    Command,
    /// The data phase to the host.
    DataIn {
        tag: u32,
        /// Read and not yet sent.
        pending: Vec<u8>,
        reading: Option<Reading>,
        /// Bytes of the host's length still to send, the fill included.
        left: u64,
        /// Bytes of it that were not the command's.
        residue: u64,
        status: u8,
    },
    /// The data phase from the host.
    DataOut {
        tag: u32,
        writing: Option<Writing>,
        /// Bytes of the host's length still to take.
        left: u64,
        residue: u64,
        status: u8,
    },
    /// The status wrapper waits to be read.
    Status([u8; 13]),
    /// A wrapper that was not one: both endpoints STALL until the reset
    /// recovery.
    Broken,
}

fn status_wrapper(tag: u32, residue: u64, status: u8) -> [u8; 13] {
    let mut b = [0; 13];
    b[..4].copy_from_slice(&CSW_SIGNATURE);
    b[4..8].copy_from_slice(&tag.to_le_bytes());
    b[8..12].copy_from_slice(&u32::try_from(residue).unwrap_or(u32::MAX).to_le_bytes());
    b[12] = status;
    b
}

/// `text` as the fixed width ASCII field of `INQUIRY` data, filled with
/// spaces.
fn field<const N: usize>(text: &str) -> [u8; N] {
    let mut b = [b' '; N];
    let printable = text.bytes().filter(|c| c.is_ascii_graphic() || *c == b' ');
    for (slot, c) in b.iter_mut().zip(printable) {
        *slot = c;
    }
    b
}

fn be16(b: &[u8]) -> u64 {
    u64::from(u16::from_be_bytes([b[0], b[1]]))
}

fn be32(b: &[u8]) -> u64 {
    u64::from(u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
}

/// A mass storage function around a [`Medium`].
#[derive(Debug)]
pub struct Msc<M> {
    name: &'static str,
    descriptors: Descriptors,
    vendor: [u8; 8],
    product: [u8; 16],
    serial: String,
    removable: bool,
    faults: Faults,
    phase: Phase,
    sense: Sense,
    /// `TEST UNIT READY` commands still to fail, see [`Faults`].
    not_ready: u32,
    /// The medium.
    pub medium: M,
}

impl<M: Medium> Msc<M> {
    /// The function `name` (the model's `NAME`).
    pub fn new(name: &'static str, id: &Identity, faults: Faults, medium: M) -> Msc<M> {
        Msc {
            name,
            descriptors: descriptors(id),
            vendor: field(&id.manufacturer),
            product: field(&id.name),
            serial: id.serial.clone(),
            removable: id.removable,
            not_ready: faults.not_ready,
            faults,
            phase: Phase::Command,
            sense: Sense::NONE,
            medium,
        }
    }

    fn block_size(&self) -> u64 {
        u64::from(self.medium.block_size().max(1))
    }

    /// Parses a command block wrapper; what is wrong with one that is
    /// not valid.
    fn wrapper(data: &[u8], v: &mut Violations) -> std::result::Result<Wrapper, String> {
        if data.len() != CBW_LENGTH {
            return Err(format!(
                "{} bytes where a command block wrapper is {CBW_LENGTH}",
                data.len()
            ));
        }
        if data[..4] != CBW_SIGNATURE {
            return Err(format!(
                "a command block wrapper with signature {}",
                crate::observer::hex(&data[..4])
            ));
        }
        let flags = data[12];
        if flags & 0x7f != 0 {
            v.report(format!("command block wrapper with reserved flags in {flags:#04x}"));
        }
        if data[13] & 0xf0 != 0 {
            v.report(format!(
                "command block wrapper with reserved bits in the unit {:#04x}",
                data[13]
            ));
        }
        let block_length = usize::from(data[14] & 0x1f);
        if data[14] & 0xe0 != 0 || !(1..=16).contains(&block_length) {
            v.report(format!(
                "command block wrapper with a command block of length {:#04x}; 1 to 16",
                data[14]
            ));
        }
        let mut block = [0; 16];
        block.copy_from_slice(&data[15..31]);
        Ok(Wrapper {
            tag: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
            length: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            to_host: flags & 0x80 != 0,
            lun: data[13] & 0x0f,
            block,
            block_length: block_length.clamp(1, 16),
        })
    }

    fn inquiry(&self, cbw: &Wrapper) -> Outcome {
        let block = &cbw.block;
        let allocation = usize::from(u16::from_be_bytes([block[3], block[4]]));
        let vital = block[1] & 1 != 0;
        let page = block[2];
        // A unit the device lacks still answers: there is nothing there.
        let kind = if cbw.lun == 0 { 0x00 } else { 0x7f };
        let mut data = match (vital, page) {
            (false, 0) => {
                let mut d =
                    vec![kind, if self.removable { 0x80 } else { 0 }, 0x04, 0x02, 31, 0, 0, 0];
                d.extend_from_slice(&self.vendor);
                d.extend_from_slice(&self.product);
                d.extend_from_slice(b"1.00");
                d
            }
            (true, 0x00) => vec![kind, 0x00, 0, 3, 0x00, 0x80, 0x83],
            (true, 0x80) => {
                let serial = self.serial.as_bytes();
                let mut d = vec![kind, 0x80, 0, serial.len().min(255) as u8];
                d.extend_from_slice(&serial[..serial.len().min(255)]);
                d
            }
            (true, 0x83) => {
                // One designator: the T10 vendor identification, ASCII.
                let mut id = self.vendor.to_vec();
                id.extend_from_slice(&self.product);
                id.extend_from_slice(self.serial.as_bytes());
                id.truncate(248);
                let mut d =
                    vec![kind, 0x83, 0, (id.len() + 4) as u8, 0x02, 0x01, 0, id.len() as u8];
                d.extend_from_slice(&id);
                d
            }
            _ => return Outcome::Failed(Sense::INVALID_FIELD),
        };
        data.truncate(allocation);
        Outcome::Data(data)
    }

    /// The mode pages asked for, behind `header` bytes still to fill.
    fn mode_pages(block: &[u8; 16], header: usize) -> Option<Vec<u8>> {
        let mut data = vec![0; header];
        match block[2] & 0x3f {
            PAGE_CACHING | PAGE_ALL => {
                // The caching page: no write cache, so nothing is lost
                // with the cable. Nothing in it may be changed, so the
                // current, changeable and default values are all zeros.
                let mut caching = [0; 20];
                caching[0] = PAGE_CACHING;
                caching[1] = 18;
                data.extend_from_slice(&caching);
            }
            _ => return None,
        }
        Some(data)
    }

    fn mode_sense(&self, block: &[u8; 16]) -> Outcome {
        let ten = block[0] == MODE_SENSE_10;
        let protect = if self.medium.read_only() { 0x80 } else { 0 };
        let Some(mut data) = Self::mode_pages(block, if ten { 8 } else { 4 }) else {
            return Outcome::Failed(Sense::INVALID_FIELD);
        };
        let allocation = if ten {
            let length = (data.len() - 2) as u16;
            data[..2].copy_from_slice(&length.to_be_bytes());
            data[3] = protect;
            usize::from(u16::from_be_bytes([block[7], block[8]]))
        } else {
            data[0] = (data.len() - 1) as u8;
            data[2] = protect;
            usize::from(block[4])
        };
        data.truncate(allocation);
        Outcome::Data(data)
    }

    /// The blocks a read or write names: the first and how many.
    fn extent(block: &[u8; 16]) -> (u64, u64) {
        match block[0] {
            READ_6 | WRITE_6 => {
                let lba = u64::from(block[1] & 0x1f) << 16 | be16(&block[2..4]);
                (lba, if block[4] == 0 { 256 } else { u64::from(block[4]) })
            }
            READ_10 | WRITE_10 | VERIFY_10 => (be32(&block[2..6]), be16(&block[7..9])),
            READ_12 | WRITE_12 => (be32(&block[2..6]), be32(&block[6..10])),
            _ => (be32(&block[2..6]) << 32 | be32(&block[6..10]), be32(&block[10..14])),
        }
    }

    /// The extent of a read, write or verify, checked against the
    /// medium.
    fn checked_extent(
        &self,
        block: &[u8; 16],
        v: &mut Violations,
    ) -> std::result::Result<(u64, u64), Sense> {
        let (lba, count) = Self::extent(block);
        let blocks = self.medium.blocks();
        if lba.checked_add(count).is_none_or(|end| end > blocks) {
            v.report(format!(
                "command {:#04x} names {count} blocks from {lba}; the medium has {blocks}",
                block[0]
            ));
            return Err(Sense::OUT_OF_RANGE);
        }
        Ok((lba, count))
    }

    /// The shortest command block a command has.
    fn block_length(opcode: u8) -> usize {
        match opcode >> 5 {
            0 => 6,
            1 | 2 => 10,
            4 => 16,
            5 => 12,
            _ => 1,
        }
    }

    /// Runs a command up to its data phase.
    fn command(&mut self, cbw: &Wrapper, v: &mut Violations) -> Outcome {
        let block = &cbw.block;
        let opcode = block[0];
        if cbw.block_length < Self::block_length(opcode) {
            v.report(format!(
                "command {opcode:#04x} in a command block of {} bytes; it takes {}",
                cbw.block_length,
                Self::block_length(opcode)
            ));
            return Outcome::Failed(Sense::INVALID_FIELD);
        }
        // These two answer whatever the unit and its state.
        match opcode {
            INQUIRY => return self.inquiry(cbw),
            REQUEST_SENSE => {
                let mut data = std::mem::take(&mut self.sense).bytes();
                data.truncate(usize::from(block[4]));
                return Outcome::Data(data);
            }
            _ => {}
        }
        if cbw.lun != 0 {
            v.report(format!("command {opcode:#04x} to unit {}; the device has one", cbw.lun));
            return Outcome::Failed(Sense::NO_SUCH_UNIT);
        }
        if opcode == REPORT_LUNS {
            let mut data = vec![0; 16];
            data[3] = 8;
            data.truncate(usize::try_from(be32(&block[6..10])).unwrap_or(usize::MAX));
            return Outcome::Data(data);
        }
        if self.not_ready > 0 {
            if opcode == TEST_UNIT_READY {
                self.not_ready -= 1;
            }
            return Outcome::Failed(Sense::BECOMING_READY);
        }
        let blocks = self.medium.blocks();
        let size = self.medium.block_size();
        match opcode {
            TEST_UNIT_READY | START_STOP_UNIT | PREVENT_ALLOW_MEDIUM_REMOVAL => Outcome::Good,
            MODE_SENSE_6 | MODE_SENSE_10 => self.mode_sense(block),
            READ_FORMAT_CAPACITIES => {
                let mut data = vec![0, 0, 0, 8];
                data.extend_from_slice(&u32::try_from(blocks).unwrap_or(u32::MAX).to_be_bytes());
                // Formatted media, and the block length in three bytes.
                data.push(0x02);
                data.extend_from_slice(&size.to_be_bytes()[1..]);
                data.truncate(usize::try_from(be16(&block[7..9])).unwrap_or(usize::MAX));
                Outcome::Data(data)
            }
            READ_CAPACITY_10 => {
                // The last block; all ones says to ask with 16.
                let last = u32::try_from(blocks.saturating_sub(1)).unwrap_or(u32::MAX);
                let mut data = last.to_be_bytes().to_vec();
                data.extend_from_slice(&size.to_be_bytes());
                Outcome::Data(data)
            }
            SERVICE_ACTION_IN_16 if block[1] & 0x1f == READ_CAPACITY_16 => {
                let mut data = blocks.saturating_sub(1).to_be_bytes().to_vec();
                data.extend_from_slice(&size.to_be_bytes());
                data.resize(32, 0);
                data.truncate(usize::try_from(be32(&block[10..14])).unwrap_or(usize::MAX));
                Outcome::Data(data)
            }
            READ_6 | READ_10 | READ_12 | READ_16 => match self.checked_extent(block, v) {
                Ok((_, 0)) => Outcome::Good,
                Ok((lba, count)) => Outcome::Read { lba, count },
                Err(sense) => Outcome::Failed(sense),
            },
            WRITE_6 | WRITE_10 | WRITE_12 | WRITE_16 => {
                if self.medium.read_only() {
                    return Outcome::Failed(Sense::WRITE_PROTECTED);
                }
                match self.checked_extent(block, v) {
                    Ok((_, 0)) => Outcome::Good,
                    Ok((lba, count)) => Outcome::Write { lba, count },
                    Err(sense) => Outcome::Failed(sense),
                }
            }
            VERIFY_10 => match self.checked_extent(block, v) {
                // With BYTCHK the host would send data to compare.
                Ok(_) if block[1] & 0x02 != 0 => Outcome::Failed(Sense::INVALID_FIELD),
                Ok(_) => Outcome::Good,
                Err(sense) => Outcome::Failed(sense),
            },
            SYNCHRONIZE_CACHE_10 | SYNCHRONIZE_CACHE_16 => match self.medium.flush() {
                Ok(()) => Outcome::Good,
                Err(_) => Outcome::Failed(Sense::WRITE_ERROR),
            },
            _ => Outcome::Failed(Sense::INVALID_COMMAND),
        }
    }

    /// A command block wrapper arrived: runs the command and sets up
    /// what the host said would follow.
    fn begin(&mut self, cbw: &Wrapper, v: &mut Violations) {
        let size = self.block_size();
        let host = u64::from(cbw.length);
        let outcome = self.command(cbw, v);
        let opcode = cbw.block[0];
        let mut status = PASSED;
        if let Outcome::Failed(sense) = &outcome {
            self.sense = *sense;
            status = FAILED;
        }
        // What the device means to move, and which way.
        let (wanted, to_host) = match &outcome {
            Outcome::Good | Outcome::Failed(_) => (0, cbw.to_host),
            Outcome::Data(d) => (d.len() as u64, true),
            Outcome::Read { count, .. } => (count * size, true),
            Outcome::Write { count, .. } => (count * size, false),
        };
        let mut moves = wanted > 0;
        if moves && host > 0 && to_host != cbw.to_host {
            v.report(format!(
                "command {opcode:#04x} moves data {} and the wrapper says {}",
                if to_host { "to the host" } else { "from the host" },
                if cbw.to_host { "to the host" } else { "from the host" },
            ));
            status = PHASE_ERROR;
            moves = false;
        } else if wanted > host {
            v.report(format!(
                "command {opcode:#04x} moves {wanted} bytes and the wrapper says {host}"
            ));
            status = PHASE_ERROR;
            // A write is not done by halves; a read sends what fits.
            moves = to_host && host > 0;
        }
        if host == 0 {
            self.phase = Phase::Status(status_wrapper(cbw.tag, 0, status));
            return;
        }
        if cbw.to_host {
            let (pending, reading) = match outcome {
                Outcome::Data(mut d) if moves => {
                    d.truncate(usize::try_from(host).unwrap_or(usize::MAX));
                    (d, None)
                }
                Outcome::Read { lba, count } if moves => {
                    (Vec::new(), Some(Reading { next: lba, left: count.min(host.div_ceil(size)) }))
                }
                _ => (Vec::new(), None),
            };
            let real = if reading.is_some() { wanted.min(host) } else { pending.len() as u64 };
            self.phase = Phase::DataIn {
                tag: cbw.tag,
                pending,
                reading,
                left: host,
                residue: host - real,
                status,
            };
        } else {
            let writing = match outcome {
                Outcome::Write { lba, count } if moves => {
                    Some(Writing { next: lba, left: count * size, partial: Vec::new() })
                }
                _ => None,
            };
            let real = if writing.is_some() { wanted } else { 0 };
            self.phase =
                Phase::DataOut { tag: cbw.tag, writing, left: host, residue: host - real, status };
        }
    }

    /// Reads blocks ahead until `pending` holds `want` bytes or the
    /// command's blocks are read. A block that fails ends the reading.
    fn read_ahead(
        &mut self,
        pending: &mut Vec<u8>,
        reading: &mut Option<Reading>,
        want: usize,
    ) -> Option<Sense> {
        let size = self.block_size();
        let r = reading.as_mut()?;
        if pending.len() >= want {
            return None;
        }
        let needed = ((want - pending.len()) as u64).div_ceil(size).min(r.left);
        let bad = first_bad(&self.faults.read_errors, r.next, needed);
        let count = bad.map_or(needed, |b| b - r.next);
        let at = pending.len();
        pending.resize(at + usize::try_from(count * size).unwrap_or(usize::MAX), 0);
        let read = if count == 0 { Ok(()) } else { self.medium.read(r.next, &mut pending[at..]) };
        if read.is_err() {
            pending.truncate(at);
        } else {
            r.next += count;
            r.left -= count;
        }
        if read.is_err() || bad.is_some() {
            *reading = None;
            return Some(Sense::READ_ERROR);
        }
        if r.left == 0 {
            *reading = None;
        }
        None
    }

    fn data_in(&mut self, max: usize) -> Reply {
        let Phase::DataIn { tag, mut pending, mut reading, left, mut residue, mut status } =
            std::mem::take(&mut self.phase)
        else {
            unreachable!("called in the data phase to the host");
        };
        let want = usize::try_from(left).unwrap_or(usize::MAX).min(max);
        if let Some(sense) = self.read_ahead(&mut pending, &mut reading, want) {
            // What was not read is fill from here on, and the residue
            // says so.
            residue = left.saturating_sub(pending.len() as u64);
            self.sense = sense;
            status = FAILED;
        }
        let mut chunk: Vec<u8> = pending.drain(..pending.len().min(want)).collect();
        // The rest of what the host asked for is fill.
        if reading.is_none() && pending.is_empty() {
            chunk.resize(want, 0);
        }
        let left = left - chunk.len() as u64;
        self.phase = if left == 0 {
            Phase::Status(status_wrapper(tag, residue, status))
        } else {
            Phase::DataIn { tag, pending, reading, left, residue, status }
        };
        Reply::Data(chunk)
    }

    fn data_out(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        let size = usize::try_from(self.block_size()).unwrap_or(usize::MAX);
        let Phase::DataOut { tag, mut writing, left, mut residue, mut status } =
            std::mem::take(&mut self.phase)
        else {
            unreachable!("called in the data phase from the host");
        };
        let taken = data.len().min(usize::try_from(left).unwrap_or(usize::MAX));
        if taken < data.len() {
            v.report(format!("{} bytes of data where the wrapper left {left} to come", data.len()));
        }
        if let Some(w) = &mut writing {
            let mine = taken.min(usize::try_from(w.left).unwrap_or(usize::MAX));
            w.partial.extend_from_slice(&data[..mine]);
            w.left -= mine as u64;
            let whole = w.partial.len() / size * size;
            let count = (whole / size) as u64;
            let bad = first_bad(&self.faults.write_errors, w.next, count);
            let good = bad.map_or(count, |b| b - w.next);
            let bytes = usize::try_from(good).unwrap_or(usize::MAX) * size;
            let wrote =
                if good == 0 { Ok(()) } else { self.medium.write(w.next, &w.partial[..bytes]) };
            if wrote.is_err() || bad.is_some() {
                // What is not on the medium is the residue.
                let written = if wrote.is_ok() { bytes as u64 } else { 0 };
                let unwritten = (w.partial.len() as u64 - written) + w.left;
                residue = (residue + unwritten).min(u64::from(u32::MAX));
                self.sense = Sense::WRITE_ERROR;
                status = FAILED;
                writing = None;
            } else {
                w.partial.drain(..whole);
                w.next += count;
            }
        }
        let left = left - taken as u64;
        self.phase = if left == 0 {
            Phase::Status(status_wrapper(tag, residue, status))
        } else {
            Phase::DataOut { tag, writing, left, residue, status }
        };
        Reply::ack()
    }
}

impl<M: Medium> DeviceModel for Msc<M> {
    fn name(&self) -> &'static str {
        self.name
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        let ours = setup.kind() == CLASS
            && setup.recipient() == RECIPIENT_INTERFACE
            && setup.index == u16::from(INTERFACE);
        match (ours, setup.request, setup.is_in()) {
            (true, GET_MAX_LUN, true) => {
                if setup.value != 0 || setup.length != 1 {
                    v.report(format!(
                        "Get Max LUN with wValue {:#06x} and wLength {}",
                        setup.value, setup.length
                    ));
                }
                // One unit, number 0.
                Reply::Data(vec![0])
            }
            (true, BULK_ONLY_RESET, false) => {
                if setup.value != 0 || setup.length != 0 {
                    v.report(format!(
                        "Bulk-Only Mass Storage Reset with wValue {:#06x} and wLength {}",
                        setup.value, setup.length
                    ));
                }
                self.phase = Phase::Command;
                Reply::ack()
            }
            _ => {
                v.report(format!(
                    "request {:#04x} {:#04x} value {:#06x} index {:#06x}: not one the function answers",
                    setup.request_type, setup.request, setup.value, setup.index
                ));
                Reply::Stall
            }
        }
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP_OUT {
            v.report(format!("OUT transfer to endpoint {ep}; commands and data go to {EP_OUT}"));
            return Reply::Stall;
        }
        match self.phase {
            Phase::Command => match Self::wrapper(data, v) {
                Ok(cbw) => {
                    self.begin(&cbw, v);
                    Reply::ack()
                }
                Err(problem) => {
                    v.report(format!("{problem}: both endpoints stall until the reset"));
                    self.phase = Phase::Broken;
                    Reply::Stall
                }
            },
            Phase::DataOut { .. } => self.data_out(data, v),
            // The device is not done with the command before.
            Phase::DataIn { .. } | Phase::Status(_) => Reply::Nak,
            Phase::Broken => Reply::Stall,
        }
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != EP_IN {
            v.report(format!("IN transfer from endpoint {ep}; data and status come from {EP_IN}"));
            return Reply::Stall;
        }
        match self.phase {
            Phase::DataIn { .. } => self.data_in(max),
            Phase::Status(wrapper) => {
                self.phase = Phase::Command;
                Reply::Data(wrapper.to_vec())
            }
            Phase::Command | Phase::DataOut { .. } => Reply::Nak,
            Phase::Broken => Reply::Stall,
        }
    }

    fn reset(&mut self) {
        self.phase = Phase::Command;
        self.sense = Sense::NONE;
        self.not_ready = self.faults.not_ready;
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::bus::{Bus, Status};
    use crate::device::Device;
    use crate::host::enumerate;
    use crate::observer::{Recorder, SharedObserver};

    /// What a command came to on the host.
    #[derive(Debug, PartialEq, Eq)]
    struct Done {
        data: Vec<u8>,
        status: u8,
        residue: u32,
    }

    struct Host {
        bus: Bus,
        seen: Arc<Mutex<Recorder>>,
        tag: u32,
        /// The length of each IN transfer of a data phase.
        chunk: usize,
    }

    impl Host {
        fn new<M: Medium + 'static>(faults: Faults, medium: M) -> Host {
            let id = Identity::new(0x1209, 0x0005, "vusb", "Test disk");
            let seen = Arc::new(Mutex::new(Recorder::default()));
            let shared: SharedObserver = seen.clone();
            let model = Msc::new("disk", &id, faults, medium);
            let mut bus = Bus::new(Device::new(Box::new(model))).with_observer(shared);
            enumerate(&mut bus).unwrap();
            Host { bus, seen, tag: 0, chunk: 1 << 16 }
        }

        fn wrapper(&mut self, block: &[u8], length: u32, to_host: bool) -> Vec<u8> {
            self.tag += 1;
            let mut cbw = CBW_SIGNATURE.to_vec();
            cbw.extend_from_slice(&self.tag.to_le_bytes());
            cbw.extend_from_slice(&length.to_le_bytes());
            cbw.extend_from_slice(&[if to_host { 0x80 } else { 0 }, 0, block.len() as u8]);
            cbw.extend_from_slice(block);
            cbw.resize(CBW_LENGTH, 0);
            cbw
        }

        fn status(&mut self) -> (u8, u32) {
            let csw = self.bus.read(EP_IN, 13);
            assert_eq!(csw.status, Status::Ok);
            assert_eq!(csw.data.len(), 13);
            assert_eq!(csw.data[..4], CSW_SIGNATURE);
            assert_eq!(csw.data[4..8], self.tag.to_le_bytes(), "the wrapper's tag");
            (
                csw.data[12],
                u32::from_le_bytes([csw.data[8], csw.data[9], csw.data[10], csw.data[11]]),
            )
        }

        /// A command with data to the host, or none.
        fn read(&mut self, block: &[u8], length: u32) -> Done {
            let cbw = self.wrapper(block, length, true);
            assert_eq!(self.bus.write(EP_OUT, &cbw).status, Status::Ok);
            let mut data = Vec::new();
            while data.len() < length as usize {
                let ask = self.chunk.min(length as usize - data.len());
                let part = self.bus.read(EP_IN, ask);
                assert_eq!(part.status, Status::Ok);
                assert!(!part.data.is_empty(), "the data phase stopped at {}", data.len());
                data.extend_from_slice(&part.data);
            }
            let (status, residue) = self.status();
            Done { data, status, residue }
        }

        /// A command with data from the host, in transfers of `cut`
        /// bytes.
        fn write(&mut self, block: &[u8], data: &[u8], cut: usize) -> Done {
            let cbw = self.wrapper(block, data.len() as u32, false);
            assert_eq!(self.bus.write(EP_OUT, &cbw).status, Status::Ok);
            for part in data.chunks(cut) {
                assert_eq!(self.bus.write(EP_OUT, part).status, Status::Ok);
            }
            let (status, residue) = self.status();
            Done { data: Vec::new(), status, residue }
        }

        fn sense(&mut self) -> Sense {
            let done = self.read(&[REQUEST_SENSE, 0, 0, 0, 18, 0], 18);
            assert_eq!((done.status, done.residue), (PASSED, 0));
            assert_eq!(done.data[0], 0x70);
            Sense { key: done.data[2], code: done.data[12], qualifier: done.data[13] }
        }

        fn violations(&self) -> Vec<String> {
            self.seen.lock().unwrap().violations.clone()
        }
    }

    fn read_10(lba: u32, count: u16) -> Vec<u8> {
        let mut block = vec![READ_10, 0];
        block.extend_from_slice(&lba.to_be_bytes());
        block.push(0);
        block.extend_from_slice(&count.to_be_bytes());
        block.push(0);
        block
    }

    fn write_10(lba: u32, count: u16) -> Vec<u8> {
        let mut block = read_10(lba, count);
        block[0] = WRITE_10;
        block
    }

    /// A disk whose every byte tells its place.
    fn pattern(blocks: usize) -> Vec<u8> {
        (0..blocks * 512).map(|i| (i / 512 * 7 + i % 251) as u8).collect()
    }

    #[test]
    fn a_host_finds_a_disk() {
        let mut host = Host::new(Faults::default(), Memory::new(64));
        let lun = Setup { request_type: 0xa1, request: GET_MAX_LUN, value: 0, index: 0, length: 1 };
        assert_eq!(host.bus.control(lun, &[]).data, [0]);

        let inquiry = host.read(&[INQUIRY, 0, 0, 0, 36, 0], 36);
        assert_eq!((inquiry.status, inquiry.residue), (PASSED, 0));
        assert_eq!(inquiry.data[..5], [0x00, 0x00, 0x04, 0x02, 31]);
        assert_eq!(&inquiry.data[8..16], b"vusb    ");
        assert_eq!(&inquiry.data[16..32], b"Test disk       ");
        assert_eq!(&inquiry.data[32..36], b"1.00");

        assert_eq!(host.read(&[TEST_UNIT_READY, 0, 0, 0, 0, 0], 0).status, PASSED);
        let capacity = host.read(&[READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0], 8);
        assert_eq!(capacity.data, [0, 0, 0, 63, 0, 0, 2, 0], "the last block, and 512 bytes");
        let mut long = vec![SERVICE_ACTION_IN_16, READ_CAPACITY_16];
        long.extend_from_slice(&[0; 8]);
        long.extend_from_slice(&[0, 0, 0, 32, 0, 0]);
        let capacity = host.read(&long, 32);
        assert_eq!(capacity.data[..12], [0, 0, 0, 0, 0, 0, 0, 63, 0, 0, 2, 0]);

        // Linux asks for the caching page with 192 bytes: what the
        // device lacks is fill, and the residue says so.
        let mode = host.read(&[MODE_SENSE_6, 0, PAGE_CACHING, 0, 192, 0], 192);
        assert_eq!((mode.status, mode.residue), (PASSED, 192 - 24));
        assert_eq!(mode.data[..6], [23, 0, 0, 0, PAGE_CACHING, 18]);
        assert!(mode.data[6..].iter().all(|b| *b == 0));
        let mode = host.read(&[MODE_SENSE_10, 0, PAGE_ALL, 0, 0, 0, 0, 0, 28, 0], 28);
        assert_eq!(mode.data[..10], [0, 26, 0, 0, 0, 0, 0, 0, PAGE_CACHING, 18]);

        let serial = host.read(&[INQUIRY, 1, 0x80, 0, 255, 0], 255);
        assert_eq!(&serial.data[..16], b"\x00\x80\x00\x0c000000000001");
        let formats = host.read(&[READ_FORMAT_CAPACITIES, 0, 0, 0, 0, 0, 0, 0, 12, 0], 12);
        assert_eq!(formats.data, [0, 0, 0, 8, 0, 0, 0, 64, 2, 0, 2, 0]);
        let units = host.read(&[REPORT_LUNS, 0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0], 16);
        assert_eq!(units.data[..4], [0, 0, 0, 8]);
        assert_eq!(host.violations(), Vec::<String>::new());
    }

    #[test]
    fn blocks_are_read_and_written() {
        let disk = pattern(16);
        let mut host = Host::new(Faults::default(), Memory::from_bytes(disk.clone()));
        let read = host.read(&read_10(3, 2), 1024);
        assert_eq!((read.status, read.residue), (PASSED, 0));
        assert_eq!(read.data, disk[3 * 512..5 * 512]);

        let fresh: Vec<u8> = (0..1536).map(|i| (i * 3) as u8).collect();
        let written = host.write(&write_10(5, 3), &fresh, 1 << 16);
        assert_eq!((written.status, written.residue), (PASSED, 0));
        assert_eq!(host.read(&read_10(5, 3), 1536).data, fresh);
        // The blocks around are as they were.
        assert_eq!(host.read(&read_10(4, 1), 512).data, disk[4 * 512..5 * 512]);
        assert_eq!(host.read(&read_10(8, 1), 512).data, disk[8 * 512..9 * 512]);

        // The other lengths of the same commands.
        let read = host.read(&[READ_6, 0, 0, 5, 1, 0], 512);
        assert_eq!(read.data, fresh[..512]);
        let read = host.read(&[READ_12, 0, 0, 0, 0, 6, 0, 0, 0, 1, 0, 0], 512);
        assert_eq!(read.data, fresh[512..1024]);
        let read = host.read(&[READ_16, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 1, 0, 0], 512);
        assert_eq!(read.data, fresh[1024..]);
        assert_eq!(host.read(&[SYNCHRONIZE_CACHE_10, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0).status, PASSED);
        assert_eq!(host.read(&[VERIFY_10, 0, 0, 0, 0, 0, 0, 0, 16, 0], 0).status, PASSED);
        assert_eq!(host.violations(), Vec::<String>::new());
    }

    #[test]
    fn the_cut_of_the_transfers_means_nothing() {
        let disk = pattern(16);
        let mut host = Host::new(Faults::default(), Memory::from_bytes(disk.clone()));
        // A read handed out a packet at a time, as through a gadget.
        host.chunk = 512;
        assert_eq!(host.read(&read_10(1, 8), 4096).data, disk[512..9 * 512]);
        // And in lengths that are no multiple of a block.
        host.chunk = 1536 + 512;
        assert_eq!(host.read(&read_10(0, 16), 8192).data, disk);

        // A write that arrives packet by packet, and one cut anywhere.
        let fresh: Vec<u8> = (0..2048).map(|i| (i * 5) as u8).collect();
        assert_eq!(host.write(&write_10(2, 4), &fresh, 512).status, PASSED);
        assert_eq!(host.read(&read_10(2, 4), 2048).data, fresh);
        assert_eq!(host.write(&write_10(9, 4), &fresh, 700).status, PASSED);
        assert_eq!(host.read(&read_10(9, 4), 2048).data, fresh);
        assert_eq!(host.violations(), Vec::<String>::new());
    }

    #[test]
    fn a_command_that_fails_keeps_the_data_phase_and_leaves_sense() {
        let mut host = Host::new(Faults::default(), Memory::new(8));
        // A command the device lacks: the host may ask.
        let toc = host.read(&[0x43, 0, 0, 0, 0, 0, 0, 0, 12, 0], 12);
        assert_eq!((toc.status, toc.residue), (FAILED, 12));
        assert_eq!(toc.data, [0; 12], "filled, as long as the host said");
        assert_eq!(host.sense(), Sense::INVALID_COMMAND);
        assert_eq!(host.sense(), Sense::NONE, "asked for once, it is gone");
        let page = host.read(&[MODE_SENSE_6, 0, 0x1c, 0, 192, 0], 192);
        assert_eq!((page.status, page.residue), (FAILED, 192));
        assert_eq!(host.sense(), Sense::INVALID_FIELD);
        assert_eq!(host.violations(), Vec::<String>::new());

        // Blocks the medium lacks: that the host should know.
        let beyond = host.read(&read_10(7, 2), 1024);
        assert_eq!((beyond.status, beyond.residue), (FAILED, 1024));
        assert_eq!(host.sense(), Sense::OUT_OF_RANGE);
        let beyond = host.write(&write_10(8, 1), &[0xaa; 512], 512);
        assert_eq!((beyond.status, beyond.residue), (FAILED, 512));
        assert_eq!(host.sense(), Sense::OUT_OF_RANGE);
        assert_eq!(host.violations().len(), 2, "{:?}", host.violations());
        // And the device goes on.
        assert_eq!(host.read(&read_10(7, 1), 512).status, PASSED);
    }

    #[test]
    fn what_the_wrapper_says_and_the_command_needs_must_agree() {
        let disk = pattern(8);
        let mut host = Host::new(Faults::default(), Memory::from_bytes(disk.clone()));
        // The host asks for less than the command has.
        let short = host.read(&read_10(0, 2), 512);
        assert_eq!(short.status, PHASE_ERROR);
        assert_eq!(short.data, disk[..512]);
        // More than it has: the rest is fill.
        let long = host.read(&read_10(0, 1), 1024);
        assert_eq!((long.status, long.residue), (PASSED, 512));
        assert_eq!(long.data[..512], disk[..512]);
        // The wrong direction: the host's data is taken and dropped.
        let wrong = host.write(&read_10(0, 1), &[0x55; 512], 512);
        assert_eq!(wrong.status, PHASE_ERROR);
        // A write the host is short of is not done by halves.
        let half = host.write(&write_10(0, 2), &[0x55; 512], 512);
        assert_eq!(half.status, PHASE_ERROR);
        assert_eq!(host.read(&read_10(0, 2), 1024).data, disk[..1024]);
        // Data where the command has none.
        let none = host.read(&[TEST_UNIT_READY, 0, 0, 0, 0, 0], 8);
        assert_eq!((none.status, none.residue, none.data.len()), (PASSED, 8, 8));
        assert_eq!(host.violations().len(), 3, "{:?}", host.violations());
    }

    #[test]
    fn a_wrapper_that_is_not_one_stalls_both_endpoints_until_the_reset() {
        let mut host = Host::new(Faults::default(), Memory::new(8));
        assert_eq!(host.bus.write(EP_OUT, b"not a wrapper").status, Status::Stall);
        assert_eq!(host.bus.read(EP_IN, 13).status, Status::Stall);
        let mut bad = host.wrapper(&[TEST_UNIT_READY, 0, 0, 0, 0, 0], 0, false);
        bad[0] = b'X';
        assert_eq!(host.bus.write(EP_OUT, &bad).status, Status::Stall);
        assert_eq!(host.violations().len(), 1, "the first is told, the rest follows from it");

        let reset =
            Setup { request_type: 0x21, request: BULK_ONLY_RESET, value: 0, index: 0, length: 0 };
        assert_eq!(host.bus.control(reset, &[]).status, Status::Ok);
        assert_eq!(host.read(&[TEST_UNIT_READY, 0, 0, 0, 0, 0], 0).status, PASSED);
    }

    #[test]
    fn a_write_protected_medium_says_so() {
        let disk = pattern(8);
        let mut host = Host::new(Faults::default(), Memory::from_bytes(disk.clone()).read_only());
        let mode = host.read(&[MODE_SENSE_6, 0, PAGE_ALL, 0, 192, 0], 192);
        assert_eq!(mode.data[2], 0x80, "the write protect bit");
        let refused = host.write(&write_10(0, 1), &[0x55; 512], 512);
        assert_eq!((refused.status, refused.residue), (FAILED, 512));
        assert_eq!(host.sense(), Sense::WRITE_PROTECTED);
        assert_eq!(host.read(&read_10(0, 1), 512).data, disk[..512]);
        assert_eq!(host.violations(), Vec::<String>::new());
    }

    fn faults(options: &[(&str, &str)]) -> Faults {
        let mut faults = Faults::default();
        for (k, v) in options {
            assert!(faults.option(k, v).unwrap(), "{k}");
        }
        faults
    }

    #[test]
    fn a_bad_block_fails_the_read_that_touches_it() {
        let disk = pattern(16);
        let mut host = Host::new(faults(&[("read-error", "5")]), Memory::from_bytes(disk.clone()));
        assert_eq!(host.read(&read_10(0, 5), 2560).status, PASSED);
        let bad = host.read(&read_10(4, 4), 2048);
        assert_eq!((bad.status, bad.residue), (FAILED, 1536));
        assert_eq!(bad.data[..512], disk[4 * 512..5 * 512], "the block before it is read");
        assert!(bad.data[512..].iter().all(|b| *b == 0));
        assert_eq!(host.sense(), Sense::READ_ERROR);
        // The same a packet at a time.
        host.chunk = 512;
        let bad = host.read(&read_10(4, 4), 2048);
        assert_eq!((bad.status, bad.residue), (FAILED, 1536));
        assert_eq!(bad.data[..512], disk[4 * 512..5 * 512]);
        assert_eq!(host.read(&read_10(6, 2), 1024).status, PASSED);
        assert_eq!(host.violations(), Vec::<String>::new());
    }

    #[test]
    fn a_bad_block_fails_the_write_that_touches_it() {
        let disk = pattern(16);
        let mut host = Host::new(faults(&[("write-error", "5")]), Memory::from_bytes(disk.clone()));
        let bad = host.write(&write_10(4, 4), &[0x77; 2048], 512);
        assert_eq!((bad.status, bad.residue), (FAILED, 1536));
        assert_eq!(host.sense(), Sense::WRITE_ERROR);
        assert_eq!(host.read(&read_10(4, 1), 512).data, [0x77; 512], "the block before it");
        assert_eq!(host.read(&read_10(5, 3), 1536).data, disk[5 * 512..8 * 512]);
        assert_eq!(host.violations(), Vec::<String>::new());
    }

    #[test]
    fn a_disk_that_spins_up_is_not_ready_at_first() {
        let mut host = Host::new(faults(&[("not-ready", "2")]), Memory::new(8));
        assert_eq!(host.read(&[INQUIRY, 0, 0, 0, 36, 0], 36).status, PASSED);
        for _ in 0..2 {
            assert_eq!(host.read(&read_10(0, 1), 512).status, FAILED);
            assert_eq!(host.read(&[TEST_UNIT_READY, 0, 0, 0, 0, 0], 0).status, FAILED);
            assert_eq!(host.sense(), Sense::BECOMING_READY);
        }
        assert_eq!(host.read(&[TEST_UNIT_READY, 0, 0, 0, 0, 0], 0).status, PASSED);
        assert_eq!(host.read(&read_10(0, 1), 512).status, PASSED);
        // A new power on spins up again.
        host.bus.reset();
        enumerate(&mut host.bus).unwrap();
        assert_eq!(host.read(&[TEST_UNIT_READY, 0, 0, 0, 0, 0], 0).status, FAILED);
        assert!(faults(&[]).option("not-ready", "x").is_err());
        assert!(!faults(&[]).option("stall", "1").unwrap());
        assert_eq!(host.violations(), Vec::<String>::new());
    }

    #[test]
    fn an_image_is_the_disk_and_keeps_what_was_written() {
        let path = std::env::temp_dir().join(format!("vusb-msc-{}.img", std::process::id()));
        let disk = pattern(16);
        // Bytes after the last whole block are not the disk's.
        let mut file = disk.clone();
        file.extend_from_slice(&[0xee; 100]);
        std::fs::write(&path, &file).unwrap();

        let mut host = Host::new(Faults::default(), Image::open(&path).unwrap());
        let capacity = host.read(&[READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0], 8);
        assert_eq!(capacity.data, [0, 0, 0, 15, 0, 0, 2, 0]);
        assert_eq!(host.read(&read_10(2, 3), 1536).data, disk[1024..2560]);
        assert_eq!(host.write(&write_10(1, 2), &[0x5a; 1024], 512).status, PASSED);
        assert_eq!(host.read(&[SYNCHRONIZE_CACHE_10, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0).status, PASSED);
        drop(host);

        let after = std::fs::read(&path).unwrap();
        assert_eq!(after.len(), file.len(), "the file keeps its size");
        assert_eq!(after[..512], disk[..512]);
        assert_eq!(after[512..1536], [0x5a; 1024]);
        assert_eq!(after[1536..], file[1536..]);

        // Write protected, the host is told and the file is not touched.
        let mut host = Host::new(Faults::default(), Image::open_read_only(&path).unwrap());
        assert_eq!(host.write(&write_10(0, 1), &[0; 512], 512).status, FAILED);
        assert_eq!(host.sense(), Sense::WRITE_PROTECTED);
        assert_eq!(host.read(&read_10(1, 1), 512).data, [0x5a; 512]);
        drop(host);
        assert_eq!(std::fs::read(&path).unwrap(), after);

        std::fs::write(&path, [0; 100]).unwrap();
        assert!(Image::open(&path).is_err(), "no whole block");
        std::fs::remove_file(&path).unwrap();
        assert!(Image::open(&path).is_err());
    }
}
