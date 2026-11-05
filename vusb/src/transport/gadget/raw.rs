// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! `/dev/raw-gadget`: the ioctls of Linux's Raw Gadget driver
//! (`include/uapi/linux/usb/raw_gadget.h`), and the controller's
//! `soft_connect` in sysfs. The only unsafe code of the crate: an
//! ioctl takes a pointer the compiler cannot check.

use std::fs::{File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::path::PathBuf;

use super::{Event, Raw, Udc};
use crate::model::{Setup, Speed};

// _IOC(direction, 'U', number, size): the direction in the top two
// bits (1 write, 2 read), the size of the argument in the next 14.
const INIT: u32 = 0x4101_5500;
const RUN: u32 = 0x0000_5501;
const EVENT_FETCH: u32 = 0x8008_5502;
const EP0_WRITE: u32 = 0x4008_5503;
const EP0_READ: u32 = 0xc008_5504;
const EP_ENABLE: u32 = 0x4009_5505;
const EP_DISABLE: u32 = 0x4004_5506;
const EP_WRITE: u32 = 0x4008_5507;
const EP_READ: u32 = 0xc008_5508;
const CONFIGURE: u32 = 0x0000_5509;
const VBUS_DRAW: u32 = 0x4004_550a;
const EP0_STALL: u32 = 0x0000_550c;
const EP_SET_HALT: u32 = 0x4004_550d;
const EP_CLEAR_HALT: u32 = 0x4004_550e;
const EP_SET_WEDGE: u32 = 0x4004_550f;

/// `UDC_NAME_LENGTH_MAX`.
const NAME: usize = 128;
/// `struct usb_raw_init`: two names and the speed.
const INIT_SIZE: usize = 2 * NAME + 1;
/// The header of `struct usb_raw_event` and of `struct usb_raw_ep_io`,
/// before their data.
const HEADER: usize = 8;
/// `struct usb_endpoint_descriptor`, with the two bytes audio adds.
const ENDPOINT_DESCRIPTOR: usize = 9;
/// `USB_RAW_IO_FLAGS_ZERO`: end with a zero length packet when the data
/// is whole packets.
const FLAG_ZERO: u16 = 1;

const EVENT_CONNECT: u32 = 1;
const EVENT_CONTROL: u32 = 2;
const EVENT_SUSPEND: u32 = 3;
const EVENT_RESUME: u32 = 4;
const EVENT_RESET: u32 = 5;
const EVENT_DISCONNECT: u32 = 6;

/// `enum usb_device_speed`.
fn speed_code(speed: Speed) -> u8 {
    match speed {
        Speed::Low => 1,
        Speed::Full => 2,
        Speed::High => 3,
    }
}

/// An open, running Raw Gadget instance.
#[derive(Debug)]
pub(super) struct Device {
    file: File,
    soft_connect: PathBuf,
}

impl Device {
    /// Opens `/dev/raw-gadget`, binds it to the controller and puts the
    /// device on the bus.
    pub(super) fn open(udc: &Udc, speed: Speed) -> io::Result<Device> {
        let named = |what: &str, e: io::Error| io::Error::new(e.kind(), format!("{what}: {e}"));
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/raw-gadget")
            .map_err(|e| named("/dev/raw-gadget (sudo modprobe raw_gadget, and root)", e))?;
        let device = Device {
            file,
            soft_connect: PathBuf::from(format!("/sys/class/udc/{}/soft_connect", udc.device)),
        };
        let mut init = [0u8; INIT_SIZE];
        for (name, at) in [(&udc.driver, 0), (&udc.device, NAME)] {
            // The names end in a NUL.
            if name.len() >= NAME || name.contains('\0') {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("{name}: not the name of a controller"),
                ));
            }
            init[at..at + name.len()].copy_from_slice(name.as_bytes());
        }
        init[2 * NAME] = speed_code(speed);
        device.pointer(INIT, &mut init).map_err(|e| named("raw-gadget init", e))?;
        device
            .value(RUN, 0)
            .map_err(|e| named(&format!("binding to the controller {}", udc.device), e))?;
        Ok(device)
    }

    /// An ioctl whose argument points to `buffer`. Every request used
    /// here names a struct that `buffer` holds whole, with the length
    /// of its flexible data, when it has one, in its header: the kernel
    /// reads and writes within `buffer`.
    #[expect(unsafe_code, reason = "an ioctl with a pointer argument")]
    fn pointer(&self, request: u32, buffer: &mut [u8]) -> io::Result<usize> {
        // SAFETY: the descriptor is open for as long as `self` is, and
        // `buffer` is valid for reads and writes of its whole length,
        // which covers what the request has the kernel touch (see the
        // callers: each sizes the buffer as the request's struct, plus
        // the data length it writes into the header).
        let n = unsafe {
            libc::ioctl(self.file.as_raw_fd(), request as libc::Ioctl, buffer.as_mut_ptr())
        };
        usize::try_from(n).map_err(|_| io::Error::last_os_error())
    }

    /// An ioctl whose argument is a number, or none.
    #[expect(unsafe_code, reason = "an ioctl, with its argument by value")]
    fn value(&self, request: u32, value: u32) -> io::Result<usize> {
        // SAFETY: the descriptor is open for as long as `self` is, and
        // these requests take their argument by value: no memory of
        // this process is handed to the kernel.
        let n = unsafe {
            libc::ioctl(self.file.as_raw_fd(), request as libc::Ioctl, libc::c_ulong::from(value))
        };
        usize::try_from(n).map_err(|_| io::Error::last_os_error())
    }

    /// A `struct usb_raw_ep_io` with room for `length` bytes of data.
    fn io(handle: u32, flags: u16, length: usize) -> io::Result<Vec<u8>> {
        let too_long = || io::Error::new(io::ErrorKind::InvalidInput, "a transfer too long");
        let mut buffer = vec![0; HEADER + length];
        let handle = u16::try_from(handle).map_err(|_| too_long())?;
        buffer[..2].copy_from_slice(&handle.to_ne_bytes());
        buffer[2..4].copy_from_slice(&flags.to_ne_bytes());
        buffer[4..8].copy_from_slice(&u32::try_from(length).map_err(|_| too_long())?.to_ne_bytes());
        Ok(buffer)
    }

    fn send(&self, request: u32, handle: u32, data: &[u8], zero: bool) -> io::Result<usize> {
        let mut buffer = Self::io(handle, if zero { FLAG_ZERO } else { 0 }, data.len())?;
        buffer[HEADER..].copy_from_slice(data);
        self.pointer(request, &mut buffer)
    }

    fn receive(&self, request: u32, handle: u32, length: usize) -> io::Result<Vec<u8>> {
        let mut buffer = Self::io(handle, 0, length)?;
        let n = self.pointer(request, &mut buffer)?.min(length);
        buffer.truncate(HEADER + n);
        buffer.drain(..HEADER);
        Ok(buffer)
    }
}

impl Raw for Device {
    fn event(&self) -> io::Result<Event> {
        // The data of a control event is the setup packet.
        let mut buffer = [0u8; HEADER + 8];
        buffer[4..8].copy_from_slice(&8u32.to_ne_bytes());
        self.pointer(EVENT_FETCH, &mut buffer)?;
        let kind = u32::from_ne_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
        let mut setup = [0u8; 8];
        setup.copy_from_slice(&buffer[HEADER..]);
        Ok(match kind {
            EVENT_CONNECT => Event::Connect,
            EVENT_CONTROL => Event::Control(Setup::from_bytes(&setup)),
            EVENT_SUSPEND => Event::Suspend,
            EVENT_RESUME => Event::Resume,
            EVENT_RESET => Event::Reset,
            EVENT_DISCONNECT => Event::Disconnect,
            other => Event::Other(other),
        })
    }

    fn ep0_write(&self, data: &[u8], zero: bool) -> io::Result<usize> {
        self.send(EP0_WRITE, 0, data, zero)
    }

    fn ep0_read(&self, length: usize) -> io::Result<Vec<u8>> {
        self.receive(EP0_READ, 0, length)
    }

    fn ep0_stall(&self) -> io::Result<()> {
        self.value(EP0_STALL, 0).map(|_| ())
    }

    fn configure(&self) -> io::Result<()> {
        self.value(CONFIGURE, 0).map(|_| ())
    }

    fn vbus_draw(&self, units: u32) -> io::Result<()> {
        self.value(VBUS_DRAW, units).map(|_| ())
    }

    fn enable(&self, descriptor: &[u8]) -> io::Result<u32> {
        let mut buffer = [0u8; ENDPOINT_DESCRIPTOR];
        let length = descriptor.len().min(ENDPOINT_DESCRIPTOR);
        buffer[..length].copy_from_slice(&descriptor[..length]);
        let handle = self.pointer(EP_ENABLE, &mut buffer)?;
        u32::try_from(handle).map_err(|_| io::Error::other("an endpoint handle out of range"))
    }

    fn disable(&self, handle: u32) -> io::Result<()> {
        self.value(EP_DISABLE, handle).map(|_| ())
    }

    fn write(&self, handle: u32, data: &[u8]) -> io::Result<usize> {
        self.send(EP_WRITE, handle, data, false)
    }

    fn read(&self, handle: u32, length: usize) -> io::Result<Vec<u8>> {
        self.receive(EP_READ, handle, length)
    }

    fn halt(&self, handle: u32) -> io::Result<()> {
        self.value(EP_SET_HALT, handle).map(|_| ())
    }

    fn wedge(&self, handle: u32) -> io::Result<()> {
        self.value(EP_SET_WEDGE, handle).map(|_| ())
    }

    fn clear_halt(&self, handle: u32) -> io::Result<()> {
        self.value(EP_CLEAR_HALT, handle).map(|_| ())
    }

    fn pull_up(&self, on: bool) -> io::Result<()> {
        std::fs::write(&self.soft_connect, if on { "connect" } else { "disconnect" })
            .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", self.soft_connect.display())))
    }
}
