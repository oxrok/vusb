# Changelog

## Unreleased

- A gadget, a third transport: `transport::gadget::Server` puts the device
  on a USB device controller of the machine through Linux's Raw Gadget
  driver (`/dev/raw-gadget`), so it is on a real cable, for any host. Every
  control request reaches the model, enumeration included. The server
  stands in for the host: it asks the model for IN transfers ahead, takes
  OUT packet by packet, and makes a STALL a halt. An unplug pulls the
  connection down through the controller's `soft_connect`; a replug and a
  renumeration bind the controller anew. `Builder::gadget` and
  `vusb run --gadget UDC` (`auto` for the only controller) serve a session
  so, and `status` says `transport=gadget`, the `udc` and whether a `host`
  configured the device; `ready` and `gone` follow the host.
  `dummy_hcd` is a controller in software, for a machine with none.
  Isochronous endpoints are served but not tried on hardware. The new
  `gadget` feature is off by default in the library and on in `vusb-cli`:
  it brings the `libc` crate on Linux and the crate's only unsafe code, the
  driver's ioctls; without it the crate still forbids unsafe code.
  `docs/gadget.md` has what to know, and `tools/gadget-check.sh` checks it
  on `dummy_hcd`.
- Mass storage: `shells::msc` is a disk, the Bulk-Only Transport and the
  SCSI commands of a disk driver around a `Medium` of blocks. `Memory` is
  a disk in memory and `Image` a disk that is a file, made beforehand: its
  size is the disk's and what the host writes is in it. The data phase is
  always as long as the host said, filled when a command has less, so a
  host sees the same over every transport. Faults: `read-error=LBA`,
  `write-error=LBA`, `not-ready=N`. The `msc-disk` example takes
  `file=PATH`, or `size=MIB` of memory, `read-only=1` and `removable=1`;
  `vusb run` takes `file=` from its command line only, never from the
  control socket.
- Isochronous streams in real time, when the device asks:
  `DeviceModel::realtime`, `false` by default. The model still reads no
  clock and sends the same packets; the bus gives every service interval
  its time, so a transfer completes when the interval of its last packet
  is over, and the transfers of an endpoint follow each other with no
  gap. `uac-mic` and `uvc-cam` take `realtime=1`: a second of sound takes
  a second and the camera gives 25 frames a second, where without it the
  host gets them as fast as it asks. `Bus::submit_at` and `Bus::poll_at`
  take the time, for a test with its own; `Bus::next_due` says when to
  poll next, and the USB/IP server wakes for it; `Bus::submit_timed` is
  for a transport that times its streams itself, as usbredir does, where
  a stream was in real time already. `Endpoint::iso_interval` is an
  isochronous endpoint's service interval. `tools/audio-check.sh` and
  `tools/video-check.sh` check both ways.

## 0.2.1 (2026-11-05)

- A second transaction log format, `fixture`, beside the default: the
  compact lines of recorded USB fixtures (`ctl_in 21 01 0300 0000 8 ->
  4142`), nothing for a held transfer until it completes, data longer
  than a limit as `@LEN:SHA256`, and bytes the host leaves undefined as
  `xx`. `observer::fixture::Log` is the observer, on any writer, with
  rules that change while it runs; `Builder::log_format`,
  `Builder::log_rules`, `vusb run --log-format` and `--log-digest-above`,
  and `format=` and `digest-above=` on the `log` command select it for a
  session's own log. Isochronous transfers are a line with their
  packets. `docs/log.md` describes both formats. The new `fixture`
  feature, on by default, brings the `sha2` crate.
- The `options` command takes `pairs`, a list of `[KEY, VALUE]`, as a model's
  options are: an object could not give the shells' repeatable `stall=` twice.
- `session::Builder::observer`: an observer of the bus from the start, the
  first attach included, for a program that keeps a log of its own.

## 0.2.0 (2026-11-05)

- usbredir, a second transport: `transport::usbredir::Server` serves the
  device to QEMU's `usb-redir` device, so it is inside a virtual machine
  on any platform QEMU runs on, with no kernel module and no root.
  `Builder::usbredir` and `vusb run --usbredir ADDRESS` select it;
  unplug, replug, reset and renumeration work as over USB/IP.
  `tools/qemu-check.sh` checks it in a real guest.
- Isochronous transfers: `DeviceModel::iso_in` and `iso_out`, one call
  per service interval; the bus completes an isochronous URB at once with
  a status per packet (`IsoPacket`, `Status::Missed`); USB/IP carries the
  packet descriptors and usbredir runs streams. `Urb::iso_packets` became
  `Urb::iso`, the packets' lengths; `Completion` and `Transfer` carry the
  packets.
- `uac-mic`, an example USB Audio microphone that hears a tone: the
  kernel's `snd-usb-audio` records it sample for sample, on the machine
  (`tools/audio-check.sh`) and in a QEMU guest.
- `uvc-cam`, an example USB Video camera that sees a test pattern: the
  host negotiates the format, and the kernel's `uvcvideo` captures the
  frames byte for byte, on the machine (`tools/video-check.sh`) and in a
  QEMU guest. Each frame shows its number, and `tools/frames.sh` reads a
  capture back.
- `tools/qemu-check.sh` takes the guest's kernel from `VUSB_GUEST_KERNEL`,
  for a machine whose own kernel lacks the drivers.
- `Session::addr` returns an `Option`: there is no USB/IP server over
  usbredir. The socket types moved to `transport::socket`; `control`
  still exports `Address` and `ListenOptions`.

## 0.1.1 (2026-11-05)

Documentation only; the code is that of 0.1.0.

- A README that says what vusb is for, with a picture, a logo, and the
  design and the control protocol on pages of their own.
- A front page for docs.rs, and examples on `DeviceModel`, `Bus`,
  `Observer` and the CDC-ACM shell.
- `vusb-cli` has its own README.

## 0.1.0 (2026-11-04)

- The device core (`DeviceModel`, `Device`, descriptors), the HID, CDC-ACM,
  FTDI and EZ-USB shells, the USB/IP server and the `vhci-hcd` attach path,
  taken out of oxrok's `emu` crate.
- `Observer`: a host reports every transfer, violation and notice to
  observers instead of writing a transaction log itself.
- `Arm`: point-triggered events (`unplug`, or a named `mark`) on the Nth
  transfer of an endpoint, or when the model marks its point.
- `Device` answers `GET_DESCRIPTOR` of the device qualifier for high speed
  devices.
- `Bus`: the host controller around a device, shared by every transport
  and by in-process tests: held transfers, per-endpoint order, bulk
  packet rounding, babble, unlink with held data, armed events, the
  renumeration signal. The USB/IP server is a codec on top of it.
- `host::enumerate`: enumerates a device on a `Bus` as a host does.
- `Session`: a model served and attached, one queue for everything that
  drives it; `Attachment` and `vhci::cleanup` for the ports.
- The control protocol: JSON lines on a Unix socket, TCP or standard
  input and output, with commands a program can add to, and live streams
  of transfers, violations and events; `control::Client`.
- `vusb-cli`: `vusb` runs the example devices, `vusb-ctl` drives any
  session. Examples `cdc-echo` and `ftdi-loopback`.
- `GET_DESCRIPTOR` of an OTG or debug descriptor is a STALL, not a
  violation: `lsusb -v` asks for them.
