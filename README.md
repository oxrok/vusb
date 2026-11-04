# vusb

Virtual USB devices in Rust: write a device as plain code, put it on the
bus, and check what a driver does to it.

- A device is a `DeviceModel`: its descriptors, how it answers control,
  OUT and IN transfers (data, NAK or STALL), and what a bus reset does to
  it. No I/O, so the same model runs in a unit test and on a host.
- `Device` wraps a model with the standard requests every device answers
  (descriptors, address, configuration, status, halts) and checks what the
  host sends; a model adds its own checks. Both report violations.
- Shells carry what every device of a kind shares: HID, CDC-ACM, FTDI
  chips, the Cypress EZ-USB FX2 boot loader.
- Hosts put a model on the bus. The USB/IP server, attached through
  Linux's `vhci-hcd`, makes it a real device for libusb, hidraw, the
  kernel's class drivers and anything else on the machine. An in-process
  host drives it from a test.
- `Observer` sees every transfer, so a consumer keeps a transaction log,
  counts violations or streams what the host does.

Linux can attach a device; the models, shells and protocol code build and
test on Linux, macOS and Windows.

## Licence

MIT or Apache-2.0, at your option. See `NOTICE` for where the code comes
from.
