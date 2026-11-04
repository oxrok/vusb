# Changelog

## Unreleased

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
