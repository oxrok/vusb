# A device on a cable

vusb can put a device on a USB device controller of the machine it runs on,
through Linux's Raw Gadget driver. The device is then at the end of a real
cable, for whatever is at the other end: another machine and its operating
system, a phone, a console, a boot loader.

```sh
sudo modprobe raw_gadget
sudo vusb run cdc-echo --gadget auto --control /tmp/echo.ctl
```

`--gadget` takes the controller's name as `/sys/class/udc` lists it, or `auto`
for the only one.

## What it needs

- **A USB device controller (UDC).** It is the hardware of a device port:
  `dwc2` on a Raspberry Pi Zero or 4 (`dtoverlay=dwc2` and the USB-C or OTG
  port), `musb` on a Beagle board, and the like on most boards with an OTG
  port. A PC has none.
- **`dummy_hcd`, where there is none.** It is a controller in software, wired
  to a host controller of the same machine, so the device is on the machine
  itself, as with `--attach`. It is the way to try a gadget out with no
  board: `sudo modprobe dummy_hcd`.
- **The `raw_gadget` module.**
  [Raw Gadget](https://docs.kernel.org/usb/raw-gadget.html) is in mainline
  Linux since 5.7 (`CONFIG_USB_RAW_GADGET`), but not every distribution builds
  it, nor `dummy_hcd`: Arch has both, Ubuntu has `raw_gadget`, Fedora has
  `dummy_hcd`. For a kernel without one,
  [xairy/raw-gadget](https://github.com/xairy/raw-gadget) has both to build
  out of tree, in `raw_gadget/` and `dummy_hcd/`. Under Secure Boot the module
  must be signed.
- **Root.** `/dev/raw-gadget` and the controller's `soft_connect` are root's.

## Driving it

Everything `vusb-ctl` does works the same way:

```sh
export VUSB_CONTROL=/tmp/echo.ctl
vusb-ctl status                        # transport=gadget, host=true
vusb-ctl unplug                        # the host sees the cable pulled
vusb-ctl replug
vusb-ctl arm event=unplug when=out=2:3
vusb-ctl options chunk=1
vusb-ctl reset                         # a fresh model
vusb-ctl subscribe tx violations --lines
```

`ready` means a host configured the device, and `gone` that it reset it or
left. A reset that changes the descriptors, and a model that renumerates, take
the device off the bus and put the new one on.

## In a program

The `gadget` feature of the library is off by default: it brings the `libc`
crate, and the crate's only unsafe code, the driver's ioctls.

```rust,ignore
let session = Builder::new(factory)
    .gadget("auto")
    .control(Address::parse("/tmp/echo.ctl"))
    .start()?;
```

## What to know

Raw Gadget hands every control request to vusb, so the host talks to the model
as over the other transports, enumeration and its faults included. But a
device controller shows a device less than a host controller shows a host,
and vusb stands in for the host:

- **IN is asked ahead.** A device never learns that the host asked for data,
  only that data it had ready was taken. So the model is asked for each IN
  transfer before the host asks, and the answer waits in the controller. An
  armed `in=EP:N` counts what the model answered, which is one transfer ahead
  of what the host took.
- **OUT arrives packet by packet.** A write of 4096 bytes reaches the model as
  eight transfers of 512. The shells take a stream however it is cut; a model
  of your own must too.
- **What a model answers is what goes on the wire.** An answer of whole
  packets does not end the host's transfer, as on any device. An empty answer
  is a zero length packet.
- **A STALL is a halt**, which the host clears. On an OUT endpoint it meets
  the packet after the one the model refused, which the controller had
  acknowledged already.
- **A control OUT with data cannot get a STALL.** The model needs the data to
  decide, and reading the data acknowledges the request. The host sees it
  accepted, and the log has a comment.
- **A control transfer is answered within a second**, with a STALL if the
  model still holds it.
- **The controller has the endpoints it has.** Each endpoint of the device
  must find one of the controller's that fits its address, type and packet
  size. When one does not, `SET_CONFIGURATION` gets a STALL and the log, with
  `--verbose`, a comment that names it.
- **Linux before 6.7 reports no reset and no disconnect.** vusb then takes
  the death of an endpoint's transfer for a reset. It also cannot let go of
  the controller for a replug, and pulls the connection up again instead,
  which `dummy_hcd` does not come back from.
- **Isochronous endpoints have not been tried on hardware.** `dummy_hcd` has no
  isochronous transfers, so `uac-mic` and `uvc-cam` enumerate there and do not
  stream.
- **The kernel logs `UDC had already stopped`** at a replug and at the end:
  the connection is pulled down before the controller is let go of.

## Checking it

`sudo tools/gadget-check.sh` puts the serial port and a disk on `dummy_hcd`
and checks that the machine binds them, that data goes through, and that an
unplug and a replug are seen. It needs both modules.
