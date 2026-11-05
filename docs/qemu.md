# A device inside a QEMU guest

vusb can serve a device over usbredir, the protocol of QEMU's `usb-redir`
device. The device then exists inside the virtual machine, whatever the guest
runs, and the host needs no kernel module and no root. It works wherever QEMU
does: Linux, macOS, Windows.

```sh
vusb run cdc-echo --usbredir /tmp/echo.sock --control /tmp/echo.ctl
```

Then start QEMU with a USB controller and a `usb-redir` device on that socket:

```sh
qemu-system-x86_64 ... \
    -device qemu-xhci \
    -chardev socket,id=vusb,path=/tmp/echo.sock \
    -device usb-redir,chardev=vusb
```

On Windows, or across machines, use TCP on both sides:
`--usbredir 127.0.0.1:4000` and `-chardev socket,id=vusb,host=127.0.0.1,port=4000`.

vusb listens and QEMU connects, so start vusb first. A guest that restarts
reconnects, and gets the device again.

## Driving it

Everything `vusb-ctl` does works the same way:

```sh
export VUSB_CONTROL=/tmp/echo.ctl
vusb-ctl status                        # transport=usbredir, guest=true
vusb-ctl unplug                        # the guest sees the device leave
vusb-ctl replug
vusb-ctl options chunk=1
vusb-ctl reset                         # a fresh model
vusb-ctl subscribe tx violations --lines
```

A reset that changes the descriptors, and a model that renumerates, take the
device out of the guest and put the new one in, on the same connection.

## In a program

```rust,ignore
let session = Builder::new(factory)
    .usbredir(Address::parse("/tmp/echo.sock"))
    .control(Address::parse("/tmp/echo.ctl"))
    .start()?;
```

## What to know

- **QEMU needs the device built in.** Some distributions package it apart:
  `qemu-device-usb-redirect` on Fedora. `qemu-system-x86_64 -device help`
  lists `usb-redir` when it is there.
- **One device per `usb-redir`.** For several, run several sessions and give
  QEMU one chardev and one `usb-redir` each.
- **Interrupt IN is polled by vusb**, every 20 ms, not at the endpoint's
  `bInterval`; QEMU asks for it once and gets what arrives.
- **No isochronous transfers yet**: a stream is refused with a STALL.
- **Timing in the guest is the guest's.** A model that counts the host's
  offers behaves as over USB/IP; what the guest's drivers do in between is
  as real as on hardware.

## Checking it

`tools/qemu-check.sh [EXAMPLE]` builds a small guest from this machine's
kernel, boots it in QEMU with the device, and checks that the guest's kernel
binds it, that data goes through, and that an unplug and a replug are seen. It
needs `dracut` and no root.
