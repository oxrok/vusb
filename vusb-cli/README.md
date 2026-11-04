# vusb-cli

The command line tools of [vusb](https://crates.io/crates/vusb), a USB device
that is only Rust code: `vusb` runs one of the example devices, and `vusb-ctl`
drives any running vusb session.

```sh
cargo install vusb-cli
```

## vusb

Serves an example device, and on Linux attaches it to the machine through
`vhci-hcd`, so `lsusb`, libusb, hidraw and the kernel's class drivers see it.

```sh
vusb list                       # the example devices
sudo modprobe vhci-hcd
sudo vusb run hid-relay --attach --open-devices --control /tmp/relay.sock
sudo vusb cleanup               # the ports a killed run left attached
```

`vusb run` exits with status 0, 2 when the host left the USB protocol or the
device's own, and 1 on an error. The examples are a HID relay board, a CDC-ACM
serial port that echoes, and an FTDI chip looped back; a product's own devices
run from its own program, on the library's `Session`.

## vusb-ctl

Talks to a running session over its control socket, from a shell or a script.
Any command goes through, those a program added included.

```sh
export VUSB_CONTROL=/tmp/relay.sock
vusb-ctl status
vusb-ctl subscribe tx violations events --lines    # a live transaction log
vusb-ctl arm event=unplug when=in=0:3              # fault injection
vusb-ctl options relays=4                          # for the resets that follow
vusb-ctl reset
vusb-ctl replug
vusb-ctl quit
```

`KEY=VALUE` words become the command's arguments, the result is printed as
`key=value` lines (`--json` for JSON), and the exit status is 1 when the
session refused the command. The commands and the wire format are in the
[control protocol](https://github.com/oxrok/vusb/blob/main/docs/control.md).

## Licence

MIT or Apache-2.0, at your option.
