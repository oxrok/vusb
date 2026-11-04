# vusb

Virtual USB devices in Rust: write a device as plain code, put it on the
bus, and check what a driver does to it. On Linux the device is real to
the whole machine: libusb, hidraw, `cdc_acm` and `ftdi_sio` all see it.

- A device is a `DeviceModel`: its descriptors, how it answers control,
  OUT and IN transfers (data, NAK or STALL), and what a bus reset does to
  it. No I/O, so the same model runs in a unit test and on a host.
- `Device` wraps a model with the standard requests every device answers
  and checks what the host sends; a model adds its own checks. Both report
  violations.
- Shells carry what every device of a kind shares: HID, CDC-ACM, FTDI
  chips, the Cypress EZ-USB FX2 boot loader.
- `Bus` is the host controller around a device: it holds a transfer the
  device NAKs, keeps each endpoint's order, cuts a babble, completes an
  unlink, and counts armed faults. Every transport and test drives the same
  one, so a device behaves the same on all of them.
- `Observer` sees every transfer, so a consumer keeps a transaction log,
  counts violations or streams what the host does.
- `Session` puts it together for a program, and its control socket lets
  `vusb-ctl` or any client reset, unplug, inject faults and watch while the
  device runs.

Linux can attach a device, through USB/IP and `vhci-hcd`. The models,
shells, in-process bus and control protocol build and test on Linux, macOS
and Windows.

## Try it

```sh
cargo install --path vusb-cli
sudo modprobe vhci-hcd

vusb list
sudo vusb run hid-relay --attach --open-devices --control /tmp/relay.sock
```

Under `sudo` the socket belongs to whoever ran `sudo`, so `vusb-ctl` needs
no root:

```sh
export VUSB_CONTROL=/tmp/relay.sock
vusb-ctl status
vusb-ctl subscribe tx violations events --lines   # a live transaction log
vusb-ctl arm event=unplug when=in=0:3             # pull the cable at the 3rd control read
vusb-ctl replug
vusb-ctl options relays=4                         # for the resets that follow
vusb-ctl reset                                    # a fresh model; re-attaches if descriptors changed
vusb-ctl quit
```

The exit status of `vusb run` is 0, 2 when the host left the protocol, and
1 on an error. `sudo vusb cleanup` detaches the ports a killed run left.

## Write a device

```rust
use vusb::bus::Bus;
use vusb::device::Device;
use vusb::host::enumerate;
use vusb::shells::cdc::{Cdc, Faults, Identity, Port, Serial};

struct Shout;

impl Serial for Shout {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        port.send(&data.to_ascii_uppercase());
    }
}

fn main() -> vusb::Result<()> {
    let id = Identity::new(0x1209, 0x0001, "Maker", "Shouter");
    let model = Cdc::new("shout", &id, Faults::default(), Shout);

    // In a test, with no kernel involved:
    let mut bus = Bus::new(Device::new(Box::new(model)));
    enumerate(&mut bus)?;
    bus.write(2, b"hello");
    assert_eq!(bus.read(2, 64).data, b"HELLO");
    Ok(())
}
```

To put a device on the machine, give `vusb::session::Builder` a function
that builds it, and call `.attach(true).control(...).start()?.run()`. See
`vusb-cli/src/bin/vusb.rs` for a complete program, and the
`vusb::examples` module for a model on each kind of shell.

## The control protocol

JSON lines on a Unix socket, TCP, or standard input and output, so anything
can drive a device without `vusb-ctl`:

```text
-> {"id":1,"cmd":"arm","args":{"event":"unplug","when":"in=1:3"}}
<- {"id":1,"ok":true,"result":{}}
-> {"id":2,"cmd":"subscribe","args":{"topics":["tx","events"]}}
<- {"id":2,"ok":true,"result":{}}
<- {"event":"tx","section":"run","line":"bulk 0x82 64 -> a5a5","endpoint":130,...}
<- {"event":"fired","name":"unplug","section":"run"}
```

| Command | Arguments | What it does |
|---|---|---|
| `hello` | | the protocol version, the commands, the topics |
| `status` | | where the device is and how it stands |
| `endpoints` | | the active configuration's endpoints |
| `reset` | | a fresh model; attached again if its descriptors changed |
| `options` | `KEY=VALUE`... | the model's options for the resets that follow; none restores the first |
| `unplug`, `replug` | | leaves the bus as a pulled cable does; comes back |
| `arm` | `event=unplug` or `event=mark name=NAME`, `when=in=EP:N`, `out=EP:N`, `model` | fires once at that point of the host's traffic |
| `section` | `name=NAME` | what follows belongs to it, in the log and on the stream |
| `log` | `file=NAME`, `off=true`, `verbose=BOOL` | the transaction log at run time |
| `subscribe`, `unsubscribe` | `topics`: `tx`, `violations`, `events`, `comments` | a live stream on this connection |
| `sync` | | answers after every input taken before it, a renumeration included |
| `quit` | | detaches the device and ends the session |

Events on the `events` topic: `ready`, `renumerated`, `replugged`,
`unplugged`, `fired`, `renumeration`, `section`.

A program adds its own commands, and `vusb-ctl` passes them through without
knowing them:

```rust,ignore
let session = Builder::new(factory)
    .command("set-temperature", "degrees=N: what the sensor reads", |args, ctx| {
        // ... change something the model shares ...
        ctx.emit("temperature-set", serde_json::json!({ "degrees": args["degrees"] }));
        Ok(serde_json::json!({}))
    })
    .start()?;
```

Faults are of two kinds. Model and shell options (`stall=`, `in-stall=N`, ...)
take effect with `options` and `reset`, from a fresh model; `arm` picks a point
in a run.

## Security

The protocol has no authentication. A Unix socket is protected by its mode;
a TCP port is open to whoever reaches it, including other local users on
loopback, so anything but loopback is refused unless you pass
`--control-allow-remote`. Running `--attach` takes root, so a client of such
a server can detach the device and make it behave as it likes, but `log` only
creates new files in the directory given with `--log-dir`.

## What it does not do yet

- Isochronous transfers: one is a violation and a STALL.
- Other transports: usbredir, for QEMU guests on any platform, is planned.
- Hubs: USB/IP and `vhci-hcd` are one device per port.
- Super speed.

## Licence

MIT or Apache-2.0, at your option. See `NOTICE` for where the code comes
from.
