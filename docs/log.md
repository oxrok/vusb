# The transaction log

A session writes what the host does to the device to a file, a line per
transfer: `vusb run --log FILE`, `Builder::log`, or the `log` command while it
runs. There are two formats. `plain` is the default; `fixture` is made to be
compared with golden files.

```sh
vusb run cdc-echo --log run.log --log-format fixture --log-digest-above 1024
vusb-ctl log format=plain                 # from now on
vusb-ctl log format=fixture digest-above=64
```

| What happened | `plain` | `fixture` |
|---|---|---|
| control IN | `control a1 01 0300 0000 0008 -> 4142` | `ctl_in 21 01 0300 0000 8 -> 4142` |
| control OUT | `control 21 09 0300 0000 0002 ff01 -> ok` | `ctl_out 21 09 0300 0000 ff01 -> ok` |
| bulk IN | `bulk 0x82 64 -> a5a5` | `bulk_in 82 64 -> a5a5` |
| bulk OUT | `bulk 0x02 2 0102 -> ok` | `bulk_out 02 0102 -> 2` |
| STALL | `-> stall` | `-> err stall` |
| held by a NAK | `-> held`, at once | nothing until it completes |
| babble | `-> 0102 babble, the URB holds 1` | `-> 0102 (babble: the URB holds 1)` |
| renumeration | `renumeration 04b4:8613` | `disconnect`, then `connect 04b4:8613` |

Both write `# section NAME`, `! violation: MESSAGE`, the word of an armed event
right after the transfer that fired it, and comments as `# TEXT` in verbose mode
only.

## The fixture format

One line per completed transfer. A log does not depend on how often the host
asks: a transfer the device holds is written when it completes.

- **The first word** is the type and the direction: `ctl_in`, `ctl_out`,
  `bulk_in`, `bulk_out`, `int_in`, `int_out`, `iso_in`, `iso_out`.
- **A control transfer** goes on with `bmRequestType`, `bRequest`, `wValue` and
  `wIndex` in hex. The direction bit of `bmRequestType` is cleared, since the
  word has it. An IN then has `wLength` in decimal, an OUT its data.
- **The others** go on with the endpoint address in two hex digits. An IN then
  has the length the device was offered, in decimal, an OUT its data.
- **After the arrow:** the data of an IN; `ok` for a control OUT; the bytes the
  device took, in decimal, for another OUT; `err stall`; `nak` for a transfer
  that ended while the device still held it.
- **Data** is hex, and `-` when there is none.
- **An unlinked IN** the device held data for is a line with the bytes that
  reached the URB.
- **In verbose mode** a held transfer is a comment: `# seq N: LINE (held)`.
- **An endpoint the configuration lacks**, a violation already, is written as
  `iso`.

### Digests and undefined bytes

Two rules depend on the device, so the caller sets them, and may change them
while the log runs.

- **`digest_above`:** data longer than this, in either direction, is written as
  `@LEN:SHA256`, its length and its SHA-256. A firmware upload is then not a
  line of megabytes. `--log-digest-above N` and `log digest-above=N` set it;
  `digest-above=off` writes every byte again.
- **`undefined`:** a function from an OUT transfer on a bulk, interrupt or
  isochronous endpoint to the ranges of its bytes the host leaves undefined,
  such as an uninitialised struct field. They are written as `xx`. In a digest
  they count as 0 and their ranges follow it, the ends included:
  `@LEN:SHA256/xx=1-2,5-5`. This rule is set in a program, with
  `Builder::log_rules` or on a `fixture::Log` of its own.

### Isochronous transfers

One line per transfer, as for the others, with the transfer's whole data, and
then its packets in order:

```text
iso_in 81 960 -> @960:5d0c... (packets: 96*10)
iso_in 83 48 -> 01020203030305050505050606060606060606 (packets: 1 2 3 missed 5 8/babble)
iso_out 04 010203 -> 3 (packets: 2 0 1)
```

- `N` is a packet of N bytes, and `0` an empty one.
- `missed` is a packet the device did not send or take.
- `N/babble` is a packet cut to the N bytes the host had room for.
- `*K` after a word says K packets alike in a row.

The number after the arrow of an `iso_out` is the bytes the device took in all.
The length of a missed OUT packet is not recorded.

## In a program

`vusb::observer::fixture::Log` is an observer that writes the fixture format to
any writer, flushed after every line. Its rules can change while it runs:

```rust,ignore
let log = Arc::new(Mutex::new(Log::new(file, Rules::digest_above(1024))));
let shared: SharedObserver = log.clone();
let bus = Bus::new(device).with_observer(shared);
// Later, when another model takes the device's place:
log.lock().unwrap().set_rules(Rules { digest_above: None, undefined: Some(ranges) });
```

The functions that make the lines are public too: `fixture::line` for a whole
transfer, and `control_line`, `transfer_line`, `data` and `hex` for their parts.
The format needs the `fixture` feature, on by default, which brings the `sha2`
crate.
