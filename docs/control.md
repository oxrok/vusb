# The control protocol

A running session takes commands on a Unix socket, on TCP, or on standard input
and output. `vusb-ctl` is a client of it, and anything that can write a line to a
socket can be another.

```sh
vusb run hid-relay --control /tmp/relay.sock    # a path, or host:port

vusb-ctl -s /tmp/relay.sock status              # or set VUSB_CONTROL
vusb-ctl -s /tmp/relay.sock arm event=unplug when=in=1:3
```

`vusb-ctl` turns `KEY=VALUE` words into arguments, prints the result as `key=value`
lines (`--json` for JSON), and exits 1 if the command was refused.

## Wire format

One JSON object per line. Every request gets exactly one reply with the same `id`.
What has an `event` field is not a reply: it goes to connections that subscribed.

```text
-> {"id":1,"cmd":"arm","args":{"event":"unplug","when":"in=1:3"}}
<- {"id":1,"ok":true,"result":{}}
-> {"id":2,"cmd":"subscribe","args":{"topics":["tx","events"]}}
<- {"id":2,"ok":true,"result":{}}
<- {"event":"tx","section":"run","line":"bulk 0x82 64 -> a5a5","endpoint":130,...}
<- {"event":"fired","name":"unplug","section":"run"}
```

All connections feed one queue, together with what the device asks for, so a
command never overlaps a re-attach.

## Commands

| Command | Arguments | What it does |
|---|---|---|
| `hello` | | the protocol version, the commands, the topics |
| `status` | | where the device is and how it stands |
| `endpoints` | | the active configuration's endpoints |
| `reset` | | a fresh model; attached again if its descriptors changed |
| `options` | `KEY=VALUE`..., or `pairs`: a list of `[KEY, VALUE]` | the model's options for the resets that follow; none restores the first ones |
| `unplug`, `replug` | | leaves the bus as a pulled cable does; comes back |
| `arm` | `event=unplug` or `event=mark name=NAME`, and `when=in=EP:N`, `out=EP:N` or `model` | fires once at that point of the host's traffic |
| `section` | `name=NAME` | what follows belongs to it, in the log and on the stream |
| `log` | `file=NAME`, `off=true`, `verbose=BOOL`, `format=plain\|fixture`, `digest-above=N\|off` | the transaction log at run time |
| `subscribe`, `unsubscribe` | `topics`: `tx`, `violations`, `events`, `comments` | a live stream on this connection |
| `sync` | | answers after every input taken before it, a renumeration included |
| `quit` | | detaches the device and ends the session |

The `events` topic carries `ready`, `gone`, `renumerated`, `replugged`, `unplugged`,
`fired`, `renumeration` and `section`.

## The log

`log file=NAME` starts a new log in the directory given with `--log-dir`, and
`log off=true` stops the log. The other arguments change the log that runs, and
those started later:

- `verbose=BOOL`: comments too, such as connections and unlinks.
- `format=plain` or `format=fixture`: the lines' format, from now on. `plain` is
  the default. `fixture` is made to be compared with golden files.
- `digest-above=N`: in the fixture format, data longer than N bytes is written as
  its length and SHA-256. `digest-above=off` writes every byte again.

Both formats are described in [log.md](log.md). The `line` of a `tx` event on a
stream is always in the plain format.

## Faults

- **Options** such as `stall=` and `in-stall=N` belong to a model or shell. They take
  effect with `options` then `reset`, from a fresh model. An object holds a key once
  and in no order, and `vusb-ctl` reads a word like `1e3` as a number, so give `pairs`
  for an option given twice, an order that matters, or a value that is text:
  `vusb-ctl options --args '{"pairs":[["stall","reset"],["stall","baud"]]}'`.
- **`arm`** picks a point in a run. `unplug` takes the device off the bus. `mark` only
  sends an event, and you decide what it means.

## Your own commands

`vusb-ctl` passes on commands it does not know, and `hello` lists them:

```rust,ignore
let session = Builder::new(factory)
    .command("set-temperature", "degrees=N: what the sensor reads", |args, ctx| {
        // ... change something the model shares ...
        ctx.emit("temperature-set", serde_json::json!({ "degrees": args["degrees"] }));
        Ok(serde_json::json!({}))
    })
    .start()?;
```

## Security

There is no authentication. A Unix socket is protected by its mode, 0660 and owned
by whoever ran `sudo`. A TCP port is open to whoever reaches it, other local users
on loopback included, so an address that is not loopback is refused without
`--control-allow-remote`.

A client can detach the device and make it misbehave. It cannot write files where
it likes: `log` only creates new files, in the directory given with `--log-dir`.
Nor can it make `vusb run` open one: a model option that names a file, the
`file=` of `msc-disk`, is taken from the command line only. A program of your
own that builds models from `options` decides that for itself.
