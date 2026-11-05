// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! `vusb`: tries a virtual USB device out. Runs one of the examples
//! the library comes with, for a demo or to see a transport work; a
//! product's own devices run from its own `main`, on the same
//! `vusb::session::Builder`.
//!
//! ```text
//! vusb list
//! sudo vusb run hid-relay --attach --open-devices --control /tmp/relay.sock
//! vusb-ctl -s /tmp/relay.sock status
//! sudo vusb cleanup
//! ```
//!
//! The exit status of `run` is 0, 2 when the host left the protocol, and
//! 1 on an error, the device detached by someone else included.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, Subcommand};
use serde_json::Value;
use vusb::attach::vhci;
use vusb::control::{Address, ListenOptions};
use vusb::observer::fixture::Rules;
use vusb::session::{BUSNUM, Builder, LogFormat};
use vusb::{Result, examples, options};

#[derive(Parser)]
#[command(name = "vusb", version, about = "Tries a virtual USB device out")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Lists the example devices.
    List,
    /// Serves an example device until told to stop.
    Run(Box<Run>),
    /// Detaches the ports a crashed run left attached (Linux, root).
    Cleanup {
        /// Every attached port, not only vusb's.
        #[arg(long)]
        all: bool,
    },
}

#[derive(Parser)]
#[expect(clippy::struct_excessive_bools, reason = "one command line switch each")]
struct Run {
    /// The example (see `list`).
    example: String,
    /// A model option, KEY=VALUE.
    #[arg(short = 'o', long = "option")]
    options: Vec<String>,
    /// The USB/IP address to listen on; port 0 picks a free one.
    #[arg(long, default_value = "127.0.0.1:3240")]
    listen: String,
    /// Attaches the device through vhci-hcd (Linux, root).
    #[arg(long, conflicts_with = "usbredir")]
    attach: bool,
    /// Serves the device over usbredir instead, for QEMU's usb-redir
    /// device: a socket path or host:port that QEMU connects to with
    /// `-chardev socket,id=u,path=PATH -device usb-redir,chardev=u`.
    #[arg(long, value_name = "ADDRESS")]
    usbredir: Option<String>,
    /// After attaching, makes the device nodes world accessible.
    #[arg(long)]
    open_devices: bool,
    /// The control socket: a path, or host:port.
    #[arg(long)]
    control: Option<String>,
    /// Who may connect to a control path, UID:GID; by default who ran sudo.
    #[arg(long, value_name = "UID:GID")]
    control_owner: Option<String>,
    /// Takes a control host:port that is not loopback. There is no
    /// authentication: anyone who reaches it controls the device.
    #[arg(long)]
    control_allow_remote: bool,
    /// Standard input and output are a control connection too; the end
    /// of input ends the run.
    #[arg(long)]
    stdio: bool,
    /// The transaction log file, truncated.
    #[arg(long)]
    log: Option<PathBuf>,
    /// Where the `log` command may start new logs; without it, it cannot.
    #[arg(long)]
    log_dir: Option<PathBuf>,
    /// The log's format: plain, or fixture, the dialect of recorded USB
    /// fixtures (`ctl_in 21 01 0300 0000 8 -> 4142`).
    #[arg(long, value_name = "FORMAT", default_value_t)]
    log_format: LogFormat,
    /// In the fixture format, data longer than N bytes is written as
    /// its length and SHA-256.
    #[arg(long, value_name = "N")]
    log_digest_above: Option<usize>,
    /// Logs comments too (connections, held transfers, unlinks).
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(0) => ExitCode::SUCCESS,
        Ok(code) => ExitCode::from(code),
        Err(e) => {
            eprintln!("vusb: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<u8> {
    match cli.command {
        Command::List => {
            for (name, about) in examples::NAMES {
                println!("{name}\t{about}");
            }
            Ok(0)
        }
        Command::Cleanup { all } => {
            for p in vhci::cleanup((!all).then_some(BUSNUM))? {
                println!("detached port {} (busid {}, devid {:#010x})", p.port, p.busid, p.devid);
            }
            Ok(0)
        }
        Command::Run(args) => serve(*args),
    }
}

fn parse_owner(text: &str) -> Result<(u32, u32)> {
    let parsed = text.split_once(':').and_then(|(u, g)| Some((u.parse().ok()?, g.parse().ok()?)));
    parsed.ok_or_else(|| vusb::Error::Invalid(format!("{text}: the owner is UID:GID")))
}

/// What a person watching wants of a lifecycle event, on standard error
/// (standard output carries the protocol under `--stdio`).
fn announce(event: &Value) {
    let name = event["event"].as_str().unwrap_or_default().to_uppercase();
    let mut fields = String::new();
    if let Some(map) = event.as_object() {
        for (k, v) in map.iter().filter(|(k, _)| *k != "event") {
            let v = v.as_str().map_or_else(|| v.to_string(), ToOwned::to_owned);
            let _ = write!(fields, " {k}={v}");
        }
    }
    eprintln!("{name}{fields}");
}

fn serve(args: Run) -> Result<u8> {
    let example = args.example.clone();
    let mut builder = Builder::new(Box::new(move |o| examples::build(&example, o)))
        .options(options::parse(&args.options)?)
        .listen(&args.listen)
        .attach(args.attach)
        .open_devices(args.open_devices)
        .log(args.log)
        .log_dir(args.log_dir)
        .log_format(args.log_format)
        .log_rules(args.log_digest_above.map_or_else(Rules::default, Rules::digest_above))
        .verbose(args.verbose)
        .stdio(args.stdio)
        .watch(announce);
    if let Some(usbredir) = &args.usbredir {
        builder = builder.usbredir(Address::parse(usbredir));
    }
    if let Some(control) = &args.control {
        let owner = args.control_owner.as_deref().map(parse_owner).transpose()?;
        let options = ListenOptions { owner, allow_remote: args.control_allow_remote };
        builder = builder.control_with(Address::parse(control), options);
    }
    let session = builder.start()?;
    if let Some(addr) = session.addr() {
        eprintln!("listening on {addr} busid 1-1");
    }
    if let Some(address) = session.usbredir_address() {
        eprintln!("usbredir on {address}");
    }
    for address in session.control_addresses() {
        eprintln!("control on {address}");
    }
    let stopper = session.stopper();
    ctrlc::set_handler(move || {
        let report = stopper.finish();
        eprintln!("violations: {}", report.violations);
        std::process::exit(if report.lost {
            1
        } else if report.violations == 0 {
            0
        } else {
            2
        });
    })
    .map_err(|e| vusb::Error::Other(format!("signal handler: {e}")))?;
    let report = session.run()?;
    eprintln!("violations: {}", report.violations);
    Ok(if report.violations == 0 { 0 } else { 2 })
}
