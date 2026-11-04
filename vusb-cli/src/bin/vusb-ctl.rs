// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! `vusb-ctl`: drives a running vusb session through its control socket.
//!
//! Any command goes through, those a program added to its session
//! included: `KEY=VALUE` words become the arguments (a value that reads
//! as a number or a boolean is one), and the result is printed as
//! `key=value` lines. It exits 0 when the command was taken and 1 when
//! not, with the session's reason on standard error.
//!
//! ```text
//! vusb-ctl -s /tmp/relay.sock hello
//! vusb-ctl -s /tmp/relay.sock arm event=unplug when=in=1:3
//! vusb-ctl -s /tmp/relay.sock options serial=QWERT
//! vusb-ctl -s /tmp/relay.sock reset
//! vusb-ctl -s /tmp/relay.sock subscribe tx violations events
//! vusb-ctl -s /tmp/relay.sock subscribe events --until fired
//! ```
//!
//! `subscribe` prints each event as a line of JSON (`--lines` prints a
//! transfer as text) until the session ends, or `--until` sees an event
//! of that name.

use std::process::ExitCode;

use clap::Parser;
use serde_json::{Map, Value, json};
use vusb::control::{Address, Client};
use vusb::{Error, Result};

#[derive(Parser)]
#[command(name = "vusb-ctl", version, about = "Drives a running vusb session")]
struct Cli {
    /// The session's control socket: a path, or host:port.
    #[arg(short, long, env = "VUSB_CONTROL")]
    socket: String,
    /// Prints the result as JSON.
    #[arg(long)]
    json: bool,
    /// The arguments as one JSON object, merged under the KEY=VALUE ones.
    #[arg(long, value_name = "OBJECT")]
    args: Option<String>,
    /// With `subscribe`: ends after an event of this name.
    #[arg(long, value_name = "EVENT")]
    until: Option<String>,
    /// With `subscribe`: prints a transfer as its line of text.
    #[arg(long)]
    lines: bool,
    /// The command (`hello` lists them).
    command: String,
    /// Its arguments, KEY=VALUE; for `subscribe`, the topics.
    words: Vec<String>,
}

fn main() -> ExitCode {
    match run(&Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("vusb-ctl: {e}");
            ExitCode::FAILURE
        }
    }
}

/// A word's value: a number or a boolean when it reads as one.
fn value(text: &str) -> Value {
    match serde_json::from_str::<Value>(text) {
        Ok(v) if v.is_number() || v.is_boolean() => v,
        _ => json!(text),
    }
}

fn arguments(cli: &Cli) -> Result<Value> {
    let mut map = match &cli.args {
        Some(text) => match serde_json::from_str(text) {
            Ok(Value::Object(m)) => m,
            _ => return Err(Error::Invalid("--args takes a JSON object".into())),
        },
        None => Map::new(),
    };
    for word in &cli.words {
        let Some((k, v)) = word.split_once('=') else {
            return Err(Error::Invalid(format!("{word}: arguments are KEY=VALUE")));
        };
        map.insert(k.to_owned(), value(v));
    }
    Ok(Value::Object(map))
}

fn print_result(result: &Value, as_json: bool) {
    if as_json {
        println!("{result}");
        return;
    }
    if let Some(map) = result.as_object() {
        for (k, v) in map {
            match v.as_str() {
                Some(s) => println!("{k}={s}"),
                None => println!("{k}={v}"),
            }
        }
    }
}

fn stream(client: &mut Client, cli: &Cli) -> Result<()> {
    let topics: Vec<&str> = cli.words.iter().map(String::as_str).collect();
    client.call("subscribe", &json!({ "topics": topics }))?;
    while let Some(event) = client.next_event()? {
        let name = event["event"].as_str().unwrap_or_default();
        match event["line"].as_str() {
            Some(line) if cli.lines => println!("{line}"),
            _ => println!("{event}"),
        }
        if cli.until.as_deref() == Some(name) {
            break;
        }
    }
    Ok(())
}

fn run(cli: &Cli) -> Result<()> {
    let mut client = Client::connect(&Address::parse(&cli.socket))
        .map_err(|e| Error::Other(format!("{}: {e}", cli.socket)))?;
    if cli.command == "subscribe" {
        return stream(&mut client, cli);
    }
    let result = client.call(&cli.command, &arguments(cli)?)?;
    print_result(&result, cli.json);
    Ok(())
}
