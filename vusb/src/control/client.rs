// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A client of the control protocol.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;

use serde_json::{Value, json};

use super::Address;
use crate::transport::socket::{Reader, Writer};
use crate::{Error, Result};

/// A connection to a running session.
pub struct Client {
    reader: BufReader<Reader>,
    writer: Writer,
    next: u64,
    events: VecDeque<Value>,
}

impl std::fmt::Debug for Client {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Client").field("pending_events", &self.events.len()).finish_non_exhaustive()
    }
}

impl Client {
    /// Connects.
    ///
    /// # Errors
    ///
    /// When nothing listens there.
    pub fn connect(address: &Address) -> Result<Client> {
        let (reader, writer): (Reader, Writer) = match address {
            Address::Tcp(a) => {
                let s = TcpStream::connect(a)?;
                s.set_nodelay(true)?;
                (Box::new(s.try_clone()?), Box::new(s))
            }
            #[cfg(unix)]
            Address::Unix(p) => {
                let s = std::os::unix::net::UnixStream::connect(p)?;
                (Box::new(s.try_clone()?), Box::new(s))
            }
            #[cfg(not(unix))]
            Address::Unix(p) => {
                return Err(Error::Invalid(format!("{}: Unix sockets need Unix", p.display())));
            }
        };
        Ok(Client { reader: BufReader::new(reader), writer, next: 1, events: VecDeque::new() })
    }

    fn read(&mut self) -> Result<Option<Value>> {
        let mut line = String::new();
        if self.reader.read_line(&mut line)? == 0 {
            return Ok(None);
        }
        serde_json::from_str(&line)
            .map(Some)
            .map_err(|e| Error::Protocol(format!("{e}: {}", line.trim_end())))
    }

    /// Sends a command and waits for its reply; the result. Events that
    /// arrive first wait for [`Client::next_event`].
    ///
    /// # Errors
    ///
    /// When the session refuses the command ([`Error::Other`], its
    /// message) or the connection fails.
    pub fn call(&mut self, cmd: &str, args: &Value) -> Result<Value> {
        let id = self.next;
        self.next += 1;
        let line = json!({"id": id, "cmd": cmd, "args": args}).to_string();
        writeln!(self.writer, "{line}")?;
        self.writer.flush()?;
        loop {
            let Some(v) = self.read()? else {
                return Err(Error::Protocol(format!(
                    "connection closed before the reply to {cmd}"
                )));
            };
            if v.get("event").is_some() {
                self.events.push_back(v);
            } else if v["id"] == json!(id) {
                return if v["ok"] == json!(true) {
                    Ok(v["result"].clone())
                } else {
                    Err(Error::Other(v["error"].as_str().unwrap_or("refused").to_owned()))
                };
            }
        }
    }

    /// The next event, waiting for it; `None` when the session is gone.
    ///
    /// # Errors
    ///
    /// When the connection fails.
    pub fn next_event(&mut self) -> Result<Option<Value>> {
        if let Some(v) = self.events.pop_front() {
            return Ok(Some(v));
        }
        loop {
            let Some(v) = self.read()? else { return Ok(None) };
            if v.get("event").is_some() {
                return Ok(Some(v));
            }
        }
    }
}
