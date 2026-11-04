// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The control protocol: how a running [`crate::session::Session`] is
//! driven from outside, by `vusb-ctl` or any program that can write a
//! line to a socket.
//!
//! JSON lines, one object per line, on a Unix socket, on TCP, or on
//! standard input and output:
//!
//! ```text
//! -> {"id":1,"cmd":"reset"}
//! <- {"id":1,"ok":true,"result":{}}
//! -> {"id":2,"cmd":"arm","args":{"event":"unplug","when":"in=1:3"}}
//! <- {"id":2,"ok":false,"error":"..."}
//! -> {"id":3,"cmd":"subscribe","args":{"topics":["tx","events"]}}
//! <- {"id":3,"ok":true,"result":{}}
//! <- {"event":"tx","section":"run","line":"bulk 0x82 64 -> a5a5", ...}
//! ```
//!
//! A request has a `cmd` and optionally an `id` (any JSON value, echoed
//! back) and an `args` object. Every request gets exactly one reply,
//! with `ok` and either `result` or `error`. What has an `event` field
//! and no `ok` is not a reply; it comes to connections that subscribed.
//!
//! All connections feed one queue, which the session drains in order,
//! together with what the device asks for: `sync` answers after every
//! input taken before it, a renumeration included. `hello` lists the
//! commands, those a program added included.
//!
//! No authentication: a Unix socket is protected by its mode (0660, or
//! 0600 when no owner is known), and a TCP port is open to whoever
//! reaches it, so anything but loopback is refused unless asked for.

mod client;
mod listener;

pub use client::Client;
pub(crate) use listener::{Accepted, Listener};
pub use listener::{Address, ListenOptions};

use serde_json::{Value, json};

/// The protocol version, which `hello` reports.
pub const PROTOCOL: u32 = 1;

/// The longest line taken: a longer one closes the connection.
pub(crate) const MAX_LINE: u64 = 1 << 20;

/// A request, parsed.
#[derive(Clone, Debug, PartialEq)]
pub struct Request {
    /// What the reply echoes.
    pub id: Value,
    /// The command.
    pub cmd: String,
    /// Its arguments, always an object.
    pub args: Value,
}

impl Request {
    /// Parses a line.
    ///
    /// # Errors
    ///
    /// The message for the error reply: not JSON, no `cmd`, `args` not
    /// an object.
    pub fn parse(line: &str) -> std::result::Result<Request, String> {
        let v: Value = serde_json::from_str(line).map_err(|e| format!("not JSON: {e}"))?;
        let Value::Object(mut m) = v else { return Err("a request is an object".to_owned()) };
        let Some(Value::String(cmd)) = m.remove("cmd") else {
            return Err("a request has a string `cmd`".to_owned());
        };
        let args = m.remove("args").unwrap_or_else(|| json!({}));
        if !args.is_object() {
            return Err("`args` is an object".to_owned());
        }
        Ok(Request { id: m.remove("id").unwrap_or(Value::Null), cmd, args })
    }
}

/// The reply to a request.
pub fn reply(id: &Value, outcome: &std::result::Result<Value, String>) -> String {
    match outcome {
        Ok(result) => json!({"id": id, "ok": true, "result": result}),
        Err(error) => json!({"id": id, "ok": false, "error": error}),
    }
    .to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_requests() {
        let r = Request::parse(r#"{"id":7,"cmd":"arm","args":{"when":"model"}}"#).unwrap();
        assert_eq!((r.id, r.cmd.as_str()), (json!(7), "arm"));
        assert_eq!(r.args["when"], "model");
        let r = Request::parse(r#"{"cmd":"sync"}"#).unwrap();
        assert_eq!((r.id, r.args), (Value::Null, json!({})));
        for bad in ["", "reset", "[]", "{}", r#"{"cmd":1}"#, r#"{"cmd":"x","args":[]}"#] {
            assert!(Request::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn replies_echo_the_id() {
        assert_eq!(reply(&json!(1), &Ok(json!({}))), r#"{"id":1,"ok":true,"result":{}}"#);
        assert_eq!(
            reply(&json!("a"), &Err("no".to_owned())),
            r#"{"error":"no","id":"a","ok":false}"#
        );
    }
}
