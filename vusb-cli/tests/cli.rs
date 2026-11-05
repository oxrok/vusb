// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The two binaries together, over TCP on loopback: no root, no
//! `vhci-hcd`, so this runs on every platform.

use std::io::{BufRead, BufReader};
use std::process::{Child, Command, Output, Stdio};

/// A `vusb run` of the relay board, and its control address.
struct Server {
    child: Child,
    control: String,
}

impl Server {
    fn start() -> Server {
        let mut child = Command::new(env!("CARGO_BIN_EXE_vusb"))
            .args(["run", "hid-relay", "--listen", "127.0.0.1:0", "--control", "127.0.0.1:0"])
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()
            .expect("vusb starts");
        let stderr = child.stderr.take().expect("stderr is piped");
        let mut lines = BufReader::new(stderr).lines();
        let control = lines
            .by_ref()
            .map(|l| l.expect("a line of stderr"))
            .find_map(|l| l.strip_prefix("control on ").map(str::to_owned))
            .expect("vusb says where its control socket is");
        // Keeps the pipe drained for as long as the server writes.
        std::thread::spawn(move || lines.for_each(drop));
        Server { child, control }
    }

    fn ctl(&self, words: &[&str]) -> Output {
        Command::new(env!("CARGO_BIN_EXE_vusb-ctl"))
            .args(["-s", &self.control])
            .args(words)
            .output()
            .expect("vusb-ctl runs")
    }

    fn ok(&self, words: &[&str]) -> String {
        let out = self.ctl(words);
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert!(out.status.success(), "{words:?}: {stderr}");
        String::from_utf8(out.stdout).expect("output is UTF-8")
    }
}

#[test]
fn vusb_ctl_drives_a_running_device() {
    let mut server = Server::start();
    let status = server.ok(&["status"]);
    assert!(status.lines().any(|l| l == "model=hid-relay"), "{status}");
    assert!(status.lines().any(|l| l == "unplugged=false"), "{status}");
    let hello = server.ok(&["--json", "hello"]);
    assert!(hello.contains("\"protocol\":1"), "{hello}");

    // Flags go on either side of the command.
    server.ok(&["section", "name=first"]);
    assert!(server.ok(&["status", "--json"]).contains("\"section\":\"first\""));

    server.ok(&["arm", "event=mark", "name=stop", "when=in=1:2"]);
    server.ok(&["options", "relays=4"]);
    let refused = server.ctl(&["options", "relays=99"]);
    assert_eq!(refused.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&refused.stderr).contains("relays=99"));
    assert_eq!(server.ctl(&["nope"]).status.code(), Some(1));
    assert_eq!(server.ctl(&["section", "bare-word"]).status.code(), Some(1));
    server.ok(&["reset"]);
    server.ok(&["unplug"]);
    assert!(server.ok(&["status"]).lines().any(|l| l == "unplugged=true"));
    server.ok(&["replug"]);

    server.ok(&["quit"]);
    assert_eq!(server.child.wait().expect("vusb ends").code(), Some(0));
}

#[test]
fn the_examples_are_listed_and_a_bad_one_is_refused() {
    let list = Command::new(env!("CARGO_BIN_EXE_vusb")).arg("list").output().expect("vusb runs");
    let text = String::from_utf8_lossy(&list.stdout);
    for name in ["hid-relay", "cdc-echo", "ftdi-loopback", "uac-mic", "uvc-cam"] {
        assert!(text.contains(name), "{text}");
    }
    let bad = Command::new(env!("CARGO_BIN_EXE_vusb"))
        .args(["run", "nope", "--listen", "127.0.0.1:0"])
        .output()
        .expect("vusb runs");
    assert_eq!(bad.status.code(), Some(1));
}
