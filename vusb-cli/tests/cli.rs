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
        Server::start_with(&[])
    }

    /// With more arguments to `vusb run`.
    fn start_with(more: &[&str]) -> Server {
        Server::run("hid-relay", more)
    }

    /// A `vusb run` of `example`, with more arguments.
    fn run(example: &str, more: &[&str]) -> Server {
        let mut child = Command::new(env!("CARGO_BIN_EXE_vusb"))
            .args(["run", example, "--listen", "127.0.0.1:0", "--control", "127.0.0.1:0"])
            .args(more)
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
fn the_log_takes_a_format() {
    let path = std::env::temp_dir().join(format!("vusb-cli-format-{}.log", std::process::id()));
    let file = path.to_str().expect("a path in UTF-8");
    let mut server = Server::start_with(&[
        "--log",
        file,
        "--log-format",
        "fixture",
        "--log-digest-above",
        "1024",
    ]);
    server.ok(&["section", "name=run"]);
    server.ok(&["log", "format=plain", "digest-above=off"]);
    server.ok(&["log", "format=fixture", "digest-above=16", "verbose=true"]);
    let refused = server.ctl(&["log", "format=golden"]);
    assert_eq!(refused.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&refused.stderr).contains("plain and fixture"));
    assert_eq!(server.ctl(&["log", "digest-above=some"]).status.code(), Some(1));
    server.ok(&["quit"]);
    assert_eq!(server.child.wait().expect("vusb ends").code(), Some(0));
    let log = std::fs::read_to_string(&path).expect("the log is there");
    assert!(log.starts_with("# section attach\n# section run\n"), "{log}");
    std::fs::remove_file(&path).expect("the log goes");
    let bad = Command::new(env!("CARGO_BIN_EXE_vusb"))
        .args(["run", "hid-relay", "--listen", "127.0.0.1:0", "--log-format", "golden"])
        .output()
        .expect("vusb runs");
    assert!(!bad.status.success());
    assert!(String::from_utf8_lossy(&bad.stderr).contains("plain and fixture"));
}

#[test]
fn the_examples_are_listed_and_a_bad_one_is_refused() {
    let list = Command::new(env!("CARGO_BIN_EXE_vusb")).arg("list").output().expect("vusb runs");
    let text = String::from_utf8_lossy(&list.stdout);
    for name in ["hid-relay", "cdc-echo", "ftdi-loopback", "msc-disk", "uac-mic", "uvc-cam"] {
        assert!(text.contains(name), "{text}");
    }
    let bad = Command::new(env!("CARGO_BIN_EXE_vusb"))
        .args(["run", "nope", "--listen", "127.0.0.1:0"])
        .output()
        .expect("vusb runs");
    assert_eq!(bad.status.code(), Some(1));
}

#[test]
fn a_disk_image_is_named_on_the_command_line_only() {
    let path = std::env::temp_dir().join(format!("vusb-cli-disk-{}.img", std::process::id()));
    std::fs::write(&path, vec![0; 64 * 512]).expect("the image is written");
    let file = format!("file={}", path.display());
    let mut server = Server::run("msc-disk", &["-o", &file]);
    // Other options leave the file where it is; the file itself stays.
    server.ok(&["options", "not-ready=2"]);
    server.ok(&["reset"]);
    server.ok(&["options", &file]);
    let refused = server.ctl(&["options", "file=/etc/hostname"]);
    assert_eq!(refused.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&refused.stderr).contains("command line"));
    server.ok(&["quit"]);
    assert_eq!(server.child.wait().expect("vusb ends").code(), Some(0));
    std::fs::remove_file(&path).expect("the image goes");

    // And a disk in memory takes none afterwards.
    let mut server = Server::run("msc-disk", &["-o", "size=1"]);
    assert_eq!(server.ctl(&["options", &file]).status.code(), Some(1));
    server.ok(&["quit"]);
    assert_eq!(server.child.wait().expect("vusb ends").code(), Some(0));
}

#[test]
fn a_gadget_wants_a_controller_and_nothing_else() {
    let run = |more: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_vusb"))
            .args(["run", "hid-relay", "--listen", "127.0.0.1:0"])
            .args(more)
            .output()
            .expect("vusb runs")
    };
    let none = run(&["--gadget", "no-such-controller"]);
    assert_eq!(none.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&none.stderr).contains("controller"));
    // Not with the other ways to put a device somewhere.
    assert_eq!(run(&["--gadget", "auto", "--attach"]).status.code(), Some(2));
    assert_eq!(run(&["--gadget", "auto", "--usbredir", "127.0.0.1:0"]).status.code(), Some(2));
}
