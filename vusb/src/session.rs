// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A session: a model served over USB/IP, attached when asked to, and
//! driven through control connections while it runs.
//!
//! ```no_run
//! use vusb::control::Address;
//! use vusb::examples::relay;
//! use vusb::session::Builder;
//!
//! # fn main() -> vusb::Result<()> {
//! let session = Builder::new(Box::new(|options| relay::build(options)))
//!     .attach(true)
//!     .control(Address::parse("/tmp/relay.sock"))
//!     .start()?;
//! let report = session.run()?;
//! println!("violations: {}", report.violations);
//! # Ok(())
//! # }
//! ```
//!
//! One queue takes everything that drives the run, in order: requests
//! from every connection, and what the device asks for (a renumeration,
//! an armed event that fired). So a command never overlaps a re-attach,
//! and `sync` answers after every input taken before it.
//!
//! A program adds commands of its own with [`Builder::command`]; the
//! generic client passes them through without knowing them, and `hello`
//! lists them. A command of the same name replaces a built in one.

use std::collections::BTreeMap;
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use crate::arm::{Arm, Event};
use crate::attach::{Attachment, Placed, PortGuard, vhci};
use crate::bus::{Bus, Renumeration, Signal};
use crate::control::{self, Accepted, Address, ListenOptions, Listener, MAX_LINE, Request};
use crate::device::Device;
use crate::model::{DeviceModel, Reply, Speed};
use crate::observer::fixture::{self, Rules};
use crate::observer::{Notice, Observer, SharedObserver, Transfer, hex};
use crate::transport::usbip::Server;
use crate::transport::usbredir;
use crate::{Error, Result};

/// Builds a model from `key=value` options, for the start and for each
/// reset.
pub type ModelFactory =
    Box<dyn Fn(&[(String, String)]) -> Result<Box<dyn DeviceModel>> + Send + 'static>;

type Handler = Box<dyn FnMut(&Value, &mut Ctx) -> Result<Value> + Send>;

/// Called with every lifecycle event, on the session's thread.
type Watcher = Box<dyn FnMut(&Value) + Send>;

/// The format of the session's transaction log.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum LogFormat {
    /// A line per transfer as [`Transfer::line`] writes it, held
    /// transfers included: `control a1 01 0300 0000 0008 -> 4142`.
    #[default]
    Plain,
    /// The dialect of recorded USB fixtures, made to be compared with
    /// golden files: `ctl_in 21 01 0300 0000 8 -> 4142`. See
    /// [`fixture`].
    Fixture,
}

impl LogFormat {
    /// The format's name, as [`str::parse`] takes it.
    pub fn name(self) -> &'static str {
        match self {
            LogFormat::Plain => "plain",
            LogFormat::Fixture => "fixture",
        }
    }
}

impl fmt::Display for LogFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl std::str::FromStr for LogFormat {
    type Err = Error;

    fn from_str(s: &str) -> Result<LogFormat> {
        match s {
            "plain" => Ok(LogFormat::Plain),
            "fixture" => Ok(LogFormat::Fixture),
            other => Err(Error::Invalid(format!("{other}: the log formats are plain and fixture"))),
        }
    }
}

/// The bus number announced by default (`0x7675` is "vu"); with the
/// device number it forms the `devid` by which [`vhci::cleanup`] finds
/// the ports left behind.
pub const BUSNUM: u32 = 0x7675;

/// Lines waiting for a client: one that lets more pile up, by not
/// reading, is dropped rather than slowing the device down.
const CLIENT_QUEUE: usize = 1 << 16;

/// How long the last replies get to reach their clients at the end.
const DRAIN: Duration = Duration::from_secs(1);

const POLL: Duration = Duration::from_millis(50);

struct Command {
    help: String,
    run: Handler,
}

/// What a session is to be.
#[expect(clippy::struct_excessive_bools, reason = "independent switches of a builder")]
pub struct Builder {
    factory: ModelFactory,
    options: Vec<(String, String)>,
    listen: String,
    busid: String,
    busnum: u32,
    attach: bool,
    open_devices: bool,
    log: Option<PathBuf>,
    log_dir: Option<PathBuf>,
    log_format: LogFormat,
    log_rules: Rules,
    verbose: bool,
    control: Vec<(Address, ListenOptions)>,
    usbredir: Option<(Address, ListenOptions)>,
    stdio: bool,
    watcher: Option<Watcher>,
    commands: BTreeMap<String, Command>,
}

impl Builder {
    /// A session of the models `factory` builds, listening on a free
    /// loopback port, not attached, with the built in commands.
    pub fn new(factory: ModelFactory) -> Builder {
        let mut b = Builder {
            factory,
            options: Vec::new(),
            listen: "127.0.0.1:0".to_owned(),
            busid: "1-1".to_owned(),
            busnum: BUSNUM,
            attach: false,
            open_devices: false,
            log: None,
            log_dir: None,
            log_format: LogFormat::default(),
            log_rules: Rules::default(),
            verbose: false,
            control: Vec::new(),
            usbredir: None,
            stdio: false,
            watcher: None,
            commands: BTreeMap::new(),
        };
        builtins(&mut b);
        b
    }

    /// The model's options at the start, which `options` with no
    /// arguments goes back to.
    #[must_use]
    pub fn options(mut self, options: Vec<(String, String)>) -> Builder {
        self.options = options;
        self
    }

    /// The USB/IP server's address.
    #[must_use]
    pub fn listen(mut self, address: &str) -> Builder {
        address.clone_into(&mut self.listen);
        self
    }

    /// The bus number announced, by which this program's ports are told
    /// from others'.
    #[must_use]
    pub fn busnum(mut self, busnum: u32) -> Builder {
        self.busnum = busnum;
        self
    }

    /// Attaches the device through `vhci-hcd` (Linux, root).
    #[must_use]
    pub fn attach(mut self, attach: bool) -> Builder {
        self.attach = attach;
        self
    }

    /// After attaching, makes the device nodes world accessible.
    #[must_use]
    pub fn open_devices(mut self, open: bool) -> Builder {
        self.open_devices = open;
        self
    }

    /// The transaction log from the start: this file, truncated.
    #[must_use]
    pub fn log(mut self, path: Option<PathBuf>) -> Builder {
        self.log = path;
        self
    }

    /// Where the `log` command may start new logs. Without it, the
    /// command can only stop the log: whoever reaches the socket of a
    /// server run as root must not get to write any file it names.
    #[must_use]
    pub fn log_dir(mut self, dir: Option<PathBuf>) -> Builder {
        self.log_dir = dir;
        self
    }

    /// The format of the transaction log; [`LogFormat::Plain`] unless
    /// set. The `log` command changes it while the session runs.
    #[must_use]
    pub fn log_format(mut self, format: LogFormat) -> Builder {
        self.log_format = format;
        self
    }

    /// The rules of a log in [`LogFormat::Fixture`]: what is written as
    /// a digest, and which bytes the host leaves undefined. A program
    /// whose rules change with the model keeps a
    /// [`fixture::Log`] of its own instead, and sets them there.
    #[must_use]
    pub fn log_rules(mut self, rules: Rules) -> Builder {
        self.log_rules = rules;
        self
    }

    /// Logs comments too (connections, held transfers, unlinks).
    #[must_use]
    pub fn verbose(mut self, verbose: bool) -> Builder {
        self.verbose = verbose;
        self
    }

    /// Takes control connections here too.
    #[must_use]
    pub fn control(self, address: Address) -> Builder {
        self.control_with(address, ListenOptions::default())
    }

    /// Takes control connections here too, opened so.
    #[must_use]
    pub fn control_with(mut self, address: Address, options: ListenOptions) -> Builder {
        self.control.push((address, options));
        self
    }

    /// Serves the device over usbredir on `address` instead of USB/IP,
    /// for QEMU's `usb-redir` device, which connects there with
    /// `-chardev socket`. Nothing is attached to this machine then:
    /// the device is inside the guest, and `ready` means a guest took
    /// it. Not with [`Builder::attach`].
    #[must_use]
    pub fn usbredir(self, address: Address) -> Builder {
        self.usbredir_with(address, ListenOptions::default())
    }

    /// Serves over usbredir on `address`, opened so.
    #[must_use]
    pub fn usbredir_with(mut self, address: Address, options: ListenOptions) -> Builder {
        self.usbredir = Some((address, options));
        self
    }

    /// Standard input and output are a control connection, subscribed
    /// to the lifecycle events; the end of input ends the session.
    #[must_use]
    pub fn stdio(mut self, stdio: bool) -> Builder {
        self.stdio = stdio;
        self
    }

    /// Called with every lifecycle event: `ready`, `renumerated`,
    /// `fired`, ...
    #[must_use]
    pub fn watch(mut self, f: impl FnMut(&Value) + Send + 'static) -> Builder {
        self.watcher = Some(Box::new(f));
        self
    }

    /// A command, which runs on the session's thread like the built in
    /// ones; one of the same name is replaced.
    #[must_use]
    pub fn command(
        mut self,
        name: &str,
        help: &str,
        run: impl FnMut(&Value, &mut Ctx) -> Result<Value> + Send + 'static,
    ) -> Builder {
        self.commands
            .insert(name.to_owned(), Command { help: help.to_owned(), run: Box::new(run) });
        self
    }

    /// Starts the transport's server on its own thread: usbredir when
    /// asked for, USB/IP otherwise.
    fn transport(
        &self,
        bus: &Arc<Mutex<Bus>>,
        hub: &Arc<Mutex<Hub>>,
        tx: &mpsc::Sender<Input>,
        stop: &Arc<AtomicBool>,
    ) -> Result<Transport> {
        if self.attach && self.usbredir.is_some() {
            return Err(Error::Invalid(
                "a device is attached here or served to QEMU, not both".into(),
            ));
        }
        let signal_tx = tx.clone();
        let signal_hub = Arc::clone(hub);
        let on_signal = move |signal| {
            // The section the transfer that caused it ran in, which a
            // re-attach goes back to; later is too late.
            let section = signal_hub.lock().expect("hub lock").section.clone();
            let _ = signal_tx.send(Input::Signal(signal, section));
        };
        let shared_stop = Arc::clone(stop);
        if let Some((address, options)) = &self.usbredir {
            let link_tx = tx.clone();
            let server = usbredir::Server::bind(Arc::clone(bus), address, options)?
                .on_signal(on_signal)
                .on_link(move |link| {
                    let _ = link_tx.send(Input::Link(link));
                });
            return Ok(Transport {
                addr: None,
                devid: 0,
                attachment: None,
                redir_address: Some(server.address()?),
                redir: Some(server.handle()),
                thread: thread::spawn(move || server.run(&shared_stop)),
            });
        }
        let devnum = if self.attach { vhci::pick_devnum(self.busnum) } else { 1 };
        let server =
            Server::new(Arc::clone(bus), &self.busid, self.busnum, devnum).on_signal(on_signal);
        let devid = server.devid();
        let listener = TcpListener::bind(&self.listen)?;
        let bound = listener.local_addr()?;
        Ok(Transport {
            addr: Some(bound),
            devid,
            attachment: self
                .attach
                .then(|| Attachment::new(bound, &self.busid, devid, self.open_devices)),
            redir: None,
            redir_address: None,
            thread: thread::spawn(move || server.run(&listener, &shared_stop)),
        })
    }

    /// Builds the model, opens the sockets and starts serving. Nothing
    /// is attached yet: that is [`Session::run`], so a signal handler
    /// can be in place first ([`Session::stopper`]).
    ///
    /// # Errors
    ///
    /// When the model refuses its options, or a socket cannot be opened.
    pub fn start(self) -> Result<Session> {
        let model = (self.factory)(&self.options)?;
        let log = match &self.log {
            Some(path) => Some(File::create(path)?),
            None => None,
        };
        let hub = Arc::new(Mutex::new(Hub {
            subscribers: Vec::new(),
            log,
            format: self.log_format,
            rules: self.log_rules,
            verbose: self.verbose,
            violations: 0,
            section: String::new(),
        }));
        let shared: SharedObserver = hub.clone();
        let bus = Arc::new(Mutex::new(Bus::new(Device::new(model)).with_observer(shared)));
        bus.lock().expect("bus lock").notice(Notice::Section("attach"));

        let (tx, rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let Transport { thread: server_thread, addr, devid, attachment, redir, redir_address } =
            self.transport(&bus, &hub, &tx, &stop)?;
        let guard = attachment.as_ref().map(Attachment::guard);

        let conn_ids = Arc::new(AtomicU64::new(1));
        let mut acceptors = Vec::new();
        let mut control_addresses = Vec::new();
        for (address, options) in &self.control {
            let listener = Listener::bind(address, options)?;
            let address = listener.address()?;
            control_addresses.push(address);
            let (tx, stop, ids) = (tx.clone(), Arc::clone(&stop), Arc::clone(&conn_ids));
            acceptors.push(thread::spawn(move || accept_loop(&listener, &tx, &stop, &ids)));
        }
        if self.stdio {
            let conn = conn_ids.fetch_add(1, Ordering::Relaxed);
            let accepted: Accepted = (Box::new(io::stdin()), Box::new(io::stdout()));
            connection(conn, accepted, &tx, true);
        }
        let ctx = Ctx {
            bus,
            hub,
            factory: self.factory,
            initial_options: self.options.clone(),
            options: self.options,
            attachment,
            redir,
            guest: false,
            placed: None,
            unplugged: false,
            unplugged_port: None,
            log_dir: self.log_dir,
            watcher: self.watcher,
            conns: BTreeMap::new(),
            conn: 0,
            quit: false,
            addr,
            devid,
            help: Vec::new(),
        };
        Ok(Session {
            ctx,
            commands: self.commands,
            rx,
            stop,
            guard,
            server_thread: Some(server_thread),
            acceptors,
            control_addresses,
            redir_address,
        })
    }
}

impl fmt::Debug for Builder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Builder")
            .field("options", &self.options)
            .field("listen", &self.listen)
            .field("busnum", &self.busnum)
            .field("attach", &self.attach)
            .field("control", &self.control)
            .finish_non_exhaustive()
    }
}

/// A transport's server, started.
struct Transport {
    thread: JoinHandle<Result<()>>,
    /// The USB/IP server's address.
    addr: Option<std::net::SocketAddr>,
    devid: u32,
    attachment: Option<Attachment>,
    redir: Option<usbredir::Handle>,
    redir_address: Option<Address>,
}

/// Stops a session from anywhere, a signal handler included.
#[derive(Clone, Debug)]
pub struct Stopper {
    stop: Arc<AtomicBool>,
    guard: Option<Arc<PortGuard>>,
    hub: Arc<Mutex<Hub>>,
}

impl Stopper {
    /// Detaches the device (once), stops serving, and reports. Safe to
    /// follow with the end of the process.
    pub fn finish(&self) -> Report {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(g) = &self.guard {
            g.release();
        }
        let violations = self.hub.lock().expect("hub lock").violations;
        Report { violations, lost: self.guard.as_ref().is_some_and(|g| g.lost()) }
    }
}

/// How a session ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Report {
    /// Violations of the protocol seen.
    pub violations: usize,
    /// Someone else detached the device during the run.
    pub lost: bool,
}

enum Input {
    Connected { conn: u64, tx: SyncSender<String>, writer: JoinHandle<()>, subscribed: bool },
    Line { conn: u64, line: String },
    Closed { conn: u64, ends_session: bool },
    Signal(Signal, String),
    Link(usbredir::Link),
}

fn accept_loop(listener: &Listener, tx: &mpsc::Sender<Input>, stop: &AtomicBool, ids: &AtomicU64) {
    while !stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok(Some(accepted)) => {
                let conn = ids.fetch_add(1, Ordering::Relaxed);
                connection(conn, accepted, tx, false);
            }
            Ok(None) => thread::sleep(POLL),
            Err(e) => {
                eprintln!("vusb: control: {e}");
                thread::sleep(POLL);
            }
        }
    }
}

/// Starts a connection's reader and writer threads. `primary` is the
/// standard input and output: subscribed from the start, and its end is
/// the session's.
fn connection(conn: u64, (reader, writer): Accepted, inputs: &mpsc::Sender<Input>, primary: bool) {
    let (tx, rx) = mpsc::sync_channel::<String>(CLIENT_QUEUE);
    let mut writer = writer;
    let writer_thread = thread::spawn(move || {
        for line in rx {
            if writeln!(writer, "{line}").and_then(|()| writer.flush()).is_err() {
                break;
            }
        }
    });
    let connected = Input::Connected { conn, tx, writer: writer_thread, subscribed: primary };
    if inputs.send(connected).is_err() {
        return;
    }
    let inputs = inputs.clone();
    thread::spawn(move || {
        let mut reader = BufReader::new(reader);
        let mut line = Vec::new();
        loop {
            line.clear();
            let n = reader.by_ref().take(MAX_LINE).read_until(b'\n', &mut line).unwrap_or(0);
            if n == 0 || (n as u64 == MAX_LINE && line.last() != Some(&b'\n')) {
                break;
            }
            let text = String::from_utf8_lossy(&line).into_owned();
            if inputs.send(Input::Line { conn, line: text }).is_err() {
                return;
            }
        }
        let _ = inputs.send(Input::Closed { conn, ends_session: primary });
    });
}

/// What topics a connection wants.
#[expect(clippy::struct_excessive_bools, reason = "one independent switch per topic")]
#[derive(Default)]
struct Subscriber {
    conn: u64,
    tx: Option<SyncSender<String>>,
    transfers: bool,
    violations: bool,
    events: bool,
    comments: bool,
}

/// The session's observer: the log, the count, and the subscribers.
/// Runs under the bus's lock, so nothing here waits.
struct Hub {
    subscribers: Vec<Subscriber>,
    log: Option<File>,
    format: LogFormat,
    rules: Rules,
    verbose: bool,
    violations: usize,
    section: String,
}

impl Hub {
    fn log_line(&mut self, line: &str) {
        if let Some(f) = &mut self.log {
            let _ = writeln!(f, "{line}");
        }
    }

    /// With the log in the fixture dialect, hands what was seen to a
    /// [`fixture::Log`] on the file, and says so: the plain format is
    /// then not written.
    fn fixture(&mut self, seen: impl FnOnce(&mut fixture::Log<&mut File>)) -> bool {
        if self.format != LogFormat::Fixture {
            return false;
        }
        if let Some(file) = &mut self.log {
            seen(&mut fixture::Log::new(file, self.rules).verbose(self.verbose));
        }
        true
    }

    fn send(&mut self, wants: impl Fn(&Subscriber) -> bool, message: &Value) {
        let mut text = None;
        self.subscribers.retain(|s| {
            let Some(tx) = &s.tx else { return false };
            if !wants(s) {
                return true;
            }
            let line = text.get_or_insert_with(|| message.to_string()).clone();
            match tx.try_send(line) {
                Ok(()) => true,
                Err(TrySendError::Full(_)) => {
                    eprintln!("vusb: control connection {} does not read; dropped", s.conn);
                    false
                }
                Err(TrySendError::Disconnected(_)) => false,
            }
        });
    }

    fn subscriber(&mut self, conn: u64, tx: &SyncSender<String>) -> &mut Subscriber {
        if let Some(i) = self.subscribers.iter().position(|s| s.conn == conn) {
            return &mut self.subscribers[i];
        }
        self.subscribers.push(Subscriber { conn, tx: Some(tx.clone()), ..Subscriber::default() });
        self.subscribers.last_mut().expect("just pushed")
    }
}

impl Observer for Hub {
    fn transfer(&mut self, t: &Transfer) {
        let line = t.line();
        if !self.fixture(|log| log.transfer(t)) {
            self.log_line(&line);
        }
        let setup = t.setup.map(|s| {
            json!({
                "request_type": s.request_type, "request": s.request,
                "value": s.value, "index": s.index, "length": s.length,
            })
        });
        let reply = match &t.reply {
            Reply::Data(d) => json!({"data": hex(d)}),
            Reply::Nak => json!("nak"),
            Reply::Stall => json!("stall"),
        };
        let message = json!({
            "event": "tx", "section": self.section, "line": line, "seqnum": t.seqnum,
            "endpoint": t.endpoint, "kind": t.kind_name(), "is_in": t.is_in, "setup": setup,
            "requested": t.requested, "offered": t.offered, "data": hex(&t.data),
            "reply": reply, "outcome": format!("{:?}", t.outcome).to_lowercase(),
        });
        self.send(|s| s.transfers, &message);
    }

    fn violation(&mut self, message: &str) {
        self.violations += 1;
        eprintln!("violation [{}]: {message}", self.section);
        if !self.fixture(|log| log.violation(message)) {
            self.log_line(&format!("! violation: {message}"));
        }
        let m = json!({"event": "violation", "section": self.section, "message": message});
        self.send(|s| s.violations, &m);
    }

    fn notice(&mut self, notice: Notice<'_>) {
        let plain = !self.fixture(|log| log.notice(notice));
        match notice {
            Notice::Comment(c) => {
                if plain && self.verbose {
                    self.log_line(&format!("# {c}"));
                }
                let m = json!({"event": "comment", "section": self.section, "message": c});
                self.send(|s| s.comments, &m);
            }
            Notice::Section(name) => {
                name.clone_into(&mut self.section);
                if plain {
                    self.log_line(&format!("# section {name}"));
                }
                self.send(|s| s.events, &json!({"event": "section", "name": name}));
            }
            Notice::Renumeration { vendor, product } => {
                if plain {
                    self.log_line(&format!("renumeration {vendor:04x}:{product:04x}"));
                }
                let m = json!({
                    "event": "renumeration", "section": self.section,
                    "vendor": vendor, "product": product,
                });
                self.send(|s| s.events, &m);
            }
            Notice::Fired(e) if plain => self.log_line(e.word()),
            Notice::Fired(_) => {}
        }
    }
}

fn placed_fields(p: &Placed) -> Value {
    let path = |p: &PathBuf| p.display().to_string();
    json!({
        "port": p.port, "busid": p.busid, "usb": p.bus_address,
        "usbfs": p.usbfs.as_ref().map(path), "nodes": p.nodes.iter().map(path).collect::<Vec<_>>(),
    })
}

impl fmt::Debug for Hub {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Hub")
            .field("subscribers", &self.subscribers.len())
            .field("violations", &self.violations)
            .field("section", &self.section)
            .finish_non_exhaustive()
    }
}

/// A running session, as a command sees it.
pub struct Ctx {
    bus: Arc<Mutex<Bus>>,
    hub: Arc<Mutex<Hub>>,
    factory: ModelFactory,
    initial_options: Vec<(String, String)>,
    options: Vec<(String, String)>,
    attachment: Option<Attachment>,
    /// The usbredir server, when that is the transport.
    redir: Option<usbredir::Handle>,
    /// A usbredir guest has the device.
    guest: bool,
    placed: Option<Placed>,
    unplugged: bool,
    unplugged_port: Option<u8>,
    log_dir: Option<PathBuf>,
    watcher: Option<Watcher>,
    conns: BTreeMap<u64, SyncSender<String>>,
    conn: u64,
    quit: bool,
    /// The USB/IP server's address; none over usbredir.
    addr: Option<std::net::SocketAddr>,
    devid: u32,
    help: Vec<Value>,
}

impl Ctx {
    /// The bus. Commands hold it briefly: the device stands still
    /// meanwhile.
    pub fn bus(&self) -> MutexGuard<'_, Bus> {
        self.bus.lock().expect("bus lock")
    }

    /// Ends the session once the command has answered.
    pub fn quit(&mut self) {
        self.quit = true;
    }

    /// Sends a lifecycle event to whoever subscribed to `events` and to
    /// the watcher; `event` is its `event` field.
    pub fn emit(&mut self, event: &str, mut fields: Value) {
        if let Value::Object(m) = &mut fields {
            m.insert("event".to_owned(), json!(event));
        } else {
            fields = json!({"event": event});
        }
        if let Some(w) = &mut self.watcher {
            w(&fields);
        }
        self.hub.lock().expect("hub lock").send(|s| s.events, &fields);
    }

    /// Starts a section: what follows belongs to it.
    pub fn section(&self, name: &str) {
        self.bus().notice(Notice::Section(name));
    }

    /// The section running.
    pub fn current_section(&self) -> String {
        self.hub.lock().expect("hub lock").section.clone()
    }

    /// Violations seen so far.
    pub fn violations(&self) -> usize {
        self.hub.lock().expect("hub lock").violations
    }

    /// Where the device is, when attached.
    pub fn placed(&self) -> Option<&Placed> {
        self.placed.as_ref()
    }

    /// Attaches the device, on port `want` when that one is free. Once
    /// the kernel has configured it, the log goes on in section `resume`
    /// when given, so that its enumeration is not in a run's section.
    fn attach(&mut self, want: Option<u8>, resume: Option<&str>) -> Result<Value> {
        if let Some(redir) = &self.redir {
            // The server resets the bus and announces the device; the
            // guest's enumeration follows on its own time.
            redir.connect();
            if let Some(name) = resume {
                self.section(name);
            }
            return Ok(json!({}));
        }
        let speed = self.bus().device().model_descriptors().speed;
        let bus = Arc::clone(&self.bus);
        let Some(attachment) = &mut self.attachment else {
            return Ok(json!({}));
        };
        let configured = || {
            if let Some(name) = resume {
                bus.lock().expect("bus lock").notice(Notice::Section(name));
            }
        };
        let placed = attachment.attach(speed, want, configured)?;
        let fields = placed_fields(&placed);
        self.placed = Some(placed);
        Ok(fields)
    }

    /// The device leaves the bus and comes back with the model's current
    /// descriptors, on the same port when it is still free. `resume` is
    /// the section to go on in, given when the model asked and the
    /// observers were told already.
    fn reattach(&mut self, why: &str, resume: Option<String>) -> Result<Value> {
        let resume = if let Some(r) = resume {
            r
        } else {
            let (vendor, product) = {
                let bus = self.bus();
                let m = bus.device().model_descriptors();
                (m.vendor_id(), m.product_id())
            };
            self.bus().notice(Notice::Renumeration { vendor, product });
            self.current_section()
        };
        self.bus().comment(&format!("{why}: detaching"));
        // From the detach on: the kernel's doing, not the tool's.
        self.section("renumeration");
        let port = self.take_off()?;
        self.bus().comment(&format!("{why}: attaching again"));
        self.attach(port, Some(&resume))
    }

    fn take_off(&mut self) -> Result<Option<u8>> {
        self.placed = None;
        if let Some(redir) = &self.redir {
            redir.disconnect();
            return Ok(None);
        }
        match &mut self.attachment {
            Some(a) => a.detach(),
            None => Ok(None),
        }
    }

    /// Whether the device is put somewhere: on this machine, or in a
    /// guest.
    fn served(&self) -> bool {
        self.attachment.is_some() || self.redir.is_some()
    }

    /// A fresh model behind the device, from the current options: its
    /// initial state for the next run. An unplugged device comes back
    /// first. A device whose descriptors changed (it renumerated, or
    /// the options say so) is attached again, in a `reset` section, not
    /// in the section of the run before.
    ///
    /// # Errors
    ///
    /// [`Error::Attach`] ends the session; the model refusing its
    /// options does not.
    pub fn reset(&mut self) -> Result<()> {
        self.replug()?;
        let fresh = (self.factory)(&self.options)?;
        let again = {
            let mut bus = self.bus();
            bus.replace_model(fresh);
            bus.comment("model reset");
            bus.device().needs_reenumeration()
        };
        if again && self.served() {
            self.section("reset");
            let fields = self.reattach("reset", None)?;
            self.emit("renumerated", fields);
        }
        Ok(())
    }

    /// The device leaves the bus, as a pulled cable does, until
    /// [`Ctx::replug`] or [`Ctx::reset`].
    ///
    /// # Errors
    ///
    /// When it cannot be detached.
    pub fn unplug(&mut self) -> Result<()> {
        if self.unplugged {
            return Ok(());
        }
        self.bus().unplug();
        self.leave()?;
        self.emit("unplugged", json!({}));
        Ok(())
    }

    fn leave(&mut self) -> Result<()> {
        self.bus().comment("unplug: detaching");
        self.unplugged_port = self.take_off()?;
        self.unplugged = true;
        Ok(())
    }

    /// An unplugged device comes back, on its port when that is free,
    /// its enumeration in a `replug` section. Nothing when it is not
    /// unplugged.
    ///
    /// # Errors
    ///
    /// When it cannot be attached.
    pub fn replug(&mut self) -> Result<()> {
        if !std::mem::take(&mut self.unplugged) {
            return Ok(());
        }
        self.section("replug");
        let port = self.unplugged_port.take();
        let fields = self.attach(port, None)?;
        if !self.served() {
            // Nobody re-imports it, so nothing else brings it back.
            self.bus().reset();
        }
        self.emit("replugged", fields);
        Ok(())
    }

    fn signal(&mut self, signal: Signal, section: String) -> Result<()> {
        match signal {
            Signal::Renumeration(Renumeration { .. }) => {
                if self.redir.is_some() {
                    // The server did the disconnect and the connect.
                    self.emit("renumerated", json!({}));
                } else if self.attachment.is_some() {
                    let fields = self.reattach("renumeration", Some(section))?;
                    self.emit("renumerated", fields);
                }
            }
            Signal::Fired(event) => {
                // The bus is off already; "fired" means off the machine
                // too, so whoever waits for it can look.
                if event == Event::Unplug {
                    self.leave()?;
                }
                self.emit("fired", json!({"name": event.word(), "section": section}));
            }
        }
        Ok(())
    }

    fn send(&mut self, conn: u64, line: String) {
        if let Some(tx) = self.conns.get(&conn)
            && tx.try_send(line).is_err()
        {
            self.conns.remove(&conn);
            self.hub.lock().expect("hub lock").subscribers.retain(|s| s.conn != conn);
        }
    }

    fn close(&mut self, conn: u64) {
        self.conns.remove(&conn);
        self.hub.lock().expect("hub lock").subscribers.retain(|s| s.conn != conn);
    }
}

impl fmt::Debug for Ctx {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Ctx")
            .field("addr", &self.addr)
            .field("redir", &self.redir)
            .field("placed", &self.placed)
            .field("unplugged", &self.unplugged)
            .finish_non_exhaustive()
    }
}

/// A session that is serving.
pub struct Session {
    ctx: Ctx,
    commands: BTreeMap<String, Command>,
    rx: mpsc::Receiver<Input>,
    stop: Arc<AtomicBool>,
    guard: Option<Arc<PortGuard>>,
    server_thread: Option<JoinHandle<Result<()>>>,
    acceptors: Vec<JoinHandle<()>>,
    control_addresses: Vec<Address>,
    redir_address: Option<Address>,
}

impl Session {
    /// Stops the session from elsewhere: a signal handler, a test.
    pub fn stopper(&self) -> Stopper {
        Stopper {
            stop: Arc::clone(&self.stop),
            guard: self.guard.clone(),
            hub: Arc::clone(&self.ctx.hub),
        }
    }

    /// The USB/IP server's address; none when the device is served
    /// over usbredir.
    pub fn addr(&self) -> Option<std::net::SocketAddr> {
        self.ctx.addr
    }

    /// Where QEMU connects, when the device is served over usbredir (a
    /// TCP port 0 is resolved).
    pub fn usbredir_address(&self) -> Option<&Address> {
        self.redir_address.as_ref()
    }

    /// The `devid` of the device, which a USB/IP client's URBs carry.
    pub fn devid(&self) -> u32 {
        self.ctx.devid
    }

    /// Where the control sockets ended up (a TCP port 0 is resolved).
    pub fn control_addresses(&self) -> &[Address] {
        &self.control_addresses
    }

    /// Attaches the device, when asked to, and takes inputs until `quit`,
    /// the end of standard input, or [`Stopper::finish`]; then detaches.
    ///
    /// # Errors
    ///
    /// When attaching fails, at the start or during the run, or someone
    /// else detaches the device.
    pub fn run(mut self) -> Result<Report> {
        let stopper = self.stopper();
        let result = self.drive();
        let report = stopper.finish();
        self.shut_down();
        result?;
        if report.lost {
            return Err(Error::Attach(
                "the device was detached by someone else during the run".into(),
            ));
        }
        Ok(report)
    }

    fn drive(&mut self) -> Result<()> {
        if self.ctx.attachment.is_some() {
            if !vhci::available() {
                return Err(Error::Attach(
                    "vhci-hcd is not loaded (sudo modprobe vhci-hcd)".into(),
                ));
            }
            let fields = self.ctx.attach(None, None)?;
            self.ctx.emit("ready", fields);
        }
        let mut writers = Vec::new();
        let result = self.inputs(&mut writers);
        // The last replies get out, unless a client stopped reading.
        self.ctx.conns.clear();
        self.ctx.hub.lock().expect("hub lock").subscribers.clear();
        let start = Instant::now();
        for w in writers {
            while !w.is_finished() && start.elapsed() < DRAIN {
                thread::sleep(Duration::from_millis(5));
            }
        }
        result
    }

    fn inputs(&mut self, writers: &mut Vec<JoinHandle<()>>) -> Result<()> {
        while !self.stop.load(Ordering::Relaxed) && !self.ctx.quit {
            match self.rx.recv_timeout(POLL) {
                Ok(Input::Connected { conn, tx, writer, subscribed }) => {
                    writers.retain(|w: &JoinHandle<()>| !w.is_finished());
                    writers.push(writer);
                    if subscribed {
                        let mut hub = self.ctx.hub.lock().expect("hub lock");
                        hub.subscriber(conn, &tx).events = true;
                    }
                    self.ctx.conns.insert(conn, tx);
                }
                Ok(Input::Line { conn, line }) => self.line(conn, line.trim())?,
                Ok(Input::Closed { conn, ends_session }) => {
                    self.ctx.close(conn);
                    if ends_session {
                        return Ok(());
                    }
                }
                Ok(Input::Signal(signal, section)) => self.ctx.signal(signal, section)?,
                Ok(Input::Link(link)) => {
                    let connected = link == usbredir::Link::Connected;
                    self.ctx.guest = connected;
                    self.ctx.emit(if connected { "ready" } else { "gone" }, json!({}));
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return Ok(()),
            }
        }
        Ok(())
    }

    /// Runs a request and answers it. What failed attaching ends the
    /// session, once the client knows.
    fn line(&mut self, conn: u64, line: &str) -> Result<()> {
        if line.is_empty() {
            return Ok(());
        }
        let request = match Request::parse(line) {
            Ok(r) => r,
            Err(e) => {
                self.ctx.send(conn, control::reply(&Value::Null, &Err(e)));
                return Ok(());
            }
        };
        self.ctx.conn = conn;
        self.ctx.help =
            self.commands.iter().map(|(name, c)| json!({"name": name, "help": c.help})).collect();
        let outcome = match self.commands.get_mut(&request.cmd) {
            Some(command) => (command.run)(&request.args, &mut self.ctx),
            None => Err(Error::Invalid(format!("no command {}", request.cmd))),
        };
        let (answer, fatal) = match outcome {
            Ok(v) => (Ok(v), None),
            Err(e @ Error::Attach(_)) => (Err(e.to_string()), Some(e)),
            Err(e) => (Err(e.to_string()), None),
        };
        self.ctx.send(conn, control::reply(&request.id, &answer));
        match fatal {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    fn shut_down(&mut self) {
        if let Some(a) = &mut self.ctx.attachment {
            a.close();
        }
        if let Some(t) = self.server_thread.take() {
            match t.join() {
                Ok(Ok(())) => {}
                Ok(Err(e)) => eprintln!("vusb: server: {e}"),
                Err(_) => eprintln!("vusb: server thread panicked"),
            }
        }
        for a in self.acceptors.drain(..) {
            let _ = a.join();
        }
    }
}

impl fmt::Debug for Session {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Session").field("ctx", &self.ctx).finish_non_exhaustive()
    }
}

fn arg_str<'a>(args: &'a Value, key: &str) -> Option<&'a str> {
    args.get(key).and_then(Value::as_str)
}

fn speed_name(speed: Speed) -> &'static str {
    match speed {
        Speed::Low => "low",
        Speed::Full => "full",
        Speed::High => "high",
    }
}

/// A value of an `options` argument: what the model parses, as text.
fn option_text(v: &Value) -> String {
    v.as_str().map_or_else(|| v.to_string(), ToOwned::to_owned)
}

fn log_command(args: &Value, ctx: &mut Ctx) -> Result<Value> {
    // What is refused is refused before anything changes.
    let format = arg_str(args, "format").map(str::parse::<LogFormat>).transpose()?;
    let digest_above = match args.get("digest-above") {
        None => None,
        Some(Value::String(s)) if s == "off" => Some(None),
        Some(n) => {
            Some(Some(n.as_u64().and_then(|n| usize::try_from(n).ok()).ok_or_else(|| {
                Error::Invalid(format!("digest-above={n}: a number of bytes, or off"))
            })?))
        }
    };
    let mut settings = format.is_some() || digest_above.is_some();
    {
        let mut hub = ctx.hub.lock().expect("hub lock");
        if let Some(format) = format {
            hub.format = format;
        }
        if let Some(above) = digest_above {
            hub.rules.digest_above = above;
        }
        if let Some(verbose) = args.get("verbose").and_then(Value::as_bool) {
            hub.verbose = verbose;
            settings = true;
        }
        if args.get("off").and_then(Value::as_bool) == Some(true) {
            hub.log = None;
            return Ok(json!({}));
        }
    }
    let Some(name) = arg_str(args, "file") else {
        return if settings {
            Ok(json!({}))
        } else {
            Err(Error::Invalid(
                "log takes file=NAME, off=true, verbose=BOOL, format=plain|fixture \
                 or digest-above=N|off"
                    .into(),
            ))
        };
    };
    let Some(dir) = &ctx.log_dir else {
        return Err(Error::Invalid(
            "this server was started without a log directory: it takes no new logs".into(),
        ));
    };
    let plain = Path::new(name).file_name().is_some_and(|n| n == name) && !name.starts_with('.');
    if !plain {
        return Err(Error::Invalid(format!("{name}: a plain file name in the log directory")));
    }
    let path = dir.join(name);
    // Never over what is there, a link planted there included.
    let file = OpenOptions::new().write(true).create_new(true).open(&path)?;
    ctx.hub.lock().expect("hub lock").log = Some(file);
    Ok(json!({"path": path.display().to_string()}))
}

fn subscribe_command(args: &Value, ctx: &mut Ctx, on: bool) -> Result<Value> {
    let Some(topics) = args.get("topics").and_then(Value::as_array) else {
        return Err(Error::Invalid("topics is a list: tx, violations, events, comments".into()));
    };
    let conn = ctx.conn;
    let Some(tx) = ctx.conns.get(&conn).cloned() else { return Ok(json!({})) };
    let mut hub = ctx.hub.lock().expect("hub lock");
    let subscriber = hub.subscriber(conn, &tx);
    for topic in topics {
        match topic.as_str() {
            Some("tx") => subscriber.transfers = on,
            Some("violations") => subscriber.violations = on,
            Some("events") => subscriber.events = on,
            Some("comments") => subscriber.comments = on,
            _ => return Err(Error::Invalid(format!("no topic {topic}"))),
        }
    }
    Ok(json!({}))
}

fn arm_command(args: &Value) -> Result<Arm> {
    let event = arg_str(args, "event");
    let when = arg_str(args, "when");
    let (Some(event), Some(when)) = (event, when) else {
        return Err(Error::Invalid(
            "arm takes event=unplug|mark (with name=NAME) and when=in=EP:N|out=EP:N|model".into(),
        ));
    };
    let spec = match (event, arg_str(args, "name")) {
        ("mark", Some(name)) => format!("mark {name} {when}"),
        (event, _) => format!("{event} {when}"),
    };
    Arm::parse(&spec)
}

fn status_command(ctx: &Ctx) -> Value {
    let placed = ctx.placed.as_ref().map(placed_fields);
    let bus = ctx.bus();
    let device = bus.device();
    let d = device.descriptors();
    json!({
        "model": device.name(),
        "transport": if ctx.redir.is_some() { "usbredir" } else { "usbip" },
        "listen": ctx.addr.map(|a| a.to_string()),
        "devid": ctx.devid,
        "attached": placed,
        "guest": ctx.redir.as_ref().map(|_| ctx.guest),
        "unplugged": ctx.unplugged,
        "speed": speed_name(d.speed),
        "vendor": d.vendor_id(),
        "product": d.product_id(),
        "configuration": device.configuration(),
        "held": bus.held(),
        "section": ctx.hub.lock().expect("hub lock").section,
        "violations": ctx.hub.lock().expect("hub lock").violations,
    })
}

fn endpoints_command(ctx: &Ctx) -> Value {
    let bus = ctx.bus();
    let device = bus.device();
    let endpoints = device
        .descriptors()
        .config_by_value(device.configuration())
        .map(crate::model::Descriptors::endpoints)
        .unwrap_or_default();
    let list: Vec<Value> = endpoints
        .iter()
        .map(|e| {
            json!({
                "address": e.address, "attributes": e.attributes,
                "max_packet": e.max_packet, "interval": e.interval,
            })
        })
        .collect();
    json!({"configuration": device.configuration(), "endpoints": list})
}

#[expect(clippy::too_many_lines, reason = "one table of the commands, each next to its help")]
fn builtins(b: &mut Builder) {
    let mut add = |name: &str, help: &str, run: Handler| {
        b.commands.insert(name.to_owned(), Command { help: help.to_owned(), run });
    };
    add(
        "hello",
        "the protocol version and the commands",
        Box::new(|_, ctx| {
            Ok(json!({
                "protocol": control::PROTOCOL,
                "vusb": env!("CARGO_PKG_VERSION"),
                "commands": ctx.help,
                "topics": ["tx", "violations", "events", "comments"],
            }))
        }),
    );
    add(
        "status",
        "where the device is and how it stands",
        Box::new(|_, ctx| Ok(status_command(ctx))),
    );
    add(
        "endpoints",
        "the active configuration's endpoints",
        Box::new(|_, ctx| Ok(endpoints_command(ctx))),
    );
    add(
        "reset",
        "a fresh model from the current options, attached again if its descriptors changed",
        Box::new(|_, ctx| ctx.reset().map(|()| json!({}))),
    );
    add(
        "options",
        "KEY=VALUE options for the resets that follow; none goes back to the first ones",
        Box::new(|args, ctx| {
            let map = args
                .as_object()
                .map(|m| m.iter().map(|(k, v)| (k.clone(), option_text(v))).collect::<Vec<_>>());
            let options = match map {
                Some(o) if !o.is_empty() => o,
                _ => ctx.initial_options.clone(),
            };
            // A bad option is refused now, not at the reset.
            (ctx.factory)(&options)?;
            ctx.options = options;
            Ok(json!({}))
        }),
    );
    add(
        "unplug",
        "the device leaves the bus, as a pulled cable does",
        Box::new(|_, ctx| ctx.unplug().map(|()| json!({}))),
    );
    add(
        "replug",
        "an unplugged device comes back",
        Box::new(|_, ctx| ctx.replug().map(|()| json!({}))),
    );
    add(
        "arm",
        "event=unplug or event=mark name=NAME, when=in=EP:N, out=EP:N or model: fires once there",
        Box::new(|args, ctx| {
            let arm = arm_command(args)?;
            ctx.bus().arm(arm);
            Ok(json!({}))
        }),
    );
    add(
        "section",
        "name=NAME: what follows belongs to it, in the log and on the stream",
        Box::new(|args, ctx| {
            let Some(name) = arg_str(args, "name") else {
                return Err(Error::Invalid("section takes name=NAME".into()));
            };
            ctx.section(name);
            Ok(json!({}))
        }),
    );
    add(
        "log",
        "file=NAME starts a log in the log directory, off=true stops it, verbose=BOOL, \
         format=plain|fixture, digest-above=N|off (fixture: longer data as its digest)",
        Box::new(log_command),
    );
    add(
        "subscribe",
        "topics: tx, violations, events, comments, streamed on this connection",
        Box::new(|args, ctx| subscribe_command(args, ctx, true)),
    );
    add(
        "unsubscribe",
        "stops those topics",
        Box::new(|args, ctx| subscribe_command(args, ctx, false)),
    );
    add(
        "sync",
        "answers after every input taken before it, a renumeration included",
        Box::new(|_, _| Ok(json!({}))),
    );
    add(
        "quit",
        "detaches the device and ends the session",
        Box::new(|_, ctx| {
            ctx.quit();
            Ok(json!({}))
        }),
    );
}

#[cfg(test)]
mod tests {
    use std::net::TcpStream;

    use super::*;
    use crate::control::Client;
    use crate::examples::relay;
    use crate::model::Setup;
    use crate::transport::usbip::{DIR_IN, DIR_OUT, cmd_submit, import, read_ret};

    fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index, length }
    }

    fn relay_factory() -> ModelFactory {
        Box::new(|options: &[(String, String)]| relay::build(options))
    }

    /// A session on loopback, running on its own thread, and a client.
    struct Running {
        client: Client,
        addr: std::net::SocketAddr,
        devid: u32,
        handle: JoinHandle<Result<Report>>,
    }

    fn start(builder: Builder) -> Running {
        let session =
            builder.control(Address::parse("127.0.0.1:0")).start().expect("session starts");
        let client = Client::connect(&session.control_addresses()[0]).unwrap();
        let (addr, devid) = (session.addr().expect("over USB/IP"), session.devid());
        Running { client, addr, devid, handle: thread::spawn(move || session.run()) }
    }

    impl Running {
        fn end(mut self) -> Report {
            self.client.call("quit", &json!({})).unwrap();
            self.handle.join().unwrap().unwrap()
        }

        /// Imports the device and configures it.
        fn device(&self) -> TcpStream {
            let (mut s, _) = import(self.addr, "1-1").unwrap();
            let set = Some(setup(0x00, 9, 1, 0, 0));
            s.write_all(&cmd_submit(1, self.devid, DIR_OUT, 0, 0, set, &[])).unwrap();
            assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
            s
        }

        fn feature(&self, s: &mut TcpStream, seq: u32) -> Vec<u8> {
            let get = Some(setup(0xa1, 1, 0x0300, 0, 8));
            s.write_all(&cmd_submit(seq, self.devid, DIR_IN, 0, 8, get, &[])).unwrap();
            read_ret(s, true).unwrap().data
        }
    }

    #[test]
    fn commands_answer_and_refuse() {
        let mut r = start(Builder::new(relay_factory()).command("ping", "answers", |args, ctx| {
            Ok(json!({"back": args["x"], "violations": ctx.violations()}))
        }));
        let hello = r.client.call("hello", &json!({})).unwrap();
        assert_eq!(hello["protocol"], control::PROTOCOL);
        let names: Vec<&str> = hello["commands"]
            .as_array()
            .unwrap()
            .iter()
            .map(|c| c["name"].as_str().unwrap())
            .collect();
        for want in ["arm", "reset", "subscribe", "sync", "ping"] {
            assert!(names.contains(&want), "{want} in {names:?}");
        }
        assert_eq!(r.client.call("ping", &json!({"x": 5})).unwrap()["back"], 5);
        let status = r.client.call("status", &json!({})).unwrap();
        assert_eq!(status["model"], relay::NAME);
        assert_eq!(status["attached"], Value::Null);
        assert_eq!(status["vendor"], 0x16c0);
        assert_eq!(status["section"], "attach");
        // Before SET_CONFIGURATION there are none.
        assert_eq!(r.client.call("endpoints", &json!({})).unwrap()["endpoints"], json!([]));
        for (cmd, args) in [
            ("nope", json!({})),
            ("arm", json!({})),
            ("arm", json!({"event": "explode", "when": "model"})),
            ("options", json!({"relays": 99})),
            ("section", json!({})),
            ("log", json!({"file": "x"})),
            ("log", json!({})),
            ("subscribe", json!({"topics": ["nope"]})),
        ] {
            assert!(r.client.call(cmd, &args).is_err(), "{cmd} {args}");
        }
        // A refused option leaves the session as it was.
        assert_eq!(r.client.call("sync", &json!({})).unwrap(), json!({}));
        assert_eq!(r.end(), Report { violations: 0, lost: false });
    }

    #[test]
    fn bad_lines_get_an_error_reply() {
        let session =
            Builder::new(relay_factory()).control(Address::parse("127.0.0.1:0")).start().unwrap();
        let Address::Tcp(a) = &session.control_addresses()[0] else { panic!() };
        let mut raw = TcpStream::connect(a).unwrap();
        let mut reader = BufReader::new(raw.try_clone().unwrap());
        let handle = thread::spawn(move || session.run());
        raw.write_all(b"not json\n\n{\"id\":3,\"cmd\":\"quit\"}\n").unwrap();
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();
        let v: Value = serde_json::from_str(&line).unwrap();
        assert_eq!((v["ok"].clone(), v["id"].clone()), (json!(false), Value::Null));
        line.clear();
        reader.read_line(&mut line).unwrap();
        let v: Value = serde_json::from_str(&line).unwrap();
        assert_eq!(
            (v["ok"].clone(), v["id"].clone()),
            (json!(true), json!(3)),
            "the blank is skipped"
        );
        handle.join().unwrap().unwrap();
    }

    #[test]
    fn options_and_reset_build_a_fresh_model() {
        let mut r = start(Builder::new(relay_factory()));
        let mut s = r.device();
        let set = Some(setup(0x21, 9, 0x0300, 0, 8));
        let on = [0xff, 1, 0, 0, 0, 0, 0, 0];
        s.write_all(&cmd_submit(2, r.devid, DIR_OUT, 0, 8, set, &on)).unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        assert_eq!(r.feature(&mut s, 3), b"ABCDE\0\0\x01");
        r.client.call("reset", &json!({})).unwrap();
        assert_eq!(r.feature(&mut s, 4), b"ABCDE\0\0\0", "relays off again");
        r.client.call("options", &json!({"serial": "QWERT"})).unwrap();
        r.client.call("reset", &json!({})).unwrap();
        assert_eq!(r.feature(&mut s, 5), b"QWERT\0\0\0");
        r.client.call("options", &json!({})).unwrap();
        r.client.call("reset", &json!({})).unwrap();
        assert_eq!(r.feature(&mut s, 6), b"ABCDE\0\0\0", "no options: the first ones");
        // The endpoint is there now.
        let e = r.client.call("endpoints", &json!({})).unwrap();
        assert_eq!(e["configuration"], 1);
        assert_eq!(e["endpoints"][0]["address"], 0x81);
        r.end();
    }

    #[test]
    fn subscribers_see_the_stream_in_order_with_sections() {
        let mut r = start(Builder::new(relay_factory()));
        r.client.call("subscribe", &json!({"topics": ["tx", "violations", "events"]})).unwrap();
        r.client.call("section", &json!({"name": "run"})).unwrap();
        let mut s = r.device();
        assert_eq!(r.feature(&mut s, 2), b"ABCDE\0\0\0");
        // A transfer to an endpoint the configuration lacks: a violation.
        s.write_all(&cmd_submit(3, r.devid, DIR_IN, 5, 8, None, &[])).unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().status, -32);
        r.client.call("unsubscribe", &json!({"topics": ["tx"]})).unwrap();
        assert_eq!(r.feature(&mut s, 4), b"ABCDE\0\0\0");
        r.client.call("sync", &json!({})).unwrap();
        let mut seen = Vec::new();
        for _ in 0..5 {
            seen.push(r.client.next_event().unwrap().unwrap());
        }
        assert_eq!(
            (seen[0]["event"].clone(), seen[0]["name"].clone()),
            (json!("section"), json!("run"))
        );
        assert_eq!(seen[1]["event"], "tx");
        assert_eq!(seen[1]["setup"]["request"], 9);
        assert_eq!(seen[2]["reply"]["data"], "4142434445000000");
        assert_eq!(seen[2]["section"], "run");
        assert_eq!(seen[2]["line"], "control a1 01 0300 0000 0008 -> 4142434445000000");
        assert_eq!(seen[3]["event"], "violation");
        assert_eq!(seen[3]["section"], "run");
        assert_eq!(seen[4]["event"], "tx", "the bad transfer");
        assert_eq!(seen[4]["reply"], "stall");
        // The missing endpoint is the only one: the device was configured.
        assert_eq!(r.end().violations, 1);
    }

    #[test]
    fn an_armed_unplug_and_a_replug_without_attaching() {
        let mut r = start(Builder::new(relay_factory()));
        r.client.call("subscribe", &json!({"topics": ["events"]})).unwrap();
        r.client.call("arm", &json!({"event": "unplug", "when": "in=0:1"})).unwrap();
        let mut s = r.device();
        assert_eq!(r.feature(&mut s, 2), b"ABCDE\0\0\0");
        // sync: the signal is in the queue before anything we send now.
        r.client.call("sync", &json!({})).unwrap();
        let fired = r.client.next_event().unwrap().unwrap();
        assert_eq!(
            (fired["event"].clone(), fired["name"].clone()),
            (json!("fired"), json!("unplug"))
        );
        assert_eq!(r.client.call("status", &json!({})).unwrap()["unplugged"], true);
        // Gone: the next URB gets no answer. A client sees it by a read
        // timeout, as a driver's URB that never comes back.
        s.set_read_timeout(Some(Duration::from_millis(100))).unwrap();
        let get = Some(setup(0xa1, 1, 0x0300, 0, 8));
        s.write_all(&cmd_submit(3, r.devid, DIR_IN, 0, 8, get, &[])).unwrap();
        assert!(read_ret(&mut s, true).is_err());
        s.set_read_timeout(None).unwrap();
        r.client.call("replug", &json!({})).unwrap();
        assert_eq!(r.client.next_event().unwrap().unwrap()["event"], "section");
        assert_eq!(r.client.next_event().unwrap().unwrap()["event"], "replugged");
        assert_eq!(r.client.call("status", &json!({})).unwrap()["unplugged"], false);
        assert_eq!(r.feature(&mut s, 4), b"ABCDE\0\0\0");
        // By command, and a mark does nothing to the device.
        r.client.call("unplug", &json!({})).unwrap();
        assert_eq!(r.client.next_event().unwrap().unwrap()["event"], "unplugged");
        r.client.call("reset", &json!({})).unwrap();
        assert_eq!(
            r.client.call("status", &json!({})).unwrap()["unplugged"],
            false,
            "reset replugs"
        );
        r.client.call("arm", &json!({"event": "mark", "name": "snap", "when": "in=0:1"})).unwrap();
        assert_eq!(r.feature(&mut s, 5), b"ABCDE\0\0\0");
        r.client.call("sync", &json!({})).unwrap();
        loop {
            let e = r.client.next_event().unwrap().unwrap();
            if e["event"] == "fired" {
                assert_eq!(e["name"], "snap");
                break;
            }
        }
        assert_eq!(r.feature(&mut s, 6), b"ABCDE\0\0\0");
        r.end();
    }

    #[test]
    fn sync_comes_after_a_renumeration_in_flight() {
        use crate::descriptor::{self, DeviceDescriptor};
        use crate::model::{Descriptors, Violations};
        /// Asks to renumerate at its first vendor write.
        struct Renumerates(Descriptors, bool);
        impl DeviceModel for Renumerates {
            fn name(&self) -> &'static str {
                "renumerates"
            }
            fn descriptors(&self) -> &Descriptors {
                &self.0
            }
            fn control(&mut self, _: Setup, _: &[u8], _: &mut Violations) -> Reply {
                self.1 = true;
                Reply::ack()
            }
            fn out(&mut self, _: u8, _: &[u8], _: &mut Violations) -> Reply {
                Reply::ack()
            }
            fn in_transfer(&mut self, _: u8, _: usize, _: &mut Violations) -> Reply {
                Reply::Nak
            }
            fn reset(&mut self) {}
            fn take_renumeration(&mut self) -> bool {
                std::mem::take(&mut self.1)
            }
        }
        let factory: ModelFactory = Box::new(|_| {
            let device = DeviceDescriptor {
                usb: 0x0200,
                class: [0, 0, 0],
                max_packet0: 64,
                vendor: 0x1234,
                product: 0x5678,
                device: 0x0100,
                strings: [0, 0, 0],
                configurations: 1,
            };
            Ok(Box::new(Renumerates(
                Descriptors {
                    speed: Speed::Full,
                    device: device.bytes(),
                    configs: vec![descriptor::configuration(1, 0, 0x80, 50, &[])],
                    strings: Vec::new(),
                },
                false,
            )))
        });
        let mut r = start(Builder::new(factory));
        r.client.call("subscribe", &json!({"topics": ["events"]})).unwrap();
        r.client.call("section", &json!({"name": "up"})).unwrap();
        let (mut s, _) = import(r.addr, "1-1").unwrap();
        let vendor = Some(setup(0x40, 0xa0, 0, 0, 0));
        s.write_all(&cmd_submit(1, r.devid, DIR_OUT, 0, 0, vendor, &[])).unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        r.client.call("sync", &json!({})).unwrap();
        assert_eq!(r.client.next_event().unwrap().unwrap()["event"], "section");
        let e = r.client.next_event().unwrap().unwrap();
        assert_eq!(e["event"], "renumeration");
        assert_eq!((e["vendor"].clone(), e["section"].clone()), (json!(0x1234), json!("up")));
        r.end();
    }

    #[test]
    fn the_log_goes_where_it_is_allowed_to() {
        let dir = std::env::temp_dir().join(format!("vusb-session-log-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let first = dir.join("first.log");
        let mut r = start(
            Builder::new(relay_factory()).log(Some(first.clone())).log_dir(Some(dir.clone())),
        );
        let mut s = r.device();
        r.client.call("section", &json!({"name": "one"})).unwrap();
        assert_eq!(r.feature(&mut s, 2), b"ABCDE\0\0\0");
        for bad in ["../x", "/etc/x", "a/b", ".hidden", ""] {
            assert!(r.client.call("log", &json!({"file": bad})).is_err(), "{bad}");
        }
        std::fs::write(dir.join("taken"), b"keep").unwrap();
        assert!(r.client.call("log", &json!({"file": "taken"})).is_err(), "never over a file");
        let path = r.client.call("log", &json!({"file": "second.log"})).unwrap();
        assert!(path["path"].as_str().unwrap().ends_with("second.log"));
        assert_eq!(r.feature(&mut s, 3), b"ABCDE\0\0\0");
        r.client.call("log", &json!({"off": true})).unwrap();
        assert_eq!(r.feature(&mut s, 4), b"ABCDE\0\0\0");
        r.end();
        let log = std::fs::read_to_string(&first).unwrap();
        assert!(log.starts_with("# section attach\n"), "{log}");
        assert!(
            log.contains("# section one\ncontrol a1 01 0300 0000 0008 -> 4142434445000000\n"),
            "{log}"
        );
        let second = std::fs::read_to_string(dir.join("second.log")).unwrap();
        assert_eq!(second, "control a1 01 0300 0000 0008 -> 4142434445000000\n");
        assert_eq!(std::fs::read(dir.join("taken")).unwrap(), b"keep");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn the_log_is_in_the_format_asked_for() {
        let dir = std::env::temp_dir().join(format!("vusb-session-format-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("fixture.log");
        let mut r = start(
            Builder::new(relay_factory())
                .log(Some(path.clone()))
                .log_format(LogFormat::Fixture)
                .log_rules(Rules::digest_above(16)),
        );
        let mut s = r.device();
        r.client.call("section", &json!({"name": "one"})).unwrap();
        r.feature(&mut s, 2);
        // The rules and the format change while it runs.
        r.client.call("log", &json!({"digest-above": 4})).unwrap();
        r.feature(&mut s, 3);
        r.client.call("log", &json!({"format": "plain"})).unwrap();
        r.feature(&mut s, 4);
        r.client.call("log", &json!({"format": "fixture", "digest-above": "off"})).unwrap();
        r.feature(&mut s, 5);
        // What is refused changes nothing, not even the half it could.
        for bad in [
            json!({"format": "golden"}),
            json!({"format": "plain", "digest-above": "many"}),
            json!({"digest-above": -1}),
        ] {
            assert!(r.client.call("log", &bad).is_err(), "{bad}");
        }
        r.feature(&mut s, 6);
        r.end();
        let log = std::fs::read_to_string(&path).unwrap();
        assert_eq!(
            log,
            "# section attach\n\
             ctl_out 00 09 0001 0000 - -> ok\n\
             # section one\n\
             ctl_in 21 01 0300 0000 8 -> 4142434445000000\n\
             ctl_in 21 01 0300 0000 8 -> \
             @8:c300361ae74b2fae5864684afc8dab8a19e4a431b02b93457424ffba180a0955\n\
             control a1 01 0300 0000 0008 -> 4142434445000000\n\
             ctl_in 21 01 0300 0000 8 -> 4142434445000000\n\
             ctl_in 21 01 0300 0000 8 -> 4142434445000000\n"
        );
        std::fs::remove_file(&path).unwrap();
        std::fs::remove_dir(&dir).unwrap();
        assert_eq!("fixture".parse::<LogFormat>().unwrap(), LogFormat::Fixture);
        assert_eq!(LogFormat::default().to_string(), "plain");
        assert!("Fixture".parse::<LogFormat>().is_err());
    }

    #[test]
    fn a_stopper_ends_the_session_and_several_clients_are_served() {
        let session =
            Builder::new(relay_factory()).control(Address::parse("127.0.0.1:0")).start().unwrap();
        let stopper = session.stopper();
        let address = session.control_addresses()[0].clone();
        let handle = thread::spawn(move || session.run());
        let mut a = Client::connect(&address).unwrap();
        let mut b = Client::connect(&address).unwrap();
        a.call("section", &json!({"name": "from-a"})).unwrap();
        assert_eq!(b.call("status", &json!({})).unwrap()["section"], "from-a");
        drop(a);
        assert_eq!(b.call("sync", &json!({})).unwrap(), json!({}));
        assert_eq!(stopper.finish().violations, 0);
        handle.join().unwrap().unwrap();
    }

    #[test]
    fn a_session_over_usbredir_is_driven_the_same_way() {
        use crate::transport::usbredir::{Guest, caps, kinds, statuses};
        let session = Builder::new(relay_factory())
            .usbredir(Address::parse("127.0.0.1:0"))
            .control(Address::parse("127.0.0.1:0"))
            .start()
            .unwrap();
        assert_eq!(session.addr(), None, "no USB/IP server");
        let redir = session.usbredir_address().unwrap().clone();
        let mut client = Client::connect(&session.control_addresses()[0]).unwrap();
        let handle = thread::spawn(move || session.run());
        client.call("subscribe", &json!({"topics": ["events"]})).unwrap();
        let status = client.call("status", &json!({})).unwrap();
        assert_eq!(
            (status["transport"].clone(), status["guest"].clone()),
            (json!("usbredir"), json!(false))
        );

        let mut guest = Guest::connect(&redir, caps::QEMU).unwrap();
        guest.device().unwrap();
        assert_eq!(guest.set_configuration(1, 1).unwrap(), (statuses::SUCCESS, 1));
        let report = setup(0xa1, 1, 0x0300, 0, 8);
        assert_eq!(guest.control(2, report, &[]).unwrap().1, b"ABCDE\0\0\0");
        client.call("sync", &json!({})).unwrap();
        assert_eq!(client.next_event().unwrap().unwrap()["event"], "ready");
        assert_eq!(client.call("status", &json!({})).unwrap()["guest"], true);

        // Unplug by command: the guest loses the device; replug brings
        // it back on the same connection.
        client.call("unplug", &json!({})).unwrap();
        assert_eq!(guest.read().unwrap().kind, kinds::DEVICE_DISCONNECT);
        guest.disconnect_ack().unwrap();
        client.call("replug", &json!({})).unwrap();
        guest.device().unwrap();
        assert_eq!(guest.set_configuration(3, 1).unwrap(), (statuses::SUCCESS, 1));

        // New options change the descriptors: the reset re-announces.
        client.call("options", &json!({"relays": 4})).unwrap();
        client.call("reset", &json!({})).unwrap();
        assert_eq!(guest.read().unwrap().kind, kinds::DEVICE_DISCONNECT);
        guest.disconnect_ack().unwrap();
        guest.device().unwrap();
        let (_, d) = guest.control(4, setup(0x80, 6, 0x0302, 0x0409, 255), &[]).unwrap();
        let name: Vec<u16> = d[2..].chunks(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        assert_eq!(String::from_utf16_lossy(&name), "USBRelay4");

        // An armed unplug fires from the guest's traffic.
        client.call("arm", &json!({"event": "unplug", "when": "in=0:1"})).unwrap();
        assert_eq!(guest.control(5, report, &[]).unwrap().0, statuses::SUCCESS);
        assert_eq!(guest.read().unwrap().kind, kinds::DEVICE_DISCONNECT);
        client.call("sync", &json!({})).unwrap();
        assert_eq!(client.call("status", &json!({})).unwrap()["unplugged"], true);
        let names: Vec<String> = std::iter::from_fn(|| {
            let e = client.next_event().ok()??;
            let name = e["event"].as_str()?.to_owned();
            (name != "fired").then_some(name).or(None)
        })
        .collect();
        assert!(names.contains(&"unplugged".to_owned()), "{names:?}");
        assert!(names.contains(&"renumerated".to_owned()), "{names:?}");

        client.call("quit", &json!({})).unwrap();
        assert_eq!(handle.join().unwrap().unwrap().violations, 0);
        // Not both at once.
        let both =
            Builder::new(relay_factory()).attach(true).usbredir(Address::parse("127.0.0.1:0"));
        assert!(both.start().is_err());
    }
}
