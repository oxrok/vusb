// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The sockets a connection comes in on: a control connection, or a
//! usbredir guest.

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};
#[cfg(unix)]
use std::path::Path;
use std::path::PathBuf;
use std::time::Duration;

use crate::{Error, Result};

/// How long a write to a client that stopped reading may take.
const WRITE_TIMEOUT: Duration = Duration::from_secs(5);

/// Where a socket is.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Address {
    /// A Unix socket's path.
    Unix(PathBuf),
    /// `host:port`.
    Tcp(String),
}

impl Address {
    /// `host:port` when it ends in a port number and has no `/`, a Unix
    /// socket's path otherwise.
    pub fn parse(text: &str) -> Address {
        match text.rsplit_once(':') {
            Some((host, port))
                if !host.is_empty() && !text.contains('/') && port.parse::<u16>().is_ok() =>
            {
                Address::Tcp(text.to_owned())
            }
            _ => Address::Unix(PathBuf::from(text)),
        }
    }
}

impl std::fmt::Display for Address {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Address::Unix(p) => write!(f, "{}", p.display()),
            Address::Tcp(a) => f.write_str(a),
        }
    }
}

/// How a socket is opened.
#[derive(Clone, Debug, Default)]
pub struct ListenOptions {
    /// Who may connect to a Unix socket: its owner and group. By default
    /// `SUDO_UID` and `SUDO_GID` when set, so that a server run under
    /// `sudo` answers the user who ran it.
    pub owner: Option<(u32, u32)>,
    /// Takes a TCP address that is not loopback. The protocol has no
    /// authentication: anyone who reaches the port controls the device.
    pub allow_remote: bool,
}

/// The reading half of an accepted connection.
pub type Reader = Box<dyn Read + Send>;
/// The writing half.
pub type Writer = Box<dyn Write + Send>;
/// An accepted connection, both halves.
pub type Accepted = (Reader, Writer);

/// A socket bound on an [`Address`], accepting connections.
pub enum Listener {
    /// On a TCP port.
    Tcp(TcpListener),
    /// On a Unix socket, removed when the listener is dropped.
    #[cfg(unix)]
    Unix(std::os::unix::net::UnixListener, PathBuf),
}

impl Listener {
    /// Binds. A TCP port 0 is resolved, see [`Listener::address`].
    ///
    /// # Errors
    ///
    /// When the address is refused (not loopback, without
    /// `allow_remote`), the path is in use, or binding fails.
    pub fn bind(address: &Address, options: &ListenOptions) -> Result<Listener> {
        let listener = match address {
            Address::Tcp(a) => {
                let addrs: Vec<_> = a.to_socket_addrs()?.collect();
                if !options.allow_remote && addrs.iter().any(|s| !s.ip().is_loopback()) {
                    return Err(Error::Invalid(format!(
                        "control on {a}: not loopback, and the protocol has no authentication; \
                         allow it explicitly if the network is yours"
                    )));
                }
                Listener::Tcp(TcpListener::bind(&addrs[..])?)
            }
            #[cfg(unix)]
            Address::Unix(path) => bind_unix(path, options)?,
            #[cfg(not(unix))]
            Address::Unix(path) => {
                return Err(Error::Invalid(format!(
                    "control on {}: Unix sockets need Unix; use host:port",
                    path.display()
                )));
            }
        };
        match &listener {
            Listener::Tcp(l) => l.set_nonblocking(true)?,
            #[cfg(unix)]
            Listener::Unix(l, _) => l.set_nonblocking(true)?,
        }
        Ok(listener)
    }

    /// Where it ended up: a TCP port 0 is resolved.
    pub fn address(&self) -> io::Result<Address> {
        match self {
            Listener::Tcp(l) => Ok(Address::Tcp(l.local_addr()?.to_string())),
            #[cfg(unix)]
            Listener::Unix(_, path) => Ok(Address::Unix(path.clone())),
        }
    }

    /// A connection, or `None` when there is none now.
    ///
    /// # Errors
    ///
    /// When accepting fails for another reason than nobody being there.
    pub fn accept(&self) -> io::Result<Option<Accepted>> {
        let accepted = match self {
            Listener::Tcp(l) => l.accept().and_then(|(s, _)| tcp_pair(s)),
            #[cfg(unix)]
            Listener::Unix(l, _) => l.accept().and_then(|(s, _)| {
                s.set_nonblocking(false)?;
                s.set_write_timeout(Some(WRITE_TIMEOUT))?;
                let r = s.try_clone()?;
                Ok((Box::new(r) as Reader, Box::new(s) as Writer))
            }),
        };
        match accepted {
            Ok(a) => Ok(Some(a)),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(e),
        }
    }
}

fn tcp_pair(s: TcpStream) -> io::Result<Accepted> {
    s.set_nonblocking(false)?;
    s.set_nodelay(true)?;
    s.set_write_timeout(Some(WRITE_TIMEOUT))?;
    let r = s.try_clone()?;
    Ok((Box::new(r), Box::new(s)))
}

impl std::fmt::Debug for Listener {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.address() {
            Ok(a) => write!(f, "Listener({a})"),
            Err(e) => write!(f, "Listener({e})"),
        }
    }
}

#[cfg(unix)]
impl Drop for Listener {
    fn drop(&mut self) {
        if let Listener::Unix(_, path) = self {
            let _ = std::fs::remove_file(path);
        }
    }
}

/// The owner a socket gets: asked for, or who ran `sudo`.
#[cfg(unix)]
fn owner(options: &ListenOptions) -> Option<(u32, u32)> {
    options.owner.or_else(|| {
        let id = |name| std::env::var(name).ok()?.parse().ok();
        Some((id("SUDO_UID")?, id("SUDO_GID")?))
    })
}

/// Makes way for a socket at `path`: a stale socket goes, one that
/// answers or anything that is not a socket is refused.
#[cfg(unix)]
fn clear_stale(path: &Path) -> Result<()> {
    use std::os::unix::fs::FileTypeExt;
    use std::os::unix::net::UnixStream;
    let Ok(meta) = std::fs::symlink_metadata(path) else { return Ok(()) };
    if !meta.file_type().is_socket() {
        return Err(Error::Invalid(format!("{}: exists and is not a socket", path.display())));
    }
    if UnixStream::connect(path).is_ok() {
        return Err(Error::Invalid(format!("{}: a server is listening there", path.display())));
    }
    std::fs::remove_file(path)?;
    Ok(())
}

/// Binds in a private directory and moves the socket into place once it
/// has its owner and mode, so nobody connects in between.
#[cfg(unix)]
fn bind_unix(path: &Path, options: &ListenOptions) -> Result<Listener> {
    use std::os::unix::fs::{DirBuilderExt, PermissionsExt, chown};
    // A client could never connect to one too long, wherever it was bound.
    std::os::unix::net::SocketAddr::from_pathname(path)
        .map_err(|e| Error::Invalid(format!("control on {}: {e}", path.display())))?;
    clear_stale(path)?;
    // Next to the socket, with a short name: a socket's path has a low
    // limit (about 100 bytes), and this one must fit too.
    let parent = path.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
    let private = parent.join(format!(".v{}", std::process::id()));
    std::fs::DirBuilder::new().mode(0o700).create(&private)?;
    let inner = private.join("s");
    let bound = (|| -> Result<std::os::unix::net::UnixListener> {
        let listener = std::os::unix::net::UnixListener::bind(&inner)
            .map_err(|e| Error::Invalid(format!("control on {}: {e}", path.display())))?;
        let owner = owner(options);
        let mode = if owner.is_some() { 0o660 } else { 0o600 };
        std::fs::set_permissions(&inner, std::fs::Permissions::from_mode(mode))?;
        if let Some((uid, gid)) = owner {
            match chown(&inner, Some(uid), Some(gid)) {
                Ok(()) => {}
                // Only root may give a file away; not root under an
                // inherited SUDO_UID, the user's own file will do.
                Err(e)
                    if options.owner.is_none() && e.kind() == io::ErrorKind::PermissionDenied => {}
                Err(e) => return Err(e.into()),
            }
        }
        std::fs::rename(&inner, path)?;
        Ok(listener)
    })();
    let _ = std::fs::remove_file(&inner);
    let _ = std::fs::remove_dir(&private);
    Ok(Listener::Unix(bound?, path.to_owned()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tells_addresses_apart() {
        assert_eq!(Address::parse("127.0.0.1:9"), Address::Tcp("127.0.0.1:9".into()));
        assert_eq!(Address::parse("localhost:9"), Address::Tcp("localhost:9".into()));
        assert_eq!(Address::parse("[::1]:9"), Address::Tcp("[::1]:9".into()));
        assert_eq!(Address::parse("/tmp/a:9"), Address::Unix("/tmp/a:9".into()));
        assert_eq!(Address::parse("v.sock"), Address::Unix("v.sock".into()));
        assert_eq!(Address::parse("v:sock"), Address::Unix("v:sock".into()));
    }

    #[test]
    fn refuses_remote_addresses_unless_asked() {
        let opts = ListenOptions::default();
        assert!(Listener::bind(&Address::Tcp("0.0.0.0:0".into()), &opts).is_err());
        assert!(Listener::bind(&Address::Tcp("127.0.0.1:0".into()), &opts).is_ok());
        let opts = ListenOptions { allow_remote: true, ..opts };
        assert!(Listener::bind(&Address::Tcp("0.0.0.0:0".into()), &opts).is_ok());
    }

    #[cfg(unix)]
    #[test]
    fn a_unix_socket_gets_its_mode_and_replaces_only_a_stale_one() {
        use std::os::unix::fs::PermissionsExt;
        let dir = std::env::temp_dir().join(format!("vusb-listener-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("c");
        let address = Address::Unix(path.clone());
        let opts = ListenOptions { owner: None, allow_remote: false };
        // Whoever runs the tests may have SUDO_UID set; the mode says so.
        let want = if owner(&opts).is_some() { 0o660 } else { 0o600 };
        let listener = Listener::bind(&address, &opts).unwrap();
        let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, want);
        assert!(Listener::bind(&address, &opts).is_err(), "a live server stays");
        // A stale socket: its file outlives the process, as after a crash.
        std::mem::forget(listener);
        let stale = std::os::unix::net::UnixListener::bind(dir.join("old")).unwrap();
        drop(stale);
        let old = Address::Unix(dir.join("old"));
        drop(Listener::bind(&old, &opts).unwrap());
        assert!(!dir.join("old").exists(), "removed on the way out");
        // As long a path as fits, near the limit, and a bare file name's
        // parent is handled too.
        let long = dir.join("l".repeat(100usize.saturating_sub(dir.as_os_str().len() + 1).max(1)));
        drop(Listener::bind(&Address::Unix(long), &opts).unwrap());
        let too_long = dir.join("l".repeat(200));
        let e = Listener::bind(&Address::Unix(too_long), &opts).err().unwrap();
        assert!(e.to_string().contains("control on"), "{e}");
        std::fs::write(dir.join("file"), b"keep").unwrap();
        assert!(Listener::bind(&Address::Unix(dir.join("file")), &opts).is_err());
        assert_eq!(std::fs::read(dir.join("file")).unwrap(), b"keep");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
