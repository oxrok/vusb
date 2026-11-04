// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Events armed to fire at a point of the host's traffic: fault
//! injection by count rather than by time.
//!
//! An [`Arm`] names an [`Event`] and a point ([`When`]): the Nth IN
//! transfer completed on an endpoint, the Nth OUT transfer taken, or
//! the moment the model marks ([`crate::model::DeviceModel::mark`]).
//! A host counts the transfers it offers and, when the point is
//! reached, fires the event once: an unplug makes the device leave the
//! bus; a mark does nothing to the device and is passed to whoever
//! drives the run, who decides what it means (interrupt the tool under
//! test, take a snapshot, stop).

use crate::model::Reply;
use crate::{Error, Result};

/// What happens when an armed point is reached.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Event {
    /// The device leaves the bus, as a pulled cable does, until it is
    /// plugged back.
    Unplug,
    /// Nothing happens to the device; the name is passed on to whoever
    /// drives the run.
    Mark(String),
}

impl Event {
    /// The word of the specification and of a log line: `unplug`, or
    /// the mark's name.
    pub fn word(&self) -> &str {
        match self {
            Event::Unplug => "unplug",
            Event::Mark(name) => name,
        }
    }
}

/// The point at which an armed [`Event`] fires.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum When {
    /// Once the `n`th IN transfer on endpoint number `ep` has completed
    /// with data (a STALL counts as completed, a NAK does not; endpoint
    /// 0 counts the control reads).
    In {
        /// The endpoint number.
        ep: u8,
        /// The count, from 1.
        n: u32,
    },
    /// Once the `n`th OUT transfer to endpoint number `ep` was taken
    /// (endpoint 0 counts the control writes).
    Out {
        /// The endpoint number.
        ep: u8,
        /// The count, from 1.
        n: u32,
    },
    /// When the model marks its point ([`crate::model::DeviceModel::mark`]).
    Model,
}

/// An armed event: `unplug WHEN` or `mark NAME WHEN`, with `WHEN` one
/// of `in=EP:N`, `out=EP:N` or `model`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Arm {
    /// What happens.
    pub event: Event,
    /// When.
    pub when: When,
    /// Transfers counted so far.
    seen: u32,
}

impl Arm {
    /// An event armed for a point.
    pub fn new(event: Event, when: When) -> Arm {
        Arm { event, when, seen: 0 }
    }

    /// Parses `unplug WHEN` or `mark NAME WHEN`.
    ///
    /// # Errors
    ///
    /// On an unknown event or point, or a count of 0.
    pub fn parse(text: &str) -> Result<Arm> {
        const USAGE: &str =
            "an armed event is `unplug WHEN` or `mark NAME WHEN`, WHEN in=EP:N, out=EP:N or model";
        let words: Vec<&str> = text.split_whitespace().collect();
        let (event, when) = match words[..] {
            ["unplug", when] => (Event::Unplug, when),
            ["mark", name, when] => (Event::Mark(name.to_owned()), when),
            _ => return Err(Error::Invalid(USAGE.to_owned())),
        };
        let count = |spec: &str| -> Result<(u8, u32)> {
            let (ep, n) =
                spec.split_once(':').ok_or_else(|| Error::Invalid(format!("{spec}: EP:N")))?;
            let ep: u8 = ep
                .parse()
                .ok()
                .filter(|e| *e < 16)
                .ok_or_else(|| Error::Invalid(format!("{spec}: EP 0 to 15")))?;
            let n: u32 = n
                .parse()
                .ok()
                .filter(|n| *n > 0)
                .ok_or_else(|| Error::Invalid(format!("{spec}: N from 1")))?;
            Ok((ep, n))
        };
        let when = if when == "model" {
            When::Model
        } else if let Some(spec) = when.strip_prefix("in=") {
            let (ep, n) = count(spec)?;
            When::In { ep, n }
        } else if let Some(spec) = when.strip_prefix("out=") {
            let (ep, n) = count(spec)?;
            When::Out { ep, n }
        } else {
            return Err(Error::Invalid(format!("no point {when}: in=EP:N, out=EP:N or model")));
        };
        Ok(Arm::new(event, when))
    }

    /// Counts a transfer offered to the model (`ep` without the
    /// direction bit, `reply` its answer) and whether the model marked
    /// its point; whether the event fires now.
    pub fn fires(&mut self, ep: u8, is_in: bool, reply: &Reply, model_point: bool) -> bool {
        let (want_ep, want_in, n) = match self.when {
            When::Model => return model_point,
            When::In { ep, n } => (ep, true, n),
            When::Out { ep, n } => (ep, false, n),
        };
        if ep != want_ep || is_in != want_in || *reply == Reply::Nak {
            return false;
        }
        self.seen += 1;
        self.seen == n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_events_and_points() {
        assert_eq!(
            Arm::parse("unplug in=1:3").unwrap(),
            Arm::new(Event::Unplug, When::In { ep: 1, n: 3 })
        );
        assert_eq!(
            Arm::parse("mark interrupt model").unwrap(),
            Arm::new(Event::Mark("interrupt".into()), When::Model)
        );
        assert_eq!(
            Arm::parse("unplug out=2:1").unwrap(),
            Arm::new(Event::Unplug, When::Out { ep: 2, n: 1 })
        );
        for bad in [
            "",
            "unplug",
            "mark model",
            "explode model",
            "unplug in=1:0",
            "unplug in=16:1",
            "unplug at=1",
            "unplug in=1",
        ] {
            assert!(Arm::parse(bad).is_err(), "{bad}");
        }
        assert_eq!(Event::Mark("stop".into()).word(), "stop");
        assert_eq!(Event::Unplug.word(), "unplug");
    }

    #[test]
    fn counts_completed_transfers_of_the_endpoint() {
        let data = Reply::Data(vec![1]);
        let mut arm = Arm::parse("unplug in=1:2").unwrap();
        assert!(!arm.fires(1, true, &Reply::Nak, false), "a NAK does not count");
        assert!(!arm.fires(1, false, &Reply::ack(), false), "nor does an OUT");
        assert!(!arm.fires(2, true, &data, false), "nor another endpoint");
        assert!(!arm.fires(1, true, &data, false));
        assert!(arm.fires(1, true, &Reply::Stall, false), "a STALL completes");
        let mut arm = Arm::parse("mark m model").unwrap();
        assert!(!arm.fires(1, true, &data, false));
        assert!(arm.fires(0, false, &Reply::Nak, true), "the model's point, whatever the transfer");
    }
}
