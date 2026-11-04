// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Parsing the `key=value` options a model is built with.
//!
//! A model that takes options (the relay count of a board, a fault to
//! inject) gets them as `(key, value)` pairs, from a command line or a
//! control connection, and refuses what it does not know. The shells'
//! `Faults::option` methods take their own keys the same way, so a model
//! offers them first and keeps the rest.

use crate::{Error, Result};

/// Splits `KEY=VALUE` words into pairs.
///
/// # Errors
///
/// On a word without `=`, or with an empty key.
pub fn parse(words: &[String]) -> Result<Vec<(String, String)>> {
    words
        .iter()
        .map(|w| match w.split_once('=') {
            Some((k, v)) if !k.is_empty() => Ok((k.to_owned(), v.to_owned())),
            _ => Err(Error::Invalid(format!("{w}: options are KEY=VALUE"))),
        })
        .collect()
}

/// Parses a decimal option within `range`.
///
/// # Errors
///
/// When it is not a number in the range.
pub fn parse_number(key: &str, value: &str, range: std::ops::RangeInclusive<u32>) -> Result<u32> {
    value.parse().ok().filter(|n| range.contains(n)).ok_or_else(|| {
        Error::Invalid(format!("{key}={value}: {} to {}", range.start(), range.end()))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_pairs_and_checks_ranges() {
        let words = vec!["relays=2".to_owned(), "serial=0x41".to_owned()];
        assert_eq!(
            parse(&words).unwrap(),
            vec![("relays".to_owned(), "2".to_owned()), ("serial".to_owned(), "0x41".to_owned())]
        );
        assert!(parse(&["relays".to_owned()]).is_err());
        assert!(parse(&["=2".to_owned()]).is_err());
        assert_eq!(parse_number("n", "5", 1..=8).unwrap(), 5);
        assert!(parse_number("n", "9", 1..=8).is_err());
        assert!(parse_number("n", "x", 1..=8).is_err());
    }
}
