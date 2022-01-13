use std::collections::BTreeMap;
use std::fmt::Display;
use std::io::{self, BufRead};

use thiserror::Error;

#[derive(Debug, PartialEq, PartialOrd, Clone)]
pub enum GCodeOperation {
    Nop,
    Move {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        e: Option<f64>,
        f: Option<f64>,
    },
    Traditional {
        letter: char,
        code: u16,
        params: GCodeTraditionalParams,
    },
    Extended {
        command: String,
        params: GCodeExtendedParams,
    },
}

impl GCodeOperation {
    pub fn is_nop(&self) -> bool {
        matches!(self, GCodeOperation::Nop)
    }
}

impl Display for GCodeOperation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GCodeOperation::Nop => Ok(()),
            GCodeOperation::Move { x, y, z, e, f: f_ } => {
                write!(f, "G1")?;
                if let Some(x) = x {
                    write!(f, " X{}", x)?;
                }
                if let Some(v) = y {
                    write!(f, " Y{}", v)?;
                }
                if let Some(v) = z {
                    write!(f, " Z{}", v)?;
                }
                if let Some(v) = e {
                    write!(f, " E{}", v)?;
                }
                if let Some(v) = f_ {
                    write!(f, " F{}", v)?;
                }
                Ok(())
            }
            GCodeOperation::Traditional {
                letter,
                code,
                params,
            } => {
                write!(f, "{}{}", letter, code)?;
                if params.len() > 0 {
                    write!(f, " ")?;
                    params.fmt(f)?;
                }
                Ok(())
            }
            GCodeOperation::Extended {
                command: cmd,
                params,
            } => {
                write!(f, "{}", cmd)?;
                if params.len() > 0 {
                    write!(f, " ")?;
                    params.fmt(f)?;
                }
                Ok(())
            }
        }
    }
}

#[derive(Debug, PartialEq, PartialOrd, Clone)]
pub struct GCodeTraditionalParams(Vec<(char, String)>);

impl GCodeTraditionalParams {
    pub fn from_vec(vec: Vec<(char, String)>) -> Self {
        Self(vec)
    }

    pub fn get_string(&self, key: char) -> Option<&str> {
        self.0.iter().find(|(c, _)| *c == key).map(|v| v.1.as_str())
    }

    pub fn get_number<T: lexical_core::FromLexical>(&self, key: char) -> Option<T> {
        self.get_string(key)
            .map(|v| lexical_core::parse(v.as_bytes()).ok())
            .flatten()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }
}

impl Display for GCodeTraditionalParams {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut first = true;
        for (c, v) in &self.0 {
            if !first {
                write!(f, " ")?;
            }
            first = false;
            write!(f, "{}{}", c, v)?;
        }
        Ok(())
    }
}

#[derive(Debug, PartialEq, PartialOrd, Clone)]
pub struct GCodeExtendedParams(BTreeMap<String, String>);

impl GCodeExtendedParams {
    pub fn get_string(&self, key: &str) -> Option<&str> {
        self.0.get(key).map(|s| s.as_str())
    }

    pub fn get_number<T: lexical_core::FromLexical>(&self, key: &str) -> Option<T> {
        self.get_string(key)
            .map(|v| lexical_core::parse(v.as_bytes()).ok())
            .flatten()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    fn requires_quotes(s: &str) -> bool {
        s.contains(char::is_whitespace)
    }
}

impl Display for GCodeExtendedParams {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut first = true;
        for (k, v) in &self.0 {
            if !first {
                write!(f, " ")?;
            }
            first = false;
            if Self::requires_quotes(k) {
                write!(f, "\"{}\"", k)?;
            } else {
                write!(f, "{}", k)?;
            }
            write!(f, "=")?;
            if Self::requires_quotes(v) {
                write!(f, "\"{}\"", v)?;
            } else {
                write!(f, "{}", v)?;
            }
        }
        Ok(())
    }
}

#[derive(Debug, PartialEq, PartialOrd, Clone)]
pub struct GCodeCommand {
    pub op: GCodeOperation,
    pub comment: Option<String>,
}

impl Display for GCodeCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.op.fmt(f)?;
        if let Some(comment) = &self.comment {
            if !self.op.is_nop() {
                write!(f, " ;{}", comment)?;
            } else {
                write!(f, ";{}", comment)?;
            }
        }
        Ok(())
    }
}

#[derive(Error, Debug)]
pub enum GCodeReadError {
    #[error("IO error")]
    IO(#[from] io::Error),
    #[error("invalid gcode")]
    ParseError(#[from] GCodeParseError),
}

pub struct GCodeReader<R: BufRead> {
    rdr: R,
    buf: String,
}
impl<R: BufRead> GCodeReader<R> {
    pub fn new(rdr: R) -> GCodeReader<R> {
        GCodeReader {
            rdr,
            buf: String::new(),
        }
    }

    pub fn buffer(&self) -> &str {
        self.buf.as_str()
    }
}

impl<R: BufRead> Iterator for GCodeReader<R> {
    type Item = Result<GCodeCommand, GCodeReadError>;

    fn next(&mut self) -> Option<Self::Item> {
        self.buf.clear();
        match self.rdr.read_line(&mut self.buf) {
            Ok(0) => None,
            Ok(_) => Some(parse_gcode(&self.buf).map_err(|e| e.into())),
            Err(e) => Some(Err(GCodeReadError::IO(e))),
        }
    }
}

pub use parser::parse_gcode;
pub use parser::GCodeParseError;

mod parser {
    #![allow(clippy::many_single_char_names)]

    use super::*;
    use nom::{
        branch::alt,
        bytes::complete::{tag, tag_no_case, take_till, take_until, take_while},
        character::complete::{satisfy, space0, space1},
        combinator::{complete, eof, map, opt, recognize},
        error::{Error, ErrorKind, ParseError},
        multi::separated_list0,
        sequence::tuple,
        Err, IResult, Parser, Slice,
    };
    use std::borrow::Cow;

    #[derive(Debug)]
    pub struct GCodeParseError {
        position: String,
    }

    impl std::error::Error for GCodeParseError {}

    impl std::fmt::Display for GCodeParseError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "gcode parse error at: {}", self.position)
        }
    }

    pub fn parse_gcode(cmd: &str) -> Result<GCodeCommand, GCodeParseError> {
        match parse(cmd.trim()) {
            Ok((_, o)) => Ok(o),
            Err(Err::Incomplete(_)) => Err(GCodeParseError {
                position: "".into(),
            }),
            Err(Err::Error(e)) => Err(GCodeParseError {
                position: e.input.into(),
            }),
            Err(Err::Failure(e)) => Err(GCodeParseError {
                position: e.input.into(),
            }),
        }
    }

    fn parse(s: &str) -> IResult<&str, GCodeCommand> {
        let (s, _) = space0(s)?;

        let (s, _line_no) = opt(line_number)(s)?;

        let (s, (op, comment)) = alt((
            complete(traditional_gcode)
                .map(|(letter, code, params, c)| (map_traditional(letter, code, params), c)),
            complete(extended_gcode),
            // Just a comment
            complete(map(comment, |c| (GCodeOperation::Nop, Some(c)))),
            // Empty line
            complete(eof.map(|_| (GCodeOperation::Nop, None))),
        ))(s)?;

        let comment = comment.map(String::from);

        Ok((s, GCodeCommand { op, comment }))
    }

    fn skip_space(s: &str) -> IResult<&str, ()> {
        if s.is_empty() {
            Ok(("", ()))
        } else {
            space0(s).map(|(s, _)| (s, ()))
        }
    }

    fn line_number(s: &str) -> IResult<&str, u64> {
        let (s, _) = tag_no_case("N")(s)?;
        let (s, v) = match lexical_core::parse_partial::<u64>(s.as_bytes()) {
            Ok((value, processed)) => (s.slice(processed..), value),
            Err(_) => return Err(Err::Error(Error::from_error_kind(s, ErrorKind::Digit))),
        };
        let (s, _) = skip_space(s)?;
        Ok((s, v))
    }

    type RawTraditionalGcode<'a> = (char, u16, Vec<(char, &'a str)>, Option<&'a str>);
    fn traditional_gcode(s: &str) -> IResult<&str, RawTraditionalGcode> {
        let (s, letter) = satisfy(|c| c.is_alphabetic())(s)?;
        let (s, code) = match lexical_core::parse_partial::<u16>(s.as_bytes()) {
            Ok((_, 0)) => return Err(Err::Error(Error::from_error_kind(s, ErrorKind::Digit))),
            Ok((value, processed)) => (s.slice(processed..), value),
            Err(_) => return Err(Err::Error(Error::from_error_kind(s, ErrorKind::Digit))),
        };
        let (s, _) = skip_space(s)?;
        let (s, params) = separated_list0(space1, traditional_param)(s)?;
        let (s, comment) = opt(comment)(s)?;
        Ok((s, (letter.to_ascii_uppercase(), code, params, comment)))
    }

    fn traditional_param(s: &str) -> IResult<&str, (char, &str)> {
        let (s, letter) = satisfy(|c| c.is_alphabetic() && c != ';')(s)?;
        let (s, value) = take_till(|c: char| c.is_whitespace() || c == ';')(s)?;
        Ok((s, (letter.to_ascii_uppercase(), value)))
    }

    fn map_traditional(letter: char, code: u16, params: Vec<(char, &str)>) -> GCodeOperation {
        match (letter, code) {
            ('G', 0 | 1) => {
                let mut x = None;
                let mut y = None;
                let mut z = None;
                let mut e = None;
                let mut f = None;

                for (c, v) in params.into_iter() {
                    let v = match lexical_core::parse::<f64>(v.as_bytes()) {
                        Ok(v) => v,
                        _ => continue,
                    };
                    match c {
                        'X' => x = Some(v),
                        'Y' => y = Some(v),
                        'Z' => z = Some(v),
                        'E' => e = Some(v),
                        'F' => f = Some(v),
                        _ => {}
                    }
                }

                GCodeOperation::Move { x, y, z, e, f }
            }
            _ => GCodeOperation::Traditional {
                letter,
                code,
                params: GCodeTraditionalParams(
                    params
                        .into_iter()
                        .map(|(c, s)| (c, String::from(s)))
                        .collect(),
                ),
            },
        }
    }

    fn extended_gcode(s: &str) -> IResult<&str, (GCodeOperation, Option<&str>)> {
        let (s, cmd) = extended_name(s)?;
        let (s, _) = skip_space(s)?;
        let (s, params) = separated_list0(space1, extended_param)(s)?;
        let (s, comment) = opt(comment)(s)?;

        Ok((
            s,
            (
                GCodeOperation::Extended {
                    command: cmd.to_lowercase(),
                    params: GCodeExtendedParams(
                        params
                            .into_iter()
                            .map(|(k, v)| (k.to_lowercase(), v.into_owned()))
                            .collect(),
                    ),
                },
                comment,
            ),
        ))
    }

    fn extended_name(s: &str) -> IResult<&str, &str> {
        recognize(tuple((
            satisfy(|c| c.is_alphabetic()),
            take_while(|c: char| c.is_alphanumeric() || c == '_'),
        )))(s)
    }

    fn extended_param(s: &str) -> IResult<&str, (&str, Cow<str>)> {
        let (s, k) = take_until("=")(s)?;
        let (s, _) = tag("=")(s)?;
        let (s, v) = maybe_quoted_string(s)?;
        Ok((s, (k, v)))
    }

    fn maybe_quoted_string(s: &str) -> IResult<&str, Cow<str>> {
        match take_till(|c: char| c.is_whitespace() || c == '"' || c == ';')(s)? {
            (s, v)
                if s.chars()
                    .next()
                    .map_or(true, |c| c.is_whitespace() || c == ';') =>
            {
                Ok((s, Cow::from(v)))
            }
            _ => todo!(),
        }
    }

    fn comment(s: &str) -> IResult<&str, &str> {
        let (s, _) = space0(s)?;
        let (s, _) = tag(";")(s)?;
        Ok(("", s.trim_end()))
    }
}
