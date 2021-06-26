use std::collections::BTreeMap;
use std::fs::File;
use std::io::{self, BufRead, BufReader};

#[derive(Debug, Clone)]
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
        params: Vec<(char, String)>,
    },
    Extended {
        cmd: String,
        params: BTreeMap<String, String>,
    },
    Raw(String),
}

#[derive(Debug)]
pub struct GCodeCommand {
    pub op: GCodeOperation,
    pub comment: Option<String>,
}

pub struct GCodeReader<R: BufRead> {
    rdr: R,
    buf: String,
}

mod parser {
    use super::{GCodeCommand, GCodeOperation};
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

    pub fn parse_gcode(cmd: &str) -> GCodeCommand {
        parse(cmd.trim()).expect("parse failed").1
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
            space1(s).map(|(s, _)| (s, ()))
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

    fn traditional_gcode(s: &str) -> IResult<&str, (char, u16, Vec<(char, &str)>, Option<&str>)> {
        let (s, letter) = satisfy(|c| c.is_alphabetic())(s)?;
        let (s, code) = match lexical_core::parse_partial::<u16>(s.as_bytes()) {
            Ok((value, processed)) => (s.slice(processed..), value),
            Err(_) => return Err(Err::Error(Error::from_error_kind(s, ErrorKind::Digit))),
        };
        let (s, _) = skip_space(s)?;
        let (s, params) = separated_list0(space1, traditional_param)(s)?;
        let (s, comment) = opt(comment)(s)?;
        Ok((s, (letter, code, params, comment)))
    }

    fn traditional_param(s: &str) -> IResult<&str, (char, &str)> {
        let (s, letter) = satisfy(|c| c.is_alphabetic())(s)?;
        let (s, value) = take_till(|c: char| c.is_whitespace())(s)?;
        Ok((s, (letter, value)))
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
                    match std::ascii::AsciiExt::to_ascii_lowercase(&c) {
                        'x' => x = Some(v),
                        'y' => y = Some(v),
                        'z' => z = Some(v),
                        'e' => e = Some(v),
                        'f' => f = Some(v),
                        _ => {}
                    }
                }

                GCodeOperation::Move { x, y, z, e, f }
            }
            _ => GCodeOperation::Traditional {
                letter,
                code,
                params: params
                    .into_iter()
                    .map(|(c, s)| (c, String::from(s)))
                    .collect(),
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
                    cmd: String::from(cmd),
                    params: params
                        .into_iter()
                        .map(|(k, v)| (String::from(k), v.into_owned()))
                        .collect(),
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
        match take_till(|c: char| c.is_whitespace() || c == '"')(s)? {
            (s, v) if s.chars().nth(0).map_or(true, |c| c.is_whitespace()) => {
                return Ok((s, Cow::from(v)))
            }
            _ => todo!(),
        }
    }

    fn comment(s: &str) -> IResult<&str, &str> {
        let (s, _) = space0(s)?;
        let (s, _) = tag(";")(s)?;
        let (s, _) = space0(s)?;
        Ok(("", s.trim_end()))
    }
}

impl<R: BufRead> GCodeReader<R> {
    pub fn new(rdr: R) -> GCodeReader<R> {
        GCodeReader {
            rdr,
            buf: String::new(),
        }
    }
}

impl<R: BufRead> Iterator for GCodeReader<R> {
    type Item = io::Result<GCodeCommand>;

    fn next(&mut self) -> Option<Self::Item> {
        self.buf.clear();
        match self.rdr.read_line(&mut self.buf) {
            Ok(0) => None,
            Ok(_) => Some(Ok(parser::parse_gcode(&self.buf))),
            Err(e) => Some(Err(e)),
        }
    }
}

fn main() {
    let file = File::open("/home/dalegaard/Downloads/3DBenchy.gcode").expect("open file");
    let rdr = BufReader::new(file);
    let rdr = GCodeReader::new(rdr);

    let mut i = 0;
    for cmd in rdr {
        let cmd = cmd.expect("gcode read");
        i += 1;
        // println!("{:?}", cmd);
    }
    println!("TOTAL {}", i);
}
