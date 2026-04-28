use std::io::{self, BufRead};

use tokio::sync::mpsc;

use crate::types::{Axis, Command};

pub const HELP_TEXT: &str = "\
  x [delta]          nudge x axis (default: current step size)
  y [delta]          nudge y axis
  z [delta]          nudge z axis
  goto <x> <y> <z>   move to absolute cartesian position
  step <value>       set default step size in metres (e.g. step 0.005)
  reset              return target to origin
  help               show this message
  quit               exit";

pub fn spawn_stdin_reader(tx: mpsc::Sender<Command>) {
    std::thread::spawn(move || {
        let stdin = io::stdin();
        for line in stdin.lock().lines() {
            let line = match line {
                Ok(l) => l,
                Err(_) => break,
            };
            let trimmed = line.trim();
            match parse(trimmed) {
                ParseOutcome::Cmd(cmd) => {
                    let is_quit = matches!(cmd, Command::Quit);
                    if tx.blocking_send(cmd).is_err() || is_quit {
                        break;
                    }
                }
                ParseOutcome::BadArgs(verb) => {
                    println!("bad arguments for '{verb}' — type 'help' for usage");
                }
                ParseOutcome::Unknown => {
                    println!("unknown command — type 'help' for usage");
                }
                ParseOutcome::Empty => {}
            }
        }
    });
}

enum ParseOutcome {
    Cmd(Command),
    BadArgs(&'static str),
    Unknown,
    Empty,
}

fn parse(input: &str) -> ParseOutcome {
    let mut parts = input.split_whitespace();
    let verb = match parts.next() {
        Some(v) => v.to_ascii_lowercase(),
        None => return ParseOutcome::Empty,
    };
    match verb.as_str() {
        "x" => ParseOutcome::Cmd(Command::Nudge { axis: Axis::X, delta: parse_f64(parts.next()) }),
        "y" => ParseOutcome::Cmd(Command::Nudge { axis: Axis::Y, delta: parse_f64(parts.next()) }),
        "z" => ParseOutcome::Cmd(Command::Nudge { axis: Axis::Z, delta: parse_f64(parts.next()) }),
        "goto" => {
            match (parse_f64(parts.next()), parse_f64(parts.next()), parse_f64(parts.next())) {
                (Some(x), Some(y), Some(z)) => ParseOutcome::Cmd(Command::Goto { x, y, z }),
                _ => ParseOutcome::BadArgs("goto"),
            }
        }
        "step" => match parse_f64(parts.next()) {
            Some(v) => ParseOutcome::Cmd(Command::SetStep(v)),
            None => ParseOutcome::BadArgs("step"),
        },
        "reset" => ParseOutcome::Cmd(Command::Reset),
        "help"  => ParseOutcome::Cmd(Command::Help),
        "quit" | "exit" | "q" => ParseOutcome::Cmd(Command::Quit),
        _ => ParseOutcome::Unknown,
    }
}

fn parse_f64(s: Option<&str>) -> Option<f64> {
    s?.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn as_cmd(o: ParseOutcome) -> Command {
        match o { ParseOutcome::Cmd(c) => c, _ => panic!("expected Cmd") }
    }

    #[test]
    fn parse_nudge_with_value() {
        let cmd = as_cmd(parse("x 0.05"));
        assert!(matches!(cmd, Command::Nudge { axis: Axis::X, delta: Some(d) } if (d - 0.05).abs() < 1e-9));
    }

    #[test]
    fn parse_nudge_no_value() {
        let cmd = as_cmd(parse("y"));
        assert!(matches!(cmd, Command::Nudge { axis: Axis::Y, delta: None }));
    }

    #[test]
    fn parse_goto() {
        let cmd = as_cmd(parse("goto 1.0 -2.0 0.5"));
        assert!(matches!(cmd, Command::Goto { x, y, z } if x == 1.0 && y == -2.0 && z == 0.5));
    }

    #[test]
    fn parse_goto_bad_args() {
        assert!(matches!(parse("goto 1 abc 3"), ParseOutcome::BadArgs("goto")));
        assert!(matches!(parse("step xyz"),     ParseOutcome::BadArgs("step")));
    }

    #[test]
    fn parse_unknown_and_empty() {
        assert!(matches!(parse("fly"), ParseOutcome::Unknown));
        assert!(matches!(parse(""),    ParseOutcome::Empty));
    }
}
