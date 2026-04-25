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
                Some(cmd) => {
                    let is_quit = matches!(cmd, Command::Quit);
                    if tx.blocking_send(cmd).is_err() || is_quit {
                        break;
                    }
                }
                None if !trimmed.is_empty() => {
                    println!("unknown command — type 'help' for usage");
                }
                None => {}
            }
        }
    });
}

fn parse(input: &str) -> Option<Command> {
    let mut parts = input.split_whitespace();
    let verb = parts.next()?.to_ascii_lowercase();
    match verb.as_str() {
        "x" => Some(Command::Nudge { axis: Axis::X, delta: parse_f64(parts.next()) }),
        "y" => Some(Command::Nudge { axis: Axis::Y, delta: parse_f64(parts.next()) }),
        "z" => Some(Command::Nudge { axis: Axis::Z, delta: parse_f64(parts.next()) }),
        "goto" => Some(Command::Goto {
            x: parse_f64(parts.next())?,
            y: parse_f64(parts.next())?,
            z: parse_f64(parts.next())?,
        }),
        "step"  => Some(Command::SetStep(parse_f64(parts.next())?)),
        "reset" => Some(Command::Reset),
        "help"  => Some(Command::Help),
        "quit" | "exit" | "q" => Some(Command::Quit),
        _ => None,
    }
}

fn parse_f64(s: Option<&str>) -> Option<f64> {
    s?.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_nudge_with_value() {
        let cmd = parse("x 0.05").unwrap();
        assert!(matches!(cmd, Command::Nudge { axis: Axis::X, delta: Some(d) } if (d - 0.05).abs() < 1e-9));
    }

    #[test]
    fn parse_nudge_no_value() {
        let cmd = parse("y").unwrap();
        assert!(matches!(cmd, Command::Nudge { axis: Axis::Y, delta: None }));
    }

    #[test]
    fn parse_goto() {
        let cmd = parse("goto 1.0 -2.0 0.5").unwrap();
        assert!(matches!(cmd, Command::Goto { x, y, z } if x == 1.0 && y == -2.0 && z == 0.5));
    }

    #[test]
    fn parse_unknown_returns_none() {
        assert!(parse("fly").is_none());
        assert!(parse("").is_none());
    }
}
