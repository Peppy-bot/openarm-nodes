// ratatui main loop. Holds the terminal alive, drives a 50 ms redraw cadence,
// translates crossterm key events into state mutations and action spawns, and
// restores the terminal on any exit path (clean, error, panic, signal).

use std::io;
use std::sync::Arc;
use std::time::Duration;

use crossterm::event::{Event, EventStream, KeyCode, KeyEvent, KeyEventKind};
use crossterm::execute;
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use futures::StreamExt;
use peppygen::NodeRunner;
use peppylib::runtime::CancellationToken;
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph, Widget};
use tracing::{info, warn};

use crate::actions::{move_arm_joints, move_gripper};
use crate::error::Result;
use crate::state::{
    ARM_DOF, ArmTarget, Focus, GRIPPER_CLOSED_M, GRIPPER_OPEN_M, GRIPPER_STEP_M, GripperTarget,
    SharedState, Side, StatusLine, UiState,
};

const REDRAW_INTERVAL: Duration = Duration::from_millis(50);
const FEEDBACK_HZ: u32 = 20;

// Restores the terminal to normal mode whenever it goes out of scope — covers
// clean exit, errors, panics, signals. No `?` in Drop, so failures during
// teardown are logged not propagated.
struct TerminalGuard;

impl TerminalGuard {
    fn install() -> Result<Self> {
        enable_raw_mode()?;
        let mut out = io::stdout();
        execute!(out, EnterAlternateScreen)?;
        Ok(Self)
    }
}

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        let mut out = io::stdout();
        if let Err(e) = execute!(out, LeaveAlternateScreen) {
            warn!(error = %e, "leave alternate screen");
        }
        if let Err(e) = disable_raw_mode() {
            warn!(error = %e, "disable raw mode");
        }
    }
}

pub async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
) -> Result<()> {
    let _guard = TerminalGuard::install()?;
    install_panic_hook();
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    let mut events = EventStream::new();
    let mut redraw = tokio::time::interval(REDRAW_INTERVAL);

    loop {
        tokio::select! {
            _ = token.cancelled() => {
                info!("ui: cancellation received, exiting");
                break;
            }
            _ = redraw.tick() => {
                let snapshot = state.lock().await;
                terminal.draw(|frame| render(frame.area(), &*snapshot, frame.buffer_mut()))?;
            }
            event = events.next() => match event {
                Some(Ok(Event::Key(key))) if key.kind == KeyEventKind::Press => {
                    if handle_key(key, &runner, &state, &token).await {
                        break;
                    }
                }
                Some(Err(e)) => warn!(error = %e, "ui: input error"),
                None => {
                    info!("ui: event stream closed, exiting");
                    break;
                }
                _ => {}
            }
        }
    }

    Ok(())
}

// Restore terminal mode on a Rust panic so a backtrace doesn't land on a raw
// terminal nobody can read.
fn install_panic_hook() {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let mut out = io::stdout();
        let _ = execute!(out, LeaveAlternateScreen);
        let _ = disable_raw_mode();
        prev(info);
    }));
}

// Returns true if the loop should exit.
async fn handle_key(
    key: KeyEvent,
    runner: &Arc<NodeRunner>,
    state: &SharedState,
    token: &CancellationToken,
) -> bool {
    match key.code {
        KeyCode::Char('q') | KeyCode::Esc => {
            token.cancel();
            true
        }
        KeyCode::Char('[') => {
            state.lock().await.focus = Focus::LeftArm;
            false
        }
        KeyCode::Char(']') => {
            state.lock().await.focus = Focus::RightArm;
            false
        }
        KeyCode::Char('{') => {
            state.lock().await.focus = Focus::LeftGripper;
            false
        }
        KeyCode::Char('}') => {
            state.lock().await.focus = Focus::RightGripper;
            false
        }
        KeyCode::Char(c @ '1'..='7') => {
            let mut s = state.lock().await;
            let side = s.focus.side();
            if s.focus.is_arm() {
                s.arm_mut(side).selected_joint = (c as u8 - b'1') as usize;
            }
            false
        }
        KeyCode::Up => {
            let mut s = state.lock().await;
            let side = s.focus.side();
            let step = s.step_rad;
            if s.focus.is_arm() {
                s.arm_mut(side).step_selected(step);
            } else {
                s.gripper_mut(side).step(GRIPPER_STEP_M);
            }
            false
        }
        KeyCode::Down => {
            let mut s = state.lock().await;
            let side = s.focus.side();
            let step = s.step_rad;
            if s.focus.is_arm() {
                s.arm_mut(side).step_selected(-step);
            } else {
                s.gripper_mut(side).step(-GRIPPER_STEP_M);
            }
            false
        }
        KeyCode::Char('+') | KeyCode::Char('=') => {
            let mut s = state.lock().await;
            s.step_size_inc();
            let v = s.step_rad;
            s.set_status(format!("step size = {v:.3} rad"));
            false
        }
        KeyCode::Char('-') | KeyCode::Char('_') => {
            let mut s = state.lock().await;
            s.step_size_dec();
            let v = s.step_rad;
            s.set_status(format!("step size = {v:.3} rad"));
            false
        }
        KeyCode::Char('h') => {
            let mut s = state.lock().await;
            let side = s.focus.side();
            if s.focus.is_arm() {
                s.arm_mut(side).joints = [0.0; ARM_DOF];
                s.set_status(format!("{} arm target reset to home", side.label()));
            }
            false
        }
        KeyCode::Enter => {
            let is_arm = state.lock().await.focus.is_arm();
            if is_arm {
                fire_arm(runner, state, token).await;
            } else {
                let pos = {
                    let s = state.lock().await;
                    s.gripper(s.focus.side()).position
                };
                fire_gripper(runner, state, token, pos, "send target").await;
            }
            false
        }
        KeyCode::Char('o') => {
            fire_gripper(runner, state, token, GRIPPER_OPEN_M, "open").await;
            false
        }
        KeyCode::Char('c') => {
            fire_gripper(runner, state, token, GRIPPER_CLOSED_M, "close").await;
            false
        }
        _ => false,
    }
}

// Validate focus + in_flight, snapshot target, spawn the action task. Returns
// without firing if the focused entity is wrong type or already busy.
async fn fire_arm(
    runner: &Arc<NodeRunner>,
    state: &SharedState,
    token: &CancellationToken,
) {
    let (side, joints) = {
        let mut s = state.lock().await;
        if !s.focus.is_arm() {
            s.set_status("Enter fires arm goals — focus an arm with [ or ]");
            return;
        }
        let side = s.focus.side();
        if s.arm(side).in_flight {
            s.set_status(format!("{} arm: previous goal still in flight", side.label()));
            return;
        }
        s.arm_mut(side).in_flight = true;
        let joints = s.arm(side).joints;
        s.set_status(format!("{} arm: firing move_arm_joints", side.label()));
        (side, joints)
    };
    move_arm_joints::spawn(
        runner.clone(),
        state.clone(),
        token.clone(),
        side,
        joints,
        FEEDBACK_HZ,
    );
}

async fn fire_gripper(
    runner: &Arc<NodeRunner>,
    state: &SharedState,
    token: &CancellationToken,
    position_m: f64,
    label: &str,
) {
    let side = {
        let mut s = state.lock().await;
        let side = match s.focus {
            Focus::LeftGripper => Side::Left,
            Focus::RightGripper => Side::Right,
            _ => {
                s.set_status("o/c fires gripper goals — focus a gripper with { or }");
                return;
            }
        };
        if s.gripper(side).in_flight {
            s.set_status(format!(
                "{} gripper: previous goal still in flight",
                side.label()
            ));
            return;
        }
        s.gripper_mut(side).in_flight = true;
        s.gripper_mut(side).position = position_m;
        s.set_status(format!("{} gripper: {label} ({position_m:.3} m)", side.label()));
        side
    };
    move_gripper::spawn(
        runner.clone(),
        state.clone(),
        token.clone(),
        side,
        position_m,
        FEEDBACK_HZ,
    );
}

// --------------------------- rendering ---------------------------

fn render(area: Rect, state: &UiState, buf: &mut ratatui::buffer::Buffer) {
    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Min(9),
            Constraint::Length(5),
            Constraint::Length(3),
        ])
        .split(area);

    render_header(rows[0], state, buf);

    let arm_cols = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(rows[1]);
    render_arm(arm_cols[0], "left arm", state.focus == Focus::LeftArm, &state.left_arm, buf);
    render_arm(arm_cols[1], "right arm", state.focus == Focus::RightArm, &state.right_arm, buf);

    let gripper_cols = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(rows[2]);
    render_gripper(
        gripper_cols[0], "left gripper",
        state.focus == Focus::LeftGripper, &state.left_gripper, buf,
    );
    render_gripper(
        gripper_cols[1], "right gripper",
        state.focus == Focus::RightGripper, &state.right_gripper, buf,
    );

    render_status(rows[3], state.status.as_ref(), buf);
}

fn render_header(area: Rect, state: &UiState, buf: &mut ratatui::buffer::Buffer) {
    let focus = match state.focus {
        Focus::LeftArm => "LEFT ARM",
        Focus::RightArm => "RIGHT ARM",
        Focus::LeftGripper => "LEFT GRIPPER",
        Focus::RightGripper => "RIGHT GRIPPER",
    };
    let line = Line::from(vec![
        Span::styled(
            "openarm01_joint_commander",
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw("    focus: "),
        Span::styled(focus, Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        Span::raw(format!("    step: {:.3} rad", state.step_rad)),
    ]);
    let block = Block::default().borders(Borders::ALL).title("session");
    Paragraph::new(line).block(block).render(area, buf);
}

fn render_arm(area: Rect, label: &str, focused: bool, arm: &ArmTarget, buf: &mut ratatui::buffer::Buffer) {
    let title = format!(
        "{}{}",
        label,
        if arm.in_flight { "  [in flight]" } else { "" }
    );
    let mut lines: Vec<Line> = Vec::with_capacity(ARM_DOF);
    for i in 0..ARM_DOF {
        let selected = focused && arm.selected_joint == i;
        let target = arm.joints[i];
        let feedback = arm
            .last_feedback
            .as_ref()
            .map(|fb| format!("{:>+7.3}", fb[i]))
            .unwrap_or_else(|| "   ---".to_string());
        let marker = if selected { "▶" } else { " " };
        let style = if selected {
            Style::default().add_modifier(Modifier::BOLD).fg(Color::Cyan)
        } else {
            Style::default()
        };
        lines.push(Line::styled(
            format!("{marker} j{}  target {:>+7.3}    fb {feedback}", i + 1, target),
            style,
        ));
    }
    let block_style = if focused {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default()
    };
    let block = Block::default()
        .borders(Borders::ALL)
        .title(title)
        .border_style(block_style);
    Paragraph::new(lines).block(block).render(area, buf);
}

fn render_gripper(
    area: Rect, label: &str, focused: bool, gripper: &GripperTarget,
    buf: &mut ratatui::buffer::Buffer,
) {
    let title = format!(
        "{}{}",
        label,
        if gripper.in_flight { "  [in flight]" } else { "" }
    );
    let feedback = match gripper.last_feedback.as_ref() {
        Some(v) if !v.is_empty() => {
            let parts: Vec<String> = v.iter().map(|x| format!("{x:+6.4}")).collect();
            parts.join(", ")
        }
        _ => "---".to_string(),
    };
    let lines = vec![
        Line::raw(format!("target  {:+6.4} m", gripper.position)),
        Line::raw(format!("fb      {feedback}")),
    ];
    let block_style = if focused {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default()
    };
    let block = Block::default()
        .borders(Borders::ALL)
        .title(title)
        .border_style(block_style);
    Paragraph::new(lines).block(block).render(area, buf);
}

fn render_status(area: Rect, status: Option<&StatusLine>, buf: &mut ratatui::buffer::Buffer) {
    let text = match status {
        Some(s) if s.is_fresh() => s.message.clone(),
        _ => "[ keys: [/] arm  {/} gripper  1-7 joint  ↑↓ step  +/- step size  Enter fire  o/c open/close  h home  q quit ]".to_string(),
    };
    let block = Block::default().borders(Borders::ALL).title("status");
    Paragraph::new(text).block(block).render(area, buf);
}

