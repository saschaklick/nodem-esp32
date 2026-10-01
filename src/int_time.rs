//! The VM's time interrupt, see `virtmach::interrupts::time` for the functions.
//!
//! Waits don't block inside `call` - that would stall every other task on
//! the single-threaded executor (wifi, websocket, uart, ...) for the whole
//! interval. Instead the deadline goes into the shared `VmClock`, the VM is
//! paused and `nodem::nodem_task` holds it until the deadline passed. The VM only runs once per frame, so a wait ends up to one
//! frame (1/60s) late; `wait_until` still counts the next interval from the
//! deadline, so a loop with it keeps its period.
//!
//! get_time and get_date read the system clock in the local time zone (TZ).
//! As long as it wasn't set (year before 2024, nothing runs SNTP yet)
//! get_time counts from the start of the program and get_date is 0, 0, 0.

use std::cell::Cell;
use std::rc::Rc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use esp_idf_svc::sys::{localtime_r, time_t, tm};
use virtmach::{interrupts::{time, SoftInterrupt}, RuntimeError, Storage, VMAtom, VirtMach};

/// Seconds since the epoch before which the system clock counts as not set (2024-01-01).
const CLOCK_SET_AFTER: u64 = 1_704_067_200;

/// State shared between the time interrupt (owned by the `Env`) and
/// `nodem::nodem_task`, which decides whether the VM may run this frame.
pub struct VmClock {
    /// When the current program started, for get_time without a set clock.
    started: Cell<Instant>,
    /// When the last wait_until returned; the first one counts from the start.
    last: Cell<Instant>,
    /// Set by a wait, the VM is held until then.
    deadline: Cell<Option<Instant>>,
}

impl VmClock {
    pub fn new() -> Self {
        let now = Instant::now();
        Self { started: Cell::new(now), last: Cell::new(now), deadline: Cell::new(None) }
    }

    /// A (re)started program: counts from now and drops a pending wait.
    pub fn restart(&self) {
        let now = Instant::now();
        self.started.set(now);
        self.last.set(now);
        self.deadline.set(None);
    }

    /// Whether the VM has to wait this frame, clears the deadline once it passed.
    pub fn hold(&self) -> bool {
        match self.deadline.get() {
            Some(deadline) if Instant::now() < deadline => true,
            Some(_) => { self.deadline.set(None); false }
            None => false,
        }
    }
}

pub struct Time {
    pub clock: Rc<VmClock>,
}

/// seconds * 1000 + milliseconds, None if either is negative
fn interval(seconds: VMAtom, milliseconds: VMAtom) -> Option<Duration> {
    if seconds < 0 || milliseconds < 0 {
        return None;
    }
    Some(Duration::from_secs(seconds as u64) + Duration::from_millis(milliseconds as u64))
}

/// The local time of the system clock, None while it isn't set.
fn local_time() -> Option<(tm, u32)> {
    let since_epoch = SystemTime::now().duration_since(UNIX_EPOCH).ok()?;
    if since_epoch.as_secs() < CLOCK_SET_AFTER {
        return None;
    }
    let secs = since_epoch.as_secs() as time_t;
    let mut local: tm = unsafe { core::mem::zeroed() };
    if unsafe { localtime_r(&secs, &mut local) }.is_null() {
        return None;
    }
    Some((local, since_epoch.subsec_millis()))
}

impl<S: Storage> SoftInterrupt<S> for Time {
    fn name(&self) -> &str {
        return time::NAME;
    }

    fn call(&mut self, vm: &mut VirtMach<S>) {
        let op = vm.stack_pop();
        match op {
            // wait_for and wait_until share the arguments and their check
            0 | 1 => {
                let (seconds, milliseconds) = (vm.stack_pop(), vm.stack_pop());
                let Some(interval) = interval(seconds, milliseconds) else {
                    if op == 1 { vm.stack_push(0); }
                    vm.error = RuntimeError::InterruptError;
                    return;
                };
                let now = Instant::now();
                let deadline = if op == 0 {
                    now + interval
                } else {
                    let deadline = self.clock.last.get() + interval;
                    let waited = deadline > now;
                    // too late: return at once and count the next interval from now
                    self.clock.last.set(if waited { deadline } else { now });
                    vm.stack_push(waited as VMAtom);
                    if !waited { return; }
                    deadline
                };
                self.clock.deadline.set(Some(deadline));
                vm.pause();
            }
            // pushed as hours, minutes, seconds, ms
            2 => {
                let (hours, minutes, seconds, ms) = match local_time() {
                    Some((local, ms)) => (local.tm_hour as u64, local.tm_min as u64, local.tm_sec as u64, ms as u64),
                    None => {
                        let elapsed = self.clock.started.get().elapsed();
                        let secs = elapsed.as_secs();
                        ((secs / 3600) % 24, (secs / 60) % 60, secs % 60, elapsed.subsec_millis() as u64)
                    }
                };
                for value in [hours, minutes, seconds, ms] { vm.stack_push(value as VMAtom); }
            }
            // pushed as year, month, day
            3 => {
                let (year, month, day) = match local_time() {
                    Some((local, _)) => (local.tm_year + 1900, local.tm_mon + 1, local.tm_mday),
                    None => (0, 0, 0),
                };
                for value in [year, month, day] { vm.stack_push(value as VMAtom); }
            }
            _ => { vm.error = RuntimeError::UnimplementedInterruptFunc; }
        }
    }
}
