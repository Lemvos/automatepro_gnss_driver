# Error handling & recovery

Design notes for the watchdog/recovery, freshness guards, and lifecycle/shutdown
code in `node.cpp`, `node.hpp`, `watchdog.hpp`, and `node_main.cpp`. This is the
rationale behind the code; the review findings that motivated it are in the
top-level `GPS_ANALYSIS.md`.

## Watchdog and recovery

The `Watchdog` monitors **data reception**: `fixCallback` resets it on every
`~/fix` message, and on timeout it requests recovery. Key points:

- **Recovery is comms-gated, not fix-gated.** The watchdog resets on *message
  arrival*, not on a *valid* fix. The receiver keeps emitting `NAV-PVT` (with
  `STATUS_NO_FIX`) while acquiring, so gating recovery on fix validity would
  reset the receiver before it could re-acquire — a reset loop that prevents a
  fix. The watchdog timeout (seconds) is far shorter than GNSS re-acquisition,
  so loss of *fix/heading/RTK quality* is **surfaced**, never used to trigger a
  hardware reset.
- **Requested on the watchdog thread, run on the executor.** The watchdog
  callback only sets the `recovery_requested_` atomic. `recovery_timer_` (a wall
  timer created in the constructor so it survives lifecycle transitions) runs
  `recovery()` on the single-threaded executor, so lifecycle transitions and the
  `gps_`/`updater_` teardown never race the executor's own callbacks.
- **Iterative, not recursive.** `recovery()` re-arms `recovery_requested_` on a
  non-active end state and returns; the next pass runs on the timer. (It used to
  recurse on a persistent outage, growing the stack until the process crashed.)
- **Escalation:** soft → hard → GPIO pulse.
  - *Soft:* a UBX `CFG-RST` controlled software reset (hot start — keeps
    battery-backed data for fast re-fix) of the receiver, then a lifecycle
    deactivate/activate. (The lifecycle cycle alone never touched the receiver.)
  - *Hard:* deactivate → cleanup → configure → activate (reopens the serial port).
  - *GPIO:* pulse the hardware reset line, wait, retry.

### GPIO reset line (shared)

`reset_gpio_line()` requests the line only for the pulse (LOW = assert for
`gpio_reset_time_` s, then HIGH = deassert) and releases it immediately, so the
line is never held across the node's lifetime. The two GNSS nodes share one
reset line: if the line is already held by the other consumer, that node is
already pulsing it (which resets both receivers), so this node logs and skips
rather than fighting over the line. The board pull-up holds the line deasserted
between pulses.

## Freshness guards

A consumer must not act on a stale last-known solution:

- **Position** freshness is the comms watchdog (`watchdog.timeout`). Loss of a
  usable fix is also surfaced via `NavSatFix.status` (`STATUS_NO_FIX`), the fix
  `/diagnostics` task, and a throttled log in `fixCallback`.
- **Heading** freshness: `headingCallback` only runs when a heading message
  arrives, so a heading stream that *stops* while position is still streaming
  would otherwise go unnoticed. `fixCallback` (which ticks at the receiver's nav
  cadence while the link is alive) checks the age of the last *valid* heading
  against the same `watchdog.timeout` window and throttle-logs if it is stale. An
  *invalid* heading (orientation covariance left at `kInvalidHeadingCovariance =
  1000.0` by the producer, `hp_pos_rec_product.cpp`) is surfaced and does not
  refresh the freshness clock, so a receiver stuck emitting invalid headings is
  still reported. Heading is surfaced only — never used to trigger recovery.

> Note: message `header.stamp` reflects publish time, not fix validity, so a
> "recent" timestamp does not by itself imply a valid fix — consumers must check
> `NavSatFix.status` / heading covariance.

## Lifecycle bring-up and shutdown (`node_main.cpp`)

- **Signal handling is async-signal-safe.** `rclcpp` signal handlers are
  disabled (`SignalHandlerOptions::None`) so `main` drives teardown and the
  context stays valid. The handler only does `g_stop.exchange(true)`; the first
  signal requests a graceful stop, a second forces `std::_Exit(1)` (escape hatch
  if teardown hangs). A watcher thread observes `g_stop` and `exec.cancel()`s the
  spin (waiting for `is_spinning()` first to avoid a cancel-before-spin race).
- **Transitions are checked, not assumed.** Bring-up verifies
  `configure()→INACTIVE` and `activate()→ACTIVE`; a stop requested during
  bring-up skips `activate()`. The teardown ladder
  (`deactivate`→`cleanup`→`shutdown`) checks each resulting state and logs when a
  fail-safe step did not run. The node is destroyed before `rclcpp::shutdown()`
  on every exit path.
- **`shutdown()` name collision.** `UbloxNode::shutdown()` (closes the serial
  port, called from `on_shutdown()` and the destructor) hides the inherited
  `LifecycleNode::shutdown()` transition. Callers that want the *transition* must
  qualify it: `node->rclcpp_lifecycle::LifecycleNode::shutdown()`.
- **Fail-safe teardown without a device.** `shutdown()` guards `gps_` (created
  only in `on_configure()`), so tearing down a node that never configured (e.g.
  configure failed on a missing device) does not null-deref.

## Tests

- `test/watchdog_test.cpp` — unit tests for the watchdog: dropout triggers
  recovery, a live link does not, a persistent outage re-fires bounded, and the
  timeout/check-interval fields are race-free under concurrent updates. Runs
  anywhere (header-only, no device).
- `test/lifecycle_test.cpp` — drives `UbloxNode` through its states. The
  device-independent cases (construct → UNCONFIGURED, configure-without-device
  fails safe, shutdown-from-unconfigured) run in CI; the full
  configure→activate→deactivate→cleanup→reconfigure→reactivate ladder runs only
  when `UBLOX_TEST_DEVICE=/dev/<receiver>` is set (skipped otherwise).
