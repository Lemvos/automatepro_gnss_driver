#!/usr/bin/env bash
# Hardware-in-the-loop test for the ublox_gps recovery ladder.
#
# Injects a comms fault by unbinding a receiver's CDC-ACM interfaces, so the
# driver stops receiving ~/fix and the watchdog escalates through its rungs:
# reopen (host side only) -> device reset (UBX-CFG-RST) -> hardware reset
# (shared GPIO pulse). Both nodes' journals are captured so the rung sequence,
# the shared-line pulse budget, the retry backoff, and the partner receiver's
# collateral recovery can all be checked after the run.
#
# SAFETY
#   - GNSS is lost on BOTH receivers. The hardware rung pulses a reset line they
#     share, so exercising one receiver takes the other down with it.
#   - MissionSupervisor triggers an emergency stop when the pose goes stale
#     (localization_timeout_s, default 5 s) during active navigation. Run only
#     with the vessel moored and autonomy idle or safe-stopped.
#   - Confirm from the hardware schematic that the reset line drives only the
#     two GNSS receivers before the first run. Nothing in software can verify
#     what else may be wired to it.
#   - Do not run gpiomon or gpioget on the reset line during the test. They
#     claim the line, and the driver then logs that it is held by a non-GNSS
#     consumer and skips the pulse, which looks like a recovery failure.
#   - RTK fix and moving-baseline heading take minutes to re-converge after a
#     hardware reset. Do not schedule cm-accuracy work straight afterwards.
#
# HARDWARE DEFAULTS
#   F9H_USB and F9P_USB are USB topology paths for one specific vessel's wiring
#   and are not portable. The script resolves the live by-id symlinks and
#   refuses to unbind anything that does not match, because the same cdc_acm
#   driver also owns the AMP IO-Controller (bilge pumps, dock latch, generator
#   status); an unguarded unbind could take out vessel IO. Rediscover with:
#     ls -l /dev/serial/by-id/ | grep F9
#     readlink -f /sys/class/tty/ttyACM0/device
#
# Requires sudo for the USB bind/unbind and for journalctl on the units. Not
# installed by CMake; run it from the source tree.
#
# RUN IT IN THE FOREGROUND. Started as a background job of a non-interactive
# shell, SIGINT is set to SIG_IGN on entry, and bash cannot trap a signal that
# was ignored on entry - the cleanup trap would be silently inert and an
# interrupt would leave a receiver unbound.

set -o pipefail

F9H_USB=1-3.3.1
F9P_USB=1-3.3.2

CDC=/sys/bus/usb/drivers/cdc_acm
LOGDIR=/tmp/gnss_recovery_test
GPIO_CHIP=gpiochip0
GPIO_LINE=134

# Restored before the hardware rung is due, so a transient case exercises only
# the reopen and device-reset rungs.
TRANSIENT_OUTAGE_S=4
# Measured on this vessel: the ladder reaches the spent pulse budget 42 s after
# the fault and announces the 60 s backoff cap at 75 s. 90 s covers both with
# margin; longer only repeats capped retries.
SUSTAINED_OUTAGE_S=90
# Ceiling for waiting on recovery after the device returns. The node can be up
# to recovery.backoff_max_s (60 s) into a backoff when the device comes back, so
# this must exceed that.
RECOVERY_WAIT_TIMEOUT_S=90
ABORT_WINDOW_S=10

RUN_HEADING=1
RUN_POSITION=1
RUN_CONTENTION=0
RUN_START_TS=""
# USB paths currently unbound, space separated, so an interrupt can rebind them.
FAULTED_USB=""

usage() {
  cat <<'EOF'
Usage: gnss_recovery_test.sh [--heading-only | --position-only] [-h]

  --heading-only   fault only the F9H; the F9P is left alone and its journal is
                   captured as the collateral check
  --position-only  fault only the F9P; the F9H provides the collateral check
  --contention     fault BOTH receivers at once, so both ladders reach the
                   hardware rung together and race for the shared reset line.
                   The only way to exercise the peer-detection branch: with one
                   receiver faulted the partner never loses data, so a second
                   ladder never runs and the line is never contended.
  -h, --help       show this help and exit

With no option both receivers are exercised, transient case then sustained
case for each. Logs are written to /tmp/gnss_recovery_test.
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --heading-only)  RUN_POSITION=0 ;;
    --position-only) RUN_HEADING=0 ;;
    --contention)    RUN_CONTENTION=1; RUN_HEADING=0; RUN_POSITION=0 ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

# Writes to stdout and to the timeline file directly. Deliberately not a
# `| tee` pipeline around the run: that would put the cases in a subshell, where
# caught traps are reset to default and FAULTED_USB would be invisible to
# cleanup, so an interrupt could not rebind the receiver.
mark() {
  local line
  line=$(date +"===== $* @ %H:%M:%S")
  echo "$line"
  [ -n "$LOGDIR" ] && echo "$line" >> "$LOGDIR/timeline.log"
}

selected() {
  # The contention case faults both, so both must pass the strict mapping check.
  [ "$RUN_CONTENTION" = 1 ] && return 0
  case "$1" in
    F9H) [ "$RUN_HEADING" = 1 ] ;;
    F9P) [ "$RUN_POSITION" = 1 ] ;;
  esac
}

# Echoes the USB topology path currently backing a receiver, or fails if the
# receiver is absent.
resolve_usb() {
  local link tty
  for link in /dev/serial/by-id/*"${1}"-if00; do
    [ -e "$link" ] || continue
    tty=$(basename "$(readlink -f "$link")") || return 1
    basename "$(dirname "$(readlink -f "/sys/class/tty/${tty}/device")")"
    return 0
  done
  return 1
}

# A receiver about to be unbound must match its expected path exactly. A partner
# that is only being observed may deviate; that costs the collateral check, not
# safety, so it warns instead of aborting.
verify_mapping() {
  local abort=0 r expect actual
  echo "--- receiver to USB mapping ---"
  for r in F9H F9P; do
    eval "expect=\$${r}_USB"
    if ! actual=$(resolve_usb "$r"); then
      if selected "$r"; then
        echo "  $r: NOT PRESENT - cannot fault a receiver that is not there"
        abort=1
      else
        echo "  $r: NOT PRESENT - collateral check unavailable"
      fi
      continue
    fi
    if [ "$actual" = "$expect" ]; then
      if selected "$r"; then
        echo "  $r: $actual  OK (will be faulted)"
      else
        echo "  $r: $actual  OK (observed only)"
      fi
    elif selected "$r"; then
      echo "  $r: $actual  MISMATCH, expected $expect - refusing to unbind"
      echo "      update ${r}_USB at the top of this script after re-checking"
      abort=1
    else
      echo "  $r: $actual  differs from $expect (observed only, not touched)"
    fi
  done
  [ "$abort" = 0 ] || { echo "ABORT: mapping check failed" >&2; exit 1; }
}

# Journals are extracted after the fact from the start timestamp rather than
# followed by a background process. Nothing is left running if the operator
# interrupts the run, which is the whole reason this is not a `journalctl -f`.
extract_logs() {
  local unit
  mkdir -p "$LOGDIR"
  for unit in f9h f9p; do
    # The redirect is performed by this shell, not by sudo, so the file stays
    # owned by the invoking user and readable without root.
    # shellcheck disable=SC2024
    sudo -n journalctl -u "dev-automatepro-gnss-${unit}.service" \
      --since "$RUN_START_TS" --no-pager -o short-precise \
      > "$LOGDIR/${unit}.log" 2>/dev/null \
      || echo "WARNING: could not extract ${unit} journal (sudo expired?)" >&2
  done
}

# Polls the journal directly for the recovery marker, so no log tail has to be
# running. Window is bounded to the current case to keep each poll cheap.
recovery_seen_since() {
  sudo -n journalctl -u "$(unit_for "$1").service" --since "$2" --no-pager 2>/dev/null \
    | grep -q "Data have been received again"
}

# Runs on every exit path including Ctrl-C, so an interrupted run still leaves
# usable logs. There is no background process to stop, by design.
cleanup() {
  local rc=$?
  trap - EXIT INT TERM

  # Interrupting inside an outage window would otherwise leave the receiver
  # detached from cdc_acm, with no /dev/serial/by-id entry for the node to open.
  # Rebinding is unconditional and idempotent: binding an already-bound
  # interface is a harmless error, leaving one unbound is not.
  if [ -n "$FAULTED_USB" ]; then
    echo "restoring$FAULTED_USB before exit"
    # shellcheck disable=SC2086
    restore_rx $FAULTED_USB
    FAULTED_USB=""
  fi

  [ -n "$RUN_START_TS" ] && extract_logs

  echo
  echo "--- GPIO $GPIO_CHIP line $GPIO_LINE (expect: unused) ---"
  gpioinfo "$GPIO_CHIP" 2>/dev/null | grep -E "line +${GPIO_LINE}:" \
    || echo "  (gpioinfo unavailable)"
  echo
  echo "--- receiver mapping after the run ---"
  local link found=0
  for link in /dev/serial/by-id/*F9[HP]-if00; do
    [ -e "$link" ] || continue
    found=1
    echo "  $(basename "$link") -> $(readlink -f "$link")"
  done
  [ "$found" = 1 ] || echo "  (none present)"
  echo
  if [ "$rc" -ne 0 ]; then
    echo "Run ended early (exit $rc); captured logs up to that point are still valid."
  fi
  echo "Logs for analysis:"
  echo "  $LOGDIR/f9h.log"
  echo "  $LOGDIR/f9p.log"
  echo "  $LOGDIR/timeline.log"
  exit "$rc"
}

fault_rx() {
  local usb i
  # Recorded before the unbind, so an interrupt between the writes still leaves
  # cleanup able to restore every interface.
  for usb in "$@"; do
    FAULTED_USB="${FAULTED_USB} ${usb}"
  done
  for usb in "$@"; do
    for i in "${usb}:1.0" "${usb}:1.1"; do
      echo -n "$i" | sudo tee "$CDC/unbind" >/dev/null 2>&1
    done
  done
}

restore_rx() {
  local usb i
  for usb in "$@"; do
    for i in "${usb}:1.0" "${usb}:1.1"; do
      echo -n "$i" | sudo tee "$CDC/bind" >/dev/null 2>&1
    done
  done
  FAULTED_USB=""
}

# Recovery is complete when the node reports data flowing again. Waiting on that
# rather than a fixed sleep keeps a transient case to a few seconds and still
# covers a sustained one, where the node may be deep in its retry backoff when
# the device returns.
wait_for_recovery() {
  local rx=$1 since=$2 deadline
  deadline=$(( $(date +%s) + RECOVERY_WAIT_TIMEOUT_S ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if recovery_seen_since "$rx" "$since"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

# A node that dies instead of recovering takes its service down with it, and
# every later case would then be measuring nothing. Stop the run instead.
unit_for() { case "$1" in F9H) echo dev-automatepro-gnss-f9h ;; F9P) echo dev-automatepro-gnss-f9p ;; esac; }

# One fault cycle: take the receiver away for the given time, give it back, then
# wait for the driver to recover on its own.
# Faults both receivers at once. Their watchdogs run on identical parameters, so
# the two ladders advance in step and their hardware-rung pulses collide - one
# node takes the line and the other must detect the peer holding it rather than
# contend for it.
run_contention_case() {
  local name=$1 secs=$2 since rx
  since=$(date '+%Y-%m-%d %H:%M:%S')

  mark "$name BEGIN (usb=$F9H_USB + $F9P_USB, outage=${secs}s)"
  fault_rx "$F9H_USB" "$F9P_USB"
  sleep "$secs"
  restore_rx "$F9H_USB" "$F9P_USB"
  mark "$name RESTORED"

  for rx in F9H F9P; do
    if wait_for_recovery "$rx" "$since"; then
      mark "$name ${rx} RECOVERED"
    else
      mark "$name ${rx} NO RECOVERY within ${RECOVERY_WAIT_TIMEOUT_S}s - see the log"
    fi
  done
  mark "$name END"
}

run_case() {
  local name=$1 usb=$2 secs=$3 rx=$4 since unit
  since=$(date '+%Y-%m-%d %H:%M:%S')

  mark "$name BEGIN (usb=$usb, outage=${secs}s)"
  fault_rx "$usb"
  sleep "$secs"
  restore_rx "$usb"
  mark "$name RESTORED"

  if wait_for_recovery "$rx" "$since"; then
    mark "$name RECOVERED"
  else
    unit=$(unit_for "$rx")
    if ! systemctl is-active --quiet "$unit"; then
      mark "$name FAILED - ${unit} is no longer active; the node did not recover"
      echo "The driver did not survive the fault, so the remaining cases would" >&2
      echo "measure nothing. Bring it back with:" >&2
      echo "  sudo systemctl start ${unit}" >&2
      return 1
    fi
    mark "$name NO RECOVERY within ${RECOVERY_WAIT_TIMEOUT_S}s - see the log"
  fi
  mark "$name END"
}

plan() {
  local what="both receivers"
  if [ "$RUN_CONTENTION" = 1 ]; then
    what="both receivers simultaneously (contention case)"
  elif [ "$RUN_POSITION" = 0 ]; then
    what="the F9H (heading) only"
  elif [ "$RUN_HEADING" = 0 ]; then
    what="the F9P (position) only"
  fi
  echo "This test interrupts GNSS and will reset BOTH receivers via the shared"
  echo "line. Faulting $what."
  echo "Ctrl-C within ${ABORT_WINDOW_S}s to abort."
}

plan
sleep "$ABORT_WINDOW_S"

sudo -v || { echo "ABORT: sudo required" >&2; exit 1; }
verify_mapping

RUN_START_TS=$(date '+%Y-%m-%d %H:%M:%S')
trap cleanup EXIT INT TERM
mkdir -p "$LOGDIR"
rm -f "$LOGDIR"/*.log

mark "RUN START"

if [ "$RUN_HEADING" = 1 ]; then
  run_case "A_F9H_TRANSIENT" "$F9H_USB" "$TRANSIENT_OUTAGE_S" F9H || exit 1
  # Reaches the hardware rung. f9p.log is the collateral evidence: the shared
  # pulse resets the position receiver too, and it must recover on its own
  # cheapest rung without escalating to a pulse of its own.
  run_case "B_F9H_SUSTAINED" "$F9H_USB" "$SUSTAINED_OUTAGE_S" F9H || exit 1
fi

if [ "$RUN_POSITION" = 1 ]; then
  run_case "C_F9P_TRANSIENT" "$F9P_USB" "$TRANSIENT_OUTAGE_S" F9P || exit 1
  run_case "D_F9P_SUSTAINED" "$F9P_USB" "$SUSTAINED_OUTAGE_S" F9P || exit 1
fi

if [ "$RUN_CONTENTION" = 1 ]; then
  run_contention_case "E_BOTH_SUSTAINED" "$SUSTAINED_OUTAGE_S"
fi

mark "RUN END"
