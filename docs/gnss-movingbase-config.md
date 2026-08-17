# GNSS moving-baseline heading — receiver configuration

This documents the device-side configuration required for the dual-receiver
moving-baseline heading, the failure mode it produces when incomplete, and the
exact changes applied to bring an affected unit to a known-good profile.

## Background

The heading is computed **on the receivers**, not in ROS:

- The **F9P** (position) runs as a *moving base*. Over its **UART2** it streams,
  to the F9H, the reference position (RTCM **4072.0** + 4072.1) **and** the base
  observations (RTCM **MSM4**: 1074/1084/1094/1124).
- The **F9H** (heading) runs as a *moving-base rover*. It accepts RTCM3 on its
  UART, computes the relative position vector, and outputs `UBX-NAV-RELPOSNED`,
  which the driver republishes as the `sensor_msgs/Imu` heading.

If the F9P is **not** sending the reference message (`4072.0`), the rover has
observations but nothing to anchor them to, so `NAV-RELPOSNED` comes back with
`flags = 1` (only `GNSS_FIX_OK`; `DIFF_SOLN`, `REL_POS_VALID` and
`REL_POS_HEAD_VALID` all clear), `rel_pos_*` are 0, and the heading is a constant
placeholder (`rel_pos_heading = 0` → yaw 90°) with covariance 1000. The driver
then logs `GNSS heading not valid`.

This is independent of the ublox driver and of any NTRIP/SPARTN correction stream
— NTRIP corrects *absolute* position and is not involved in the moving-baseline
*heading*. Reference: u-blox
[ZED-F9P Moving Base Application Note (UBX-19009093)](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf).

## As-found configuration (heading not working)

Captured with `scripts/gnss_config_probe.py` (then named `gnss_movingbase_probe.py`):

```
F9P  (position / moving base)  [/dev/ttyACM1]
  RAM:
    CFG_TMODE_MODE                       = 0
    CFG_RATE_MEAS                        = 1000      # 1 Hz
    CFG_RATE_NAV                         = 1
    CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2  = 0         # <-- MISSING (root cause)
    CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2  = 1
    CFG_MSGOUT_RTCM_3X_TYPE1074_UART2    = 1         # GPS MSM4
    CFG_MSGOUT_RTCM_3X_TYPE1084_UART2    = 1         # GLONASS MSM4
    CFG_MSGOUT_RTCM_3X_TYPE1094_UART2    = 1         # Galileo MSM4
    CFG_MSGOUT_RTCM_3X_TYPE1124_UART2    = 1         # BeiDou MSM4
    CFG_UART2_BAUDRATE                   = 38400     # <-- differs from known-good
    CFG_UART2OUTPROT_RTCM3X              = 1
  FLASH: 4072.1, 1074, 1084, 1094, 1124, UART2OUTPROT_RTCM3X
         (no 4072.0, no UART2 baud override)

F9H  (heading / rover)  [/dev/ttyACM0]
  RAM:
    CFG_UART1INPROT_RTCM3X               = 1
    CFG_UART2INPROT_RTCM3X               = 1
    CFG_UART1_BAUDRATE                   = 38400
    CFG_UART2_BAUDRATE                   = 38400     # <-- differs from known-good
    CFG_MSGOUT_UBX_NAV_RELPOSNED_USB     = 1
  FLASH: UART2INPROT_RTCM3X
```

## Known-good configuration (heading working)

```
F9P  (position / moving base)  [/dev/ttyACM1]
  RAM:
    CFG_TMODE_MODE                       = 0
    CFG_RATE_MEAS                        = 1000
    CFG_RATE_NAV                         = 1
    CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2  = 1         # reference-station PVT
    CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2  = 1
    CFG_MSGOUT_RTCM_3X_TYPE1074_UART2    = 1
    CFG_MSGOUT_RTCM_3X_TYPE1084_UART2    = 1
    CFG_MSGOUT_RTCM_3X_TYPE1094_UART2    = 1
    CFG_MSGOUT_RTCM_3X_TYPE1124_UART2    = 1
    CFG_UART2_BAUDRATE                   = 115200
    CFG_UART2OUTPROT_RTCM3X              = 1
  FLASH: RATE_MEAS, 4072.0, 4072.1, 1074, 1084, 1094, 1124,
         UART2_BAUDRATE(115200), UART2OUTPROT_RTCM3X

F9H  (heading / rover)  [/dev/ttyACM0]
  RAM:
    CFG_UART1INPROT_RTCM3X               = 1
    CFG_UART2INPROT_RTCM3X               = 1
    CFG_UART1_BAUDRATE                   = 38400
    CFG_UART2_BAUDRATE                   = 115200
    CFG_MSGOUT_UBX_NAV_RELPOSNED_USB     = 1
  FLASH: UART2_BAUDRATE(115200)
```

Note: MSM7 (1077/1087/1097/1127) and 1230 are `0` on both units — this fleet
uses MSM4, which is sufficient for the short on-vehicle baseline.

## Changes applied (every difference, and why)

| # | Module | Key | From | To | Why |
|---|--------|-----|------|----|-----|
| 1 | F9P | `CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2` | 0 | 1 | **The fix.** RTCM 4072.0 is the moving base's reference-station PVT. Without it the rover gets observations but no reference and cannot fix the baseline (`flags:1`, no heading). 4072.1 + MSM4 were already enabled, so this single message was the functional gap. |

That single message is the whole fix. Everything else was already identical:
`4072.1`, MSM4 (`1074/1084/1094/1124`), `UART2OUTPROT_RTCM3X`, F9H
`UART1/2 INPROT_RTCM3X`, `NAV-RELPOSNED` output, the 1 Hz rate, and
`TMODE_MODE = 0`. `gnss_apply_default.py` also explicitly writes MSM7 + 1230 as
`0` so a unit that had them on is brought to the exact profile.

## What the default apply writes (every field, and why)

`gnss_apply_default.py --apply` writes the fields below to **both** receivers
(one `CFG-VALSET` each), to layers **RAM + BBR + FLASH** (`layers = 7`) so they
survive a reset or power-cycle. Run it with the serial ports free (container
stopped). The scripts address the receivers by their stable `by-id` symlinks
(`...F9P-if00`, `...F9H-if00`) — the same nodes the driver opens — so they always
hit the right module even when the kernel renumbers `/dev/ttyACMx`.

### F9P — the moving base

The base computes its own position each epoch and streams, over **UART2** to the
F9H, the reference info plus the raw observations the rover needs to solve the
baseline.

| Key | Value | Why |
|-----|-------|-----|
| `CFG_TMODE_MODE` | 0 | Time mode **off**. A moving base must not be a stationary / survey-in base — it has to recompute its own position every epoch. |
| `CFG_RATE_MEAS` | 1000 | Measurement period 1000 ms → **1 Hz**. Base and rover must run at the same rate. |
| `CFG_RATE_NAV` | 1 | One nav solution per measurement → 1 Hz. |
| `CFG_UART2OUTPROT_RTCM3X` | 1 | Enable **RTCM3 output on UART2** (the wire to the F9H). Without it the base sends nothing the rover can use. |
| `CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2` | 1 | **RTCM 4072.0** — u-blox moving-base reference-station PVT. The anchor for the baseline; the message whose absence breaks heading. |
| `CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2` | 1 | **RTCM 4072.1** — additional moving-base reference info; companion to 4072.0. |
| `CFG_MSGOUT_RTCM_3X_TYPE1074_UART2` | 1 | **GPS MSM4** base observations. |
| `CFG_MSGOUT_RTCM_3X_TYPE1084_UART2` | 1 | **GLONASS MSM4** base observations. |
| `CFG_MSGOUT_RTCM_3X_TYPE1094_UART2` | 1 | **Galileo MSM4** base observations. |
| `CFG_MSGOUT_RTCM_3X_TYPE1124_UART2` | 1 | **BeiDou MSM4** base observations. |
| `CFG_MSGOUT_RTCM_3X_TYPE1077/1087/1097/1127_UART2` | 0 | **MSM7** (high-resolution) equivalents **off**. MSM4 is enough for the short on-vehicle baseline and uses ~half the UART bandwidth; keeping MSM7 off matches the reference unit and avoids saturating the link. |
| `CFG_MSGOUT_RTCM_3X_TYPE1230_UART2` | 0 | GLONASS code-phase biases **off** — not needed for this moving-base setup (reference unit has it off). |

### F9H — the moving-base rover

The rover takes the base's RTCM in, solves the relative-position vector, and
outputs it as `UBX-NAV-RELPOSNED`, which the driver republishes as the heading.

| Key | Value | Why |
|-----|-------|-----|
| `CFG_RATE_MEAS` | 1000 | 1 Hz, matching the base. |
| `CFG_RATE_NAV` | 1 | 1 Hz. |
| `CFG_UART1INPROT_RTCM3X` | 1 | Accept **RTCM3 input on UART1**. |
| `CFG_UART2INPROT_RTCM3X` | 1 | Accept **RTCM3 input on UART2** (the wire from the base). The rover must accept RTCM in to use the base's data. |
| `CFG_MSGOUT_UBX_NAV_RELPOSNED_USB` | 1 | Output **UBX-NAV-RELPOSNED** on USB — the relative-position / heading vector the driver reads. |

The optional `--uart2-baud N` flag additionally writes `CFG_UART2_BAUDRATE = N`
to **both** modules (see the baud note next).

**UART2 baud — change both ends together, or not at all.** The base→rover link
only works if the F9P and F9H use the *same* UART2 baud. Changing only one side
breaks the link (an early one-sided attempt that left the modules at mismatched
bauds is what made `115200` look unsafe). Setting both ends to the same value
works: units run reliably at either 38400 or 115200. `gnss_apply_default.py`
therefore leaves baud alone by default and only changes it with the opt-in
`--uart2-baud N` flag, which writes `CFG_UART2_BAUDRATE = N` to **both** modules
in the same run. This is always reversible: the receivers are configured over
**USB** (`ttyACM`), which is independent of the UART2 link baud, so a bad value
can be undone by re-running with the previous baud.

All values are written to RAM + BBR + **FLASH**, so they survive resets.

## How to apply

With the receivers' serial ports free (stop the GNSS process/container first):

```bash
sudo python3 -m pip install pyubx2
docker stop automatepro-core-driver

sudo python3 scripts/gnss_config_probe.py            # before
sudo python3 scripts/gnss_apply_default.py --apply   # expect ACK-ACK for BOTH modules
sudo python3 scripts/gnss_config_probe.py            # after — F9P 4072.0 now 1

docker start automatepro-core-driver
```

To also match a reference unit's UART2 baud (writes both modules in one run):

```bash
sudo python3 scripts/gnss_apply_default.py --uart2-baud 115200 --apply
```

Revert with the previous value (e.g. `--uart2-baud 38400`) if the heading does
not come back — the USB config path stays reachable regardless of the link baud.

Confirm both modules returned `ACK-ACK` before restarting. After restart, allow 30–60 s, then
`NAV-RELPOSNED` `flags` should show `DIFF_SOLN | REL_POS_VALID | REL_POS_HEAD_VALID`
and `rel_pos_length` non-zero:

```bash
ros2 topic echo --once /sensor/gnss/heading/navrelposned --field flags
```
