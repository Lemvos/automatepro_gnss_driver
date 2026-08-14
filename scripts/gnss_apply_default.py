#!/usr/bin/env python3
"""Provision the F9P (base) and F9H (rover) to the known-good moving-baseline
config (a verified-good profile), and persist it to flash.

Why: for moving-baseline heading the F9P must send the F9H, over UART2, the
reference position (RTCM 4072.0/4072.1) AND base observations (MSM4:
1074/1084/1094/1124); the F9H must accept RTCM3 on its UART and output
UBX-NAV-RELPOSNED. If the F9P is missing 4072.0 (or the observations) the rover
gets NAV-RELPOSNED with flags=1 and no heading. See u-blox UBX-19009093.

This writes the known-good message set to RAM + BBR + FLASH on both receivers.
Each receiver is written in one CFG-VALSET. It also *disables* MSM7/1230 so the
result matches the verified-good profile.

It deliberately does NOT touch UART2 baud. Both modules must already share the
SAME UART2 baud (the value the link runs at); forcing a different baud can break
a working link (observed: 115200 broke a unit wired for 38400). If you must
change it, set the same value on both ends and re-verify the heading.

DRY-RUN by default. Pass --apply to write. Reversible (re-run with edited
values). Run with the core-driver container stopped (ports free):
    docker stop automatepro-core-driver
    sudo python3 scripts/gnss_apply_default.py            # preview
    sudo python3 scripts/gnss_apply_default.py --apply    # write
    sudo python3 scripts/gnss_config_probe.py             # verify
    docker start automatepro-core-driver
"""
import sys
from serial import Serial
from pyubx2 import UBXMessage, UBXReader

LAYERS_RAM_BBR_FLASH = 7   # bit0 RAM | bit1 BBR | bit2 FLASH

# Use the stable by-id symlinks (the nodes the driver opens), NOT /dev/ttyACMx:
# the ACM numbers are reassigned on re-enumeration and the F9P/F9H can swap, so
# a hard-coded ttyACMx risks writing one receiver's config to the other.
F9P_PORT = ('/dev/serial/by-id/'
            'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver_F9P-if00')
F9H_PORT = ('/dev/serial/by-id/'
            'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver_F9H-if00')

# port -> (label, [(CFG key, value), ...])
DEVICES = {
    F9P_PORT: ('F9P  (position / moving base)', [
        ('CFG_TMODE_MODE', 0),                          # not survey-in/fixed
        ('CFG_RATE_MEAS', 1000), ('CFG_RATE_NAV', 1),   # 1 Hz
        ('CFG_UART2OUTPROT_RTCM3X', 1),                 # UART2 emits RTCM3
        # moving-base output to the F9H: reference + MSM4 observations
        ('CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2', 1),     # reference-station PVT
        ('CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2', 1),     # additional ref info
        ('CFG_MSGOUT_RTCM_3X_TYPE1074_UART2', 1),       # GPS MSM4
        ('CFG_MSGOUT_RTCM_3X_TYPE1084_UART2', 1),       # GLONASS MSM4
        ('CFG_MSGOUT_RTCM_3X_TYPE1094_UART2', 1),       # Galileo MSM4
        ('CFG_MSGOUT_RTCM_3X_TYPE1124_UART2', 1),       # BeiDou MSM4
        # keep MSM7 + 1230 off, to match the verified-good profile exactly
        ('CFG_MSGOUT_RTCM_3X_TYPE1077_UART2', 0),
        ('CFG_MSGOUT_RTCM_3X_TYPE1087_UART2', 0),
        ('CFG_MSGOUT_RTCM_3X_TYPE1097_UART2', 0),
        ('CFG_MSGOUT_RTCM_3X_TYPE1127_UART2', 0),
        ('CFG_MSGOUT_RTCM_3X_TYPE1230_UART2', 0),
    ]),
    F9H_PORT: ('F9H  (heading / rover)', [
        ('CFG_RATE_MEAS', 1000), ('CFG_RATE_NAV', 1),   # 1 Hz
        ('CFG_UART1INPROT_RTCM3X', 1),                  # accept RTCM in
        ('CFG_UART2INPROT_RTCM3X', 1),
        ('CFG_MSGOUT_UBX_NAV_RELPOSNED_USB', 1),        # heading output
    ]),
}


def apply(port, cfg):
    with Serial(port, 38400, timeout=2) as ser:
        ser.write(UBXMessage.config_set(LAYERS_RAM_BBR_FLASH, 0, cfg).serialize())
        ubr = UBXReader(ser, protfilter=2, quitonerror=0)  # skip non-UBX noise
        for _ in range(80):
            try:
                _, parsed = ubr.read()
            except Exception:
                break
            if parsed is None:
                break
            if parsed.identity in ('ACK-ACK', 'ACK-NAK'):
                return parsed.identity
    return None


def parse_uart2_baud(argv):
    """Opt-in UART2 baud override. Returns the requested baud or None.

    Off by default: the F9P<->F9H link baud is set on both ends and a working
    value must be left alone (forcing 38400->115200 once broke a live link).
    Pass it ONLY to deliberately match a reference unit, and ALWAYS to the same
    value on both modules (this script does write both). Reachability is safe
    either way: we configure the receivers over USB (ttyACM), which is
    independent of the UART2 link baud, so a bad value is always reversible by
    re-running with the previous baud.
    """
    if '--uart2-baud' not in argv:
        return None
    i = argv.index('--uart2-baud')
    try:
        return int(argv[i + 1])
    except (IndexError, ValueError):
        print('ERROR: --uart2-baud needs an integer, e.g. --uart2-baud 115200')
        sys.exit(2)


def main():
    do_apply = '--apply' in sys.argv
    uart2_baud = parse_uart2_baud(sys.argv)
    if uart2_baud is not None:
        print(f'WARNING: also setting CFG_UART2_BAUDRATE={uart2_baud} on BOTH '
              'modules.\n         Both ends change together; revert with '
              '--uart2-baud <previous>.')
    for port, (label, cfg) in DEVICES.items():
        if uart2_baud is not None:
            cfg = cfg + [('CFG_UART2_BAUDRATE', uart2_baud)]
        print(f'\n{label}  [{port}]  (layers RAM+BBR+FLASH)')
        for k, v in cfg:
            print(f'  {k:42s} = {v}')
        if not do_apply:
            continue
        ack = apply(port, cfg)
        print(f'  -> {ack or "(no ACK seen — check container is stopped / wiring)"}')
        if ack == 'ACK-NAK':
            print('     NAK: device rejected the set; stopping.')
            return
    if not do_apply:
        print('\nDRY-RUN — nothing written. Re-run with --apply (container stopped).')
    else:
        print('\nDone. Verify with gnss_config_probe.py, restart the container, and')
        print('check /sensor/gnss/heading/navrelposned flags (expect 2|4|256 set).')


if __name__ == '__main__':
    main()
