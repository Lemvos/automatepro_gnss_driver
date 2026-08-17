#!/usr/bin/env python3
"""Read-only u-blox config probe for the F9P (base) and F9H (rover).

Polls UBX CFG-VALGET for a curated set of moving-baseline keys from both the RAM
(running) and FLASH (persisted) layers and prints them. Non-destructive: only
CFG-VALGET polls are sent, nothing is written.

Why a CURATED key list, not the whole config DB: a CFG-VALGET that includes any
key the firmware does not recognise is rejected wholesale (ACK-NAK, no values),
and polling hundreds of keys also means reading the receiver's own 1 Hz message
stream frame by frame until they all arrive -- slow and brittle. Every key below
is valid on these receivers, so each device answers in a single CFG-VALGET.

The serial ports must be FREE -> stop the core-driver container first:
    docker stop automatepro-core-driver
    sudo python3 scripts/gnss_config_probe.py
    docker start automatepro-core-driver
"""
import sys
from serial import Serial
from pyubx2 import UBXMessage, UBXReader

# Stable by-id symlinks (the same nodes the driver opens), NOT /dev/ttyACMx:
# the kernel renumbers the ACM nodes across re-enumeration and the F9P/F9H can
# swap, so a hard-coded ttyACMx can read the wrong receiver or a missing node.
F9P_PORT = ('/dev/serial/by-id/'
            'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver_F9P-if00')
F9H_PORT = ('/dev/serial/by-id/'
            'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver_F9H-if00')

# label -> (port, [curated CFG keys to read])
DEVICES = {
    'F9P  (position / moving base)': (F9P_PORT, [
        'CFG_TMODE_MODE',
        'CFG_RATE_MEAS', 'CFG_RATE_NAV',
        'CFG_UART2_BAUDRATE', 'CFG_UART2OUTPROT_RTCM3X',
        # moving-base output to the F9H: reference (4072.x) + observations
        'CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2', 'CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2',
        'CFG_MSGOUT_RTCM_3X_TYPE1074_UART2', 'CFG_MSGOUT_RTCM_3X_TYPE1084_UART2',
        'CFG_MSGOUT_RTCM_3X_TYPE1094_UART2', 'CFG_MSGOUT_RTCM_3X_TYPE1124_UART2',
        'CFG_MSGOUT_RTCM_3X_TYPE1077_UART2', 'CFG_MSGOUT_RTCM_3X_TYPE1087_UART2',
        'CFG_MSGOUT_RTCM_3X_TYPE1097_UART2', 'CFG_MSGOUT_RTCM_3X_TYPE1127_UART2',
        'CFG_MSGOUT_RTCM_3X_TYPE1230_UART2',
    ]),
    'F9H  (heading / rover)': (F9H_PORT, [
        'CFG_RATE_MEAS', 'CFG_RATE_NAV',
        'CFG_UART1INPROT_RTCM3X', 'CFG_UART2INPROT_RTCM3X',
        'CFG_UART1_BAUDRATE', 'CFG_UART2_BAUDRATE',
        'CFG_MSGOUT_UBX_NAV_RELPOSNED_USB', 'CFG_MSGOUT_UBX_RXM_RTCM_USB',
    ]),
}
LAYERS = {0: 'RAM (running)', 2: 'FLASH (persisted)'}


def poll(port, keys, layer):
    """Send one CFG-VALGET and collect the key/value pairs from the reply.

    Breaks as soon as every key has arrived, the device NAKs the poll, or the
    read times out, so it never blocks on the receiver's own message stream.
    """
    out = {}
    with Serial(port, 38400, timeout=2) as ser:
        ser.write(UBXMessage.config_poll(layer, 0, keys).serialize())
        ubr = UBXReader(ser, protfilter=2, quitonerror=0)
        for _ in range(40):  # bounded; tolerate interleaved device frames
            try:
                _, parsed = ubr.read()
            except Exception:
                break
            if parsed is None:
                break
            if parsed.identity == 'CFG-VALGET':
                for k in keys:
                    if hasattr(parsed, k):
                        out[k] = getattr(parsed, k)
                if all(k in out for k in keys):
                    break
            elif parsed.identity == 'ACK-NAK':
                break  # poll rejected (an unknown key) -> stop waiting
    return out


def main():
    for label, (port, keys) in DEVICES.items():
        print(f'\n==================== {label}  [{port}] ====================')
        for layer, lname in LAYERS.items():
            try:
                vals = poll(port, keys, layer)
            except Exception as exc:
                print(f'  [{lname}] ERROR: {exc}')
                if isinstance(exc, OSError) and exc.errno in (2, 5, 16):
                    print('         port busy/missing — free it first: '
                          'docker stop automatepro-core-driver')
                continue
            print(f'  --- {lname} ---')
            if not vals:
                print('    (no response / none of these keys set)')
            for k in keys:
                if k in vals:
                    print(f'    {k:42s} = {vals[k]}')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
