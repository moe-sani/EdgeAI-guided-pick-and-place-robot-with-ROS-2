"""Set up and calibrate the SO-101 follower arm (same steps and file format as LeRobot).

  ros2 run so101_cell so101_calibrate --port /dev/ttyACM0                  # calibrate
  ros2 run so101_cell so101_calibrate --port /dev/ttyACM0 --setup-motors   # first time only: motor ids

Calibration: put the arm in the middle of its range (LeRobot's "homing position": every joint centred,
gripper half closed), press ENTER, then move every joint through its full range by hand and press ENTER.
The result goes to the motors and to the LeRobot calibration JSON that so101_driver reads.
Find the port with `ls /dev/ttyACM*` (unplug/replug to see which one is the follower's board).
"""
import argparse
import os
import select
import sys
import time

from so101_cell.driver import DEFAULT_CALIBRATION
from so101_cell.feetech import (ID, LOCK, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT, MOTORS,
                                RESOLUTION, FeetechBus, save_calibration)


def enter_pressed():
    return select.select([sys.stdin], [], [], 0)[0] and sys.stdin.readline() is not None


def setup_motors(bus):
    """Give each servo its id, one at a time (new STS3215s all answer as id 1)."""
    for name in reversed(list(MOTORS)):
        input(f"Connect ONLY the '{name}' motor to the board, then press ENTER...")
        found = [i for i in range(1, 254) if bus.ping(i)]
        if len(found) != 1:
            print(f'  expected one motor, found ids {found}; skipped')
            continue
        bus.write(found[0], LOCK, 0)
        bus.write(found[0], ID, MOTORS[name])
        print(f"  '{name}': id {found[0]} -> {MOTORS[name]}")


def calibrate(bus, path):
    ids = list(MOTORS.values())
    missing = [i for i in ids if not bus.ping(i)]
    if missing:
        sys.exit(f'motors {missing} do not answer; check power and cables (or run with --setup-motors)')
    bus.torque(ids, False)
    bus.configure(ids, gripper_id=MOTORS['gripper'])
    for i in ids:  # reset the limits so the whole turn can be read
        bus.write(i, MIN_POSITION_LIMIT, 0)
        bus.write(i, MAX_POSITION_LIMIT, RESOLUTION - 1)
    input('Move the arm to the middle of its range (see the LeRobot SO-101 calibration picture), '
          'then press ENTER...')
    offsets = bus.set_half_turn_homings(ids)
    print('Now move every joint through its whole range by hand. Press ENTER when done.')
    lo = bus.sync_read_positions(ids)
    hi = dict(lo)
    while not enter_pressed():
        pos = bus.sync_read_positions(ids)
        lo = {i: min(lo[i], p) for i, p in pos.items()}
        hi = {i: max(hi[i], p) for i, p in pos.items()}
        print('  ' + '  '.join(f'{n} {lo[i]}..{pos[i]}..{hi[i]}' for n, i in MOTORS.items()), end='\r')
        time.sleep(0.05)
    print()
    lo[MOTORS['wrist_roll']], hi[MOTORS['wrist_roll']] = 0, RESOLUTION - 1  # turns all the way round
    calib = {}
    for name, i in MOTORS.items():
        if hi[i] - lo[i] < 100:
            sys.exit(f"'{name}' barely moved ({lo[i]}..{hi[i]}); calibration not saved, run it again")
        calib[name] = {'id': i, 'drive_mode': 0, 'homing_offset': offsets[i], 'range_min': lo[i], 'range_max': hi[i]}
    bus.write_calibration(calib)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    save_calibration(path, calib)
    print(f'Calibration saved to {path}')
    for name, c in calib.items():
        print(f"  {name:<14} offset {c['homing_offset']:+5d}  range {c['range_min']}..{c['range_max']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--file', default=DEFAULT_CALIBRATION, help=f'calibration JSON (default {DEFAULT_CALIBRATION})')
    parser.add_argument('--setup-motors', action='store_true', help='assign the motor ids first (new arm)')
    args = parser.parse_args()
    bus = FeetechBus(args.port)
    try:
        if args.setup_motors:
            setup_motors(bus)
        calibrate(bus, os.path.expanduser(args.file))
    finally:
        bus.close()


if __name__ == '__main__':
    main()
