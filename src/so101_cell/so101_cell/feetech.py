"""Minimal Feetech STS3215 bus (the SO-101's servos), plus the LeRobot calibration format.

Speaks the Feetech/SCServo protocol 0 directly over pyserial (1 Mbaud, little-endian 16-bit registers),
so the robot PC needs no LeRobot install. Calibration files are the same JSON LeRobot writes
(~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json), so an arm calibrated with
LeRobot works as is, and one calibrated here works in LeRobot.
"""
import json
import math
import termios
import time

import serial

# Register addresses (STS/SMS series control table).
ID = 5
RETURN_DELAY_TIME = 7
MIN_POSITION_LIMIT = 9
MAX_POSITION_LIMIT = 11
MAX_TORQUE_LIMIT = 16
PHASE = 18
P_COEFFICIENT = 21
D_COEFFICIENT = 22
I_COEFFICIENT = 23
PROTECTION_CURRENT = 28
HOMING_OFFSET = 31
OPERATING_MODE = 33
OVERLOAD_TORQUE = 36
TORQUE_ENABLE = 40
ACCELERATION = 41
GOAL_POSITION = 42
LOCK = 55
PRESENT_POSITION = 56
MAXIMUM_ACCELERATION = 85

# 2-byte registers among the ones above (the rest are 1 byte).
_WORD = {MIN_POSITION_LIMIT, MAX_POSITION_LIMIT, MAX_TORQUE_LIMIT, PROTECTION_CURRENT, HOMING_OFFSET,
         GOAL_POSITION, PRESENT_POSITION}

_PING, _READ, _WRITE, _SYNC_READ, _SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
RESOLUTION = 4096
HALF_TURN = (RESOLUTION - 1) // 2  # 2047

# The SO-101's motors, in URDF joint order (LeRobot's names and ids).
MOTORS = {'shoulder_pan': 1, 'shoulder_lift': 2, 'elbow_flex': 3, 'wrist_flex': 4, 'wrist_roll': 5, 'gripper': 6}


class BusError(Exception):
    pass


def _sign_magnitude(value, sign_bit=11):
    """Encode a signed int the way the STS stores Homing_Offset (bit 11 = sign)."""
    return (1 << sign_bit) | -value if value < 0 else value


def _checksum(body):
    return (~sum(body)) & 0xFF


class FeetechBus:
    def __init__(self, port, baudrate=1_000_000, timeout=0.05):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def close(self):
        self.ser.close()

    # ---------- packets ----------
    def _io(self, fn, *args):
        try:
            return fn(*args)
        except (serial.SerialException, OSError, termios.error) as e:  # e.g. the USB cable was pulled
            raise BusError(f'serial port: {e}')

    def _packet(self, body):
        self._io(self.ser.write, bytes([0xFF, 0xFF, *body, _checksum(body)]))

    def _send(self, motor_id, instruction, params=()):
        self._io(self.ser.reset_input_buffer)
        self._packet([motor_id, len(params) + 2, instruction, *params])

    def _receive(self):
        """One status packet: (id, error, params). Raises BusError on timeout or a bad checksum."""
        head = self._io(self.ser.read, 4)
        while len(head) == 4 and head[:2] != b'\xff\xff':  # resync on stray bytes
            nxt = self._io(self.ser.read, 1)
            if not nxt:
                break
            head = head[1:] + nxt
        if len(head) < 4 or head[:2] != b'\xff\xff':
            raise BusError('no reply from the servo bus (power on? right port?)')
        motor_id, length = head[2], head[3]
        rest = self._io(self.ser.read, length)
        if len(rest) < length:
            raise BusError(f'short reply from motor {motor_id}')
        error, params, checksum = rest[0], rest[1:-1], rest[-1]
        if _checksum([motor_id, length, error, *params]) != checksum:
            raise BusError(f'bad checksum from motor {motor_id}')
        return motor_id, error, bytes(params)

    def ping(self, motor_id):
        try:
            self._send(motor_id, _PING)
            return self._receive()[0] == motor_id
        except BusError:
            return False

    def read(self, motor_id, address, retries=3):
        size = 2 if address in _WORD else 1
        for attempt in range(retries):
            try:
                self._send(motor_id, _READ, (address, size))
                _, _, params = self._receive()
                return int.from_bytes(params[:size], 'little')
            except BusError:
                if attempt == retries - 1:
                    raise

    def write(self, motor_id, address, value):
        size = 2 if address in _WORD else 1
        self._send(motor_id, _WRITE, (address, *int(value).to_bytes(size, 'little')))
        self._receive()  # status packet

    def sync_read_positions(self, ids, retries=3):
        """{id: raw Present_Position} for all ``ids`` with one request."""
        for attempt in range(retries):
            try:
                self._send(0xFE, _SYNC_READ, (PRESENT_POSITION, 2, *ids))
                out = {}
                for _ in ids:
                    motor_id, _, params = self._receive()
                    out[motor_id] = int.from_bytes(params[:2], 'little')
                if set(out) == set(ids):
                    return out
            except BusError:
                pass
            time.sleep(0.002)
        raise BusError(f'sync read of motors {list(ids)} failed')

    def sync_write_goals(self, goals):
        """Write Goal_Position to several motors at once ({id: raw}); no reply."""
        params = [GOAL_POSITION, 2]
        for motor_id, raw in goals.items():
            params += [motor_id, *int(raw).to_bytes(2, 'little')]
        self._packet([0xFE, len(params) + 2, _SYNC_WRITE, *params])

    # ---------- setup ----------
    def torque(self, ids, on):
        for motor_id in ids:
            self.write(motor_id, TORQUE_ENABLE, 1 if on else 0)
            self.write(motor_id, LOCK, 1 if on else 0)

    def configure(self, ids, gripper_id=None, p=16, i=0, d=32):
        """LeRobot's SO-101 follower settings (written with the torque off)."""
        for motor_id in ids:
            self.write(motor_id, RETURN_DELAY_TIME, 0)
            self.write(motor_id, MAXIMUM_ACCELERATION, 254)
            self.write(motor_id, ACCELERATION, 254)
            phase = self.read(motor_id, PHASE)
            if phase & 0x10:  # keep positions within [0, 4095]
                self.write(motor_id, PHASE, phase & ~0x10)
            self.write(motor_id, OPERATING_MODE, 0)  # position mode
            self.write(motor_id, P_COEFFICIENT, p)
            self.write(motor_id, I_COEFFICIENT, i)
            self.write(motor_id, D_COEFFICIENT, d)
            if motor_id == gripper_id:  # don't burn the gripper servo while it squeezes a cube
                self.write(motor_id, MAX_TORQUE_LIMIT, 500)
                self.write(motor_id, PROTECTION_CURRENT, 250)
                self.write(motor_id, OVERLOAD_TORQUE, 25)

    def write_calibration(self, calib):
        """Store homing offsets and position limits in the motors (torque must be off)."""
        for c in calib.values():
            self.write(c['id'], HOMING_OFFSET, _sign_magnitude(c['homing_offset']))
            self.write(c['id'], MIN_POSITION_LIMIT, c['range_min'])
            self.write(c['id'], MAX_POSITION_LIMIT, c['range_max'])

    def set_half_turn_homings(self, ids):
        """Make each motor's current position read 2047 (the middle); returns {id: homing_offset}."""
        for motor_id in ids:  # positions must be read without an old offset
            self.write(motor_id, HOMING_OFFSET, 0)
        raw = self.sync_read_positions(ids)
        offsets = {}
        for motor_id, pos in raw.items():
            offsets[motor_id] = max(-2047, min(2047, pos - HALF_TURN))  # Feetech: present = actual - offset
            self.write(motor_id, HOMING_OFFSET, _sign_magnitude(offsets[motor_id]))
        return offsets


# ---------- calibration (LeRobot JSON) and unit conversion ----------
def load_calibration(path):
    with open(path) as f:
        calib = json.load(f)
    missing = [m for m in MOTORS if m not in calib]
    if missing:
        raise ValueError(f'{path}: no calibration for {missing}')
    return calib


def save_calibration(path, calib):
    with open(path, 'w') as f:
        json.dump(calib, f, indent=4)


class JointMap:
    """Raw servo steps <-> URDF joint angles (rad).

    Arm joints use LeRobot's DEGREES mode, which is the URDF angle: 0 at the middle of the calibrated range.
    The gripper uses LeRobot's 0..100 range, mapped linearly onto the URDF gripper limits.
    """

    def __init__(self, calib, gripper_limits=(-0.174533, 1.74533)):
        self.calib = calib
        self.gripper_limits = gripper_limits

    def to_rad(self, joint, raw):
        c = self.calib[joint]
        lo, hi = c['range_min'], c['range_max']
        if joint == 'gripper':
            frac = (raw - lo) / (hi - lo)
            frac = 1.0 - frac if c.get('drive_mode') else frac
            return self.gripper_limits[0] + frac * (self.gripper_limits[1] - self.gripper_limits[0])
        deg = (raw - (lo + hi) / 2.0) * 360.0 / (RESOLUTION - 1)
        return math.radians(-deg if c.get('drive_mode') else deg)

    def limits(self, joint):
        """(lowest, highest) angle in rad the calibrated range allows."""
        c = self.calib[joint]
        a, b = self.to_rad(joint, c['range_min']), self.to_rad(joint, c['range_max'])
        return min(a, b), max(a, b)

    def to_raw(self, joint, rad):
        c = self.calib[joint]
        lo, hi = c['range_min'], c['range_max']
        if joint == 'gripper':
            g_lo, g_hi = self.gripper_limits
            frac = (rad - g_lo) / (g_hi - g_lo)
            frac = 1.0 - frac if c.get('drive_mode') else frac
            raw = lo + frac * (hi - lo)
        else:
            deg = math.degrees(-rad if c.get('drive_mode') else rad)
            raw = (lo + hi) / 2.0 + deg * (RESOLUTION - 1) / 360.0
        return int(round(min(max(raw, lo), hi)))
