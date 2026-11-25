"""MotorBus: talk to G1 joint motors on a bus that has no controller on it."""
from __future__ import annotations

import time

from .protocol import (BROADCAST_ID, MODE_IDLE, MODE_RESTART, MODE_RUN, REPLY_LEN, MotorCmd, MotorState,
                       find_reply)
from .joints import Joint
from .serial_port import open_port


class BusBusyError(RuntimeError):
    """Something else (a robot controller) is already talking on the bus."""


class MotorFaultError(RuntimeError):
    """A motor would not enter run mode."""


class MotorBus:
    """One RS-485 chain of motors.

        with MotorBus() as bus:
            print(bus.scan())                 # {4: MotorState(...), ...}
            bus.enable([4, 5, 6])             # zero torque: joints move freely
            state = bus.command(4, pos=state.pos, kp=300, kd=40)
        # leaving the block returns the motors to idle, which brakes them

    Idle mode is not free: the motor brakes the joint passively. Run mode with an all-zero command
    is the free state. The bus is half duplex and motors only speak when polled, so every call that sends a
    command also returns that motor's fresh state. Call command() for every motor in a
    steady loop; about 340 exchanges per second per motor was reached from Python with
    three motors, the G1 controller does about 1,300.
    """

    def __init__(self, port: str | None = None, reply_timeout: float = 0.004, check_silent: bool = True):
        self.ser = open_port(port)
        self.reply_timeout = reply_timeout
        self.enabled: set[int] = set()
        self.limits: dict[int, tuple[int, int]] = {}
        self.joints: dict[int, Joint] = {}
        self._count: dict[int, int] = {}
        if check_silent and self.listen(0.3):
            self.close(brake=False)
            raise BusBusyError("bus is not silent: a controller is already polling the motors. "
                               "Two masters on one RS-485 bus corrupt each other; use listen() only.")

    # -- context manager ---------------------------------------------------------------
    def __enter__(self) -> "MotorBus":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self, brake: bool = True) -> None:
        if self.ser.is_open:
            try:
                if brake and self.enabled:
                    self.disable()
            finally:
                self.ser.close()

    # -- low level ---------------------------------------------------------------------
    def listen(self, seconds: float) -> bytes:
        """Receive without transmitting."""
        end, buf = time.monotonic() + seconds, b""
        while time.monotonic() < end:
            buf += self.ser.read(4096)
        return buf

    def transfer(self, cmd: MotorCmd) -> MotorState | None:
        """Send one frame and wait for that motor's reply. None on timeout or bad checksum."""
        self.ser.reset_input_buffer()
        self.ser.write(cmd.encode())
        end, rx = time.monotonic() + self.reply_timeout, b""
        while time.monotonic() < end:
            rx += self.ser.read(256)
            if len(rx) >= REPLY_LEN:
                state = find_reply(rx)
                if state:
                    return state if state.id == cmd.id else None
        return None

    def _byte3(self, motor_id: int, flag: bool) -> int:
        # The G1 controller toggles this byte between 0x80 and 0x90 about every 8 frames once
        # polling is running. A motor enters run mode without it and without the flag bit; it is
        # reproduced as seen because every motion test so far was done with it.
        n = self._count.get(motor_id, 0)
        self._count[motor_id] = n + 1
        if not flag:
            return 0
        return 0x80 if (n // 8) % 2 == 0 else 0x90

    # -- discovery and state -----------------------------------------------------------
    def scan(self, ids=range(BROADCAST_ID), tries: int = 3) -> dict[int, MotorState]:
        """Poll ids with the idle frame (mode 0, all zero) and return who answers."""
        found = {}
        for motor_id in ids:
            for _ in range(tries):
                state = self.transfer(MotorCmd(id=motor_id))
                if state:
                    found[motor_id] = state
                    break
        return found

    def read(self, motor_id: int) -> MotorState | None:
        """Fresh state without changing what the motor is doing.

        An idle motor gets the idle poll; an enabled motor gets a zero-torque frame, which
        DROPS any position hold. Use command() to keep holding.
        """
        if motor_id in self.enabled:
            return self.command(motor_id)
        return self.transfer(MotorCmd(id=motor_id))

    # -- mode --------------------------------------------------------------------------
    def enable(self, ids) -> dict[int, MotorState]:
        """Put motors in run mode with zero torque, in the order the G1 controller uses at
        boot: idle polls, then the flag bit, then mode 1. Joints are free to move afterwards."""
        ids = list(ids)
        last: dict[int, MotorState] = {}
        for flag, mode, cycles in ((False, MODE_IDLE, 20), (True, MODE_IDLE, 50), (True, MODE_RUN, 5)):
            for _ in range(cycles):
                for i in ids:
                    state = self.transfer(MotorCmd(id=i, mode=mode, flag=flag, byte3=self._byte3(i, flag)))
                    if state:
                        last[i] = state
        for i in ids:
            if i not in last or last[i].mode != MODE_RUN:
                for _ in range(20):
                    for j in ids:
                        self.transfer(MotorCmd(id=j))
                if i not in last:
                    raise MotorFaultError(f"motor {i} did not answer")
                why = (f"it reports status word 0x{last[i].error:08x}; call restart() to clear it (positions may come "
                       f"back shifted by whole output turns)" if last[i].fault else "no fault reported")
                raise MotorFaultError(f"motor {i} did not enter run mode: {why}")
        self.enabled.update(ids)
        try:
            for i in ids:
                if i in self.joints:
                    self.joints[i].sync(last[i].pos)
        except ValueError as e:
            self.disable(ids)
            raise MotorFaultError(f"motor {i}: {e}; re-measure the end stops") from None
        return last

    def disable(self, ids=None) -> None:
        """Return motors to idle mode. This is also the way to stop a moving joint.

        Idle brakes the joint passively: from top speed (about 1.27 M counts/s) three wrist joints
        stopped in 18-27 ms with the supply voltage steady and no fault. Do not brake a fast joint
        with a damping command on a bench supply: kd 80 sagged a 24 V supply to 15 V and latched
        fault 0x8, and a gentler damping brake pumped it to 30 V until it cut out. A motor that is
        no longer polled drops to idle by itself after 0.5-1 s.
        """
        ids = list(self.enabled if ids is None else ids)
        for _ in range(20):
            for i in ids:
                self.transfer(MotorCmd(id=i))
        self.enabled.difference_update(ids)

    def restart(self, ids, wait: float = 0.5) -> dict[int, MotorState]:
        """Restart motors (mode 7). Clears a latched fault such as 0x8.

        The motor stops answering for about 0.3 s and comes back idle with status 0. Its position
        reading may come back shifted by a whole number of output-shaft turns (16 rotor turns =
        524,288 counts on one wrist motor, 25 turns = 819,200 on another), so re-check end stops and
        any stored positions afterwards. Returns the state of each motor after the restart.
        """
        ids = list(ids)
        for i in ids:
            self.transfer(MotorCmd(id=i, mode=MODE_RESTART))
        self.enabled.difference_update(ids)
        time.sleep(wait)
        after = {}
        for i in ids:
            for _ in range(5):
                state = self.transfer(MotorCmd(id=i))
                if state:
                    after[i] = state
                    break
        for i, state in after.items():
            if i in self.joints:
                try:
                    self.joints[i].sync(state.pos)
                except ValueError as e:
                    raise MotorFaultError(f"motor {i}: {e}; re-measure the end stops") from None
        return after

    # -- commands ----------------------------------------------------------------------
    def set_joint(self, motor_id: int, joint: Joint) -> None:
        """Attach a Joint (gear ratio, end stops, zero). enable() and restart() then re-reference it
        to the motor's position reading, and command() keeps position targets inside its end stops."""
        self.joints[motor_id] = joint

    def set_limits(self, motor_id: int, low: int, high: int) -> None:
        """Clamp every position target for this motor to [low, high] raw counts."""
        if low >= high:
            raise ValueError("low must be below high")
        self.limits[motor_id] = (low, high)

    def command(self, motor_id: int, pos: int = 0, vel: int = 0, tau: int = 0, kp: int = 0, kd: int = 0
                ) -> MotorState | None:
        """Send one hybrid command (raw counts) to an enabled motor and return its state.

        torque = tau + kp * (pos - measured) + kd * (vel - measured). All zero = zero torque.
        The motor keeps applying the last command it received, so keep calling this in a loop.
        """
        if motor_id not in self.enabled:
            raise RuntimeError(f"motor {motor_id} is not enabled; call enable() first")
        if kp and motor_id in self.joints:
            pos = self.joints[motor_id].clamp(pos)
        if kp and motor_id in self.limits:
            low, high = self.limits[motor_id]
            pos = min(max(pos, low), high)
        return self.transfer(MotorCmd(id=motor_id, mode=MODE_RUN, flag=True, byte3=self._byte3(motor_id, True),
                                      pos=pos, vel=vel, tau=tau, kp=kp, kd=kd))
