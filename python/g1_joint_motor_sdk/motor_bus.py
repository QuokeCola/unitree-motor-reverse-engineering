"""!
@file motor_bus.py
@brief MotorBus: drive G1 joint motors on a bus without a controller.
"""
from __future__ import annotations

import time
from collections.abc import Iterable

from g1_joint_motor_sdk import joint as joint_lib
from g1_joint_motor_sdk import protocol
from g1_joint_motor_sdk import serial_port


class BusBusyError(RuntimeError):
    """!
    @brief Raised when a controller is already polling the bus.
    """


class MotorFaultError(RuntimeError):
    """!
    @brief Raised when a motor does not answer or will not enter run mode.
    """


class MotorBus:
    """!
    @brief One RS-485 chain of motors, with this program as the only master.

    @code{.py}
    with MotorBus() as bus:
        start = bus.enable([4, 5, 6])          # run mode, joints free
        bus.command(4, pos=start[4].pos, kp=300, kd=40)   # in a steady loop
    @endcode

    Motors answer only when polled, so every command returns that motor's
    fresh state. A motor that is no longer polled drops to idle after
    0.5-1 s. Leaving the `with` block returns enabled motors to idle.
    """

    def __init__(self, port: str | None = None, reply_timeout: float = 0.004,
                 check_silent: bool = True):
        """!
        @brief Opens the adapter at 6 Mbaud.
        @param port Device name; None selects the only FTDI adapter.
        @param reply_timeout Longest wait for a reply, in seconds.
        @param check_silent Listen first and refuse a bus that has traffic.
        @exception BusBusyError The bus is not silent.
        @exception RuntimeError The port cannot be opened.
        """
        ## The open pyserial port.
        self._serial = serial_port.open_port(port)
        ## Longest wait for a reply, in seconds.
        self._reply_timeout = reply_timeout
        ## Frames sent per motor, for the sequence byte.
        self._frame_count: dict[int, int] = {}
        ## Ids of the motors enabled through this object.
        self.enabled: set[int] = set()
        ## Joints attached with set_joint(), by motor id.
        self.joints: dict[int, joint_lib.Joint] = {}
        if check_silent and self.listen(0.3):
            self._serial.close()
            raise BusBusyError("bus is not silent: a controller is already "
                               "polling the motors")

    def __enter__(self) -> MotorBus:
        """!
        @brief Enters a `with` block.
        @return This object.
        """
        return self

    def __exit__(self, *exc_info) -> None:
        """!
        @brief Leaves a `with` block and closes the bus.
        """
        self.close()

    def close(self) -> None:
        """!
        @brief Returns enabled motors to idle and closes the port.
        """
        if not self._serial.is_open:
            return
        try:
            if self.enabled:
                self.disable()
        finally:
            self._serial.close()

    def listen(self, seconds: float) -> bytes:
        """!
        @brief Receives without transmitting.
        @param seconds How long to listen.
        @return The raw bytes received.
        """
        end = time.monotonic() + seconds
        received = b""
        while time.monotonic() < end:
            received += self._serial.read(4096)
        return received

    def transfer(self, cmd: protocol.MotorCmd) -> protocol.MotorState | None:
        """!
        @brief Sends one frame and waits for that motor's reply.
        @param cmd Command to send.
        @return The reply, or None on timeout or a bad checksum.
        """
        self._serial.reset_input_buffer()
        self._serial.write(cmd.encode())
        end = time.monotonic() + self._reply_timeout
        received = b""
        while time.monotonic() < end:
            received += self._serial.read(256)
            if len(received) < protocol.REPLY_LEN:
                continue
            state = protocol.find_reply(received)
            if state:
                return state if state.id == cmd.id else None
        return None

    def scan(self, ids: Iterable[int] = range(protocol.BROADCAST_ID),
             tries: int = 3) -> dict[int, protocol.MotorState]:
        """!
        @brief Polls ids with the idle frame.
        @param ids Ids to poll; all of 0-14 by default.
        @param tries Attempts per id.
        @return The motors that answered, by id.
        """
        found = {}
        for motor_id in ids:
            for _ in range(tries):
                state = self.transfer(protocol.MotorCmd(id=motor_id))
                if state:
                    found[motor_id] = state
                    break
        return found

    def read(self, motor_id: int) -> protocol.MotorState | None:
        """!
        @brief Reads a motor's state.

        On an enabled motor this sends a zero-torque command, which drops a
        position hold.

        @param motor_id Motor to read.
        @return The state, or None if it did not reply.
        """
        if motor_id in self.enabled:
            return self.command(motor_id)
        return self.transfer(protocol.MotorCmd(id=motor_id))

    def enable(self, ids: Iterable[int]) -> dict[int, protocol.MotorState]:
        """!
        @brief Puts motors in run mode with zero torque; the joints are free.

        Joints attached with set_joint() are re-referenced to the reading.

        @param ids Motors to enable.
        @return The state of each motor.
        @exception MotorFaultError A motor does not reach run mode.
        """
        ids = list(ids)
        last: dict[int, protocol.MotorState] = {}
        # Same order as the G1 controller at boot: idle, flag bit, run mode.
        steps = ((False, protocol.MODE_IDLE, 20), (True, protocol.MODE_IDLE, 50),
                 (True, protocol.MODE_RUN, 5))
        for flag, mode, cycles in steps:
            for _ in range(cycles):
                for motor_id in ids:
                    state = self.transfer(protocol.MotorCmd(
                        id=motor_id, mode=mode, flag=flag,
                        byte3=self._next_byte3(motor_id, flag)))
                    if state:
                        last[motor_id] = state
        for motor_id in ids:
            state = last.get(motor_id)
            if state is None:
                self._send_idle(ids, 20)
                raise MotorFaultError(f"motor {motor_id} did not answer")
            if state.mode != protocol.MODE_RUN:
                self._send_idle(ids, 20)
                raise MotorFaultError(
                    f"motor {motor_id} did not enter run mode (status "
                    f"0x{state.error:08x}); restart() clears a latched fault")
        self.enabled.update(ids)
        for motor_id in ids:
            if motor_id not in self.joints:
                continue
            try:
                self.joints[motor_id].sync(last[motor_id].pos)
            except ValueError as error:
                self.disable(ids)
                raise MotorFaultError(f"motor {motor_id}: {error}") from None
        return last

    def disable(self, ids: Iterable[int] | None = None) -> None:
        """!
        @brief Returns motors to idle, which brakes the joint passively.

        This is also the way to stop a moving joint.

        @param ids Motors to idle; None means every enabled motor.
        """
        ids = list(self.enabled if ids is None else ids)
        self._send_idle(ids, 20)
        self.enabled.difference_update(ids)

    def restart(self, ids: Iterable[int],
                wait: float = 0.5) -> dict[int, protocol.MotorState]:
        """!
        @brief Restarts motors (mode 7), clearing a latched fault.

        The position reading may come back shifted by whole output turns;
        attached joints are re-referenced.

        @param ids Motors to restart.
        @param wait Seconds allowed for the restart (about 0.3 s needed).
        @return The state of each motor after the restart.
        @exception MotorFaultError An attached joint no longer fits.
        """
        ids = list(ids)
        for motor_id in ids:
            self.transfer(protocol.MotorCmd(id=motor_id,
                                            mode=protocol.MODE_RESTART))
        self.enabled.difference_update(ids)
        time.sleep(wait)
        after = {}
        for motor_id in ids:
            for _ in range(5):
                state = self.transfer(protocol.MotorCmd(id=motor_id))
                if state:
                    after[motor_id] = state
                    break
        for motor_id, state in after.items():
            if motor_id not in self.joints:
                continue
            try:
                self.joints[motor_id].sync(state.pos)
            except ValueError as error:
                raise MotorFaultError(f"motor {motor_id}: {error}") from None
        return after

    def command(self, motor_id: int, pos: int = 0, vel: int = 0, tau: int = 0,
                kp: int = 0, kd: int = 0) -> protocol.MotorState | None:
        """!
        @brief Sends one hybrid command, in raw counts, to an enabled motor.

        Position targets are limited to the end stops of an attached joint.

        @param motor_id Motor to command.
        @param pos Position target.
        @param vel Velocity target.
        @param tau Feed-forward torque.
        @param kp Position gain.
        @param kd Damping gain.
        @return The motor's state, or None if it did not reply.
        @exception RuntimeError The motor is not enabled.
        """
        if motor_id not in self.enabled:
            raise RuntimeError(f"motor {motor_id} is not enabled; call "
                               "enable() first")
        if kp and motor_id in self.joints:
            pos = self.joints[motor_id].clamp(pos)
        return self.transfer(protocol.MotorCmd(
            id=motor_id, mode=protocol.MODE_RUN, flag=True,
            byte3=self._next_byte3(motor_id, True), pos=pos, vel=vel, tau=tau,
            kp=kp, kd=kd))

    def set_joint(self, motor_id: int, joint: joint_lib.Joint) -> None:
        """!
        @brief Attaches a joint description to a motor.
        @param motor_id Motor the joint belongs to.
        @param joint Gear ratio, end stops and zero.
        """
        self.joints[motor_id] = joint

    def _next_byte3(self, motor_id: int, flag: bool) -> int:
        """!
        @brief Sequence byte as the G1 controller sends it.
        @param motor_id Motor the frame is for.
        @param flag Whether the flag bit is set in this frame.
        @return 0 without the flag, else 0x80 / 0x90 changing every 8 frames.
        """
        count = self._frame_count.get(motor_id, 0)
        self._frame_count[motor_id] = count + 1
        if not flag:
            return 0
        return 0x80 if (count // 8) % 2 == 0 else 0x90

    def _send_idle(self, ids: Iterable[int], cycles: int) -> None:
        """!
        @brief Sends idle polls to the given motors.
        @param ids Motors to poll.
        @param cycles Number of rounds.
        """
        ids = list(ids)
        for _ in range(cycles):
            for motor_id in ids:
                self.transfer(protocol.MotorCmd(id=motor_id))
