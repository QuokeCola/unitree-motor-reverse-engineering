"""!
@file motor_bus.py
@brief MotorBus: drive G1 joint motors on a bus without a controller.
"""
from __future__ import annotations

import time
from collections.abc import Iterable

from g1motor import joint as joint_lib
from g1motor import protocol
from g1motor import serial_port


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
        bus.command(4, start[4].pos, 0, 0, 300, 40)   # in a steady loop
    @endcode

    Motors answer only when polled, so every command returns that motor's
    fresh state. A motor that is no longer polled drops to idle after
    0.5-1 s. Leaving the `with` block returns enabled motors to idle.
    """

    def __init__(self, port: str = "", reply_timeout: float = 0.004,
                 check_silent: bool = True):
        """!
        @brief Opens the adapter at 6 Mbaud.
        @param port Device name; empty selects the only FTDI adapter.
        @param reply_timeout Longest wait for a reply, in seconds.
        @param check_silent Listen first and refuse a bus that has traffic.
        @exception BusBusyError The bus is not silent.
        @exception RuntimeError The port cannot be opened.
        """
        ## The open serial port.
        self._port = serial_port.SerialPort()
        ## Longest wait for a reply, in seconds.
        self._reply_timeout = reply_timeout
        ## Motors enabled here.
        self._enabled: set[int] = set()
        ## Joints attached, by id.
        self._joints: dict[int, joint_lib.Joint] = {}
        ## Frames sent per motor.
        self._frame_count: dict[int, int] = {}
        self._port.open(port or serial_port.SerialPort.find_port(),
                        protocol.BAUD)
        if check_silent and self.listen(0.3):
            self._port.close()
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
        if not self._port.is_open():
            return
        try:
            if self._enabled:
                self.disable()
        finally:
            self._port.close()

    def listen(self, seconds: float) -> bytes:
        """!
        @brief Receives without transmitting.
        @param seconds How long to listen.
        @return The raw bytes received.
        """
        end = time.monotonic() + seconds
        received = b""
        while time.monotonic() < end:
            received += self._port.read(4096, 0.001)
        return received

    def transfer(self, cmd: protocol.MotorCmd) -> protocol.MotorState | None:
        """!
        @brief Sends one frame and waits for that motor's reply.
        @param cmd Command to send.
        @return The reply, or None on timeout or a bad checksum.
        """
        self._port.flush_input()
        self._port.write(cmd.encode())
        end = time.monotonic() + self._reply_timeout
        received = b""
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return None
            received += self._port.read(256, left)
            if len(received) < protocol.REPLY_LEN:
                continue
            state = protocol.find_reply(received)
            if state:
                return state if state.id == cmd.id else None

    def scan(self, tries: int = 3) -> dict[int, protocol.MotorState]:
        """!
        @brief Polls ids 0-14 with the idle frame.
        @param tries Attempts per id.
        @return The motors that answered, by id.
        """
        found = {}
        for motor_id in range(protocol.BROADCAST_ID):
            for _ in range(tries):
                state = self.transfer(protocol.MotorCmd(id=motor_id))
                if state:
                    found[motor_id] = state
                    break
        return found

    def read(self, id: int) -> protocol.MotorState | None:
        """!
        @brief Reads a motor's state.

        On an enabled motor this sends a zero-torque command, which drops a
        position hold.

        @param id Motor to read.
        @return The state, or None if it did not reply.
        """
        if self.is_enabled(id):
            return self.command(id)
        return self.transfer(protocol.MotorCmd(id=id))

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
        self._enabled.update(ids)
        for motor_id in ids:
            if motor_id not in self._joints:
                continue
            try:
                self._joints[motor_id].sync(last[motor_id].pos)
            except ValueError as error:
                self.disable(ids)
                raise MotorFaultError(f"motor {motor_id}: {error}") from None
        return last

    def disable(self, ids: Iterable[int] = ()) -> None:
        """!
        @brief Returns motors to idle, which brakes the joint passively.

        This is also the way to stop a moving joint.

        @param ids Motors to idle; empty means every enabled motor.
        """
        ids = list(ids) or list(self._enabled)
        self._send_idle(ids, 20)
        self._enabled.difference_update(ids)

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
        self._enabled.difference_update(ids)
        time.sleep(wait)
        after = {}
        for motor_id in ids:
            for _ in range(5):
                state = self.transfer(protocol.MotorCmd(id=motor_id))
                if state:
                    after[motor_id] = state
                    break
        for motor_id, state in after.items():
            if motor_id not in self._joints:
                continue
            try:
                self._joints[motor_id].sync(state.pos)
            except ValueError as error:
                raise MotorFaultError(f"motor {motor_id}: {error}") from None
        return after

    def command(self, id: int, pos: int = 0, vel: int = 0, tau: int = 0,
                kp: int = 0, kd: int = 0) -> protocol.MotorState | None:
        """!
        @brief Sends one hybrid command, in raw counts, to an enabled motor.

        Position targets are limited to the end stops of an attached joint.

        @param id Motor to command.
        @param pos Position target.
        @param vel Velocity target.
        @param tau Feed-forward torque.
        @param kp Position gain.
        @param kd Damping gain.
        @return The motor's state, or None if it did not reply.
        @exception RuntimeError The motor is not enabled.
        """
        if not self.is_enabled(id):
            raise RuntimeError(f"motor {id} is not enabled; call enable() "
                               "first")
        if kp and id in self._joints:
            pos = self._joints[id].clamp(pos)
        return self.transfer(protocol.MotorCmd(
            id=id, mode=protocol.MODE_RUN, flag=True,
            byte3=self._next_byte3(id, True), pos=pos, vel=vel, tau=tau,
            kp=kp, kd=kd))

    def set_joint(self, id: int, joint: joint_lib.Joint) -> None:
        """!
        @brief Attaches a joint description to a motor.
        @param id Motor the joint belongs to.
        @param joint Gear ratio, end stops and zero.
        """
        self._joints[id] = joint

    def joint(self, id: int) -> joint_lib.Joint:
        """!
        @brief The joint attached to a motor.
        @param id Motor id.
        @return The joint, as re-referenced by enable() and restart().
        @exception KeyError None is attached.
        """
        return self._joints[id]

    def is_enabled(self, id: int) -> bool:
        """!
        @brief Whether a motor was enabled through this object.
        @param id Motor id.
        @return True if it is enabled.
        """
        return id in self._enabled

    def _next_byte3(self, id: int, flag: bool) -> int:
        """!
        @brief Sequence byte as the G1 controller sends it.
        @param id Motor the frame is for.
        @param flag Whether the flag bit is set in this frame.
        @return 0 without the flag, else 0x80 / 0x90 changing every 8 frames.
        """
        count = self._frame_count.get(id, 0)
        self._frame_count[id] = count + 1
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
