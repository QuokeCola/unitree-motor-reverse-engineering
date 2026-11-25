#!/usr/bin/env python3
"""!
@file hardware_test.py
@brief Live test on motors with no controller on the bus.

Usage: hardware_test.py [port]
Scans, enables every motor found, holds position for 2 s, restarts one motor
and checks it comes back. The joints are held in place, not moved.
"""
import sys
import time

import g1_joint_motor_sdk as sdk

## Number of failed checks.
_failures = 0


def check(condition: bool, text: str) -> None:
    """!
    @brief Records and prints a failed check.
    @param condition Result of the check.
    @param text What was checked.
    """
    global _failures
    if not condition:
        _failures += 1
        print("FAILED:", text)


def main() -> int:
    """!
    @brief Runs the test.
    @return Exit code; 0 if every check passed.
    """
    port = sys.argv[1] if len(sys.argv) > 1 else None
    with sdk.MotorBus(port) as bus:
        found = bus.scan()
        check(bool(found), "at least one motor answers")
        if not found:
            return 1
        ids = sorted(found)
        for motor_id, state in found.items():
            check(state.mode == sdk.MODE_IDLE, "idle reply reports mode 0")
            check(10 < state.voltage < 60, "plausible bus voltage")
            check(state.temp_coil >= state.temp_shell, "coil >= shell")
            print(f"id {motor_id}: pos {state.pos}, {state.voltage} V, "
                  f"status 0x{state.error:x}")
        if any(state.fault for state in found.values()):
            bus.restart(ids)

        absent = (ids[0] + 7) % 15
        if absent not in found:
            check(bus.transfer(sdk.MotorCmd(id=absent)) is None,
                  "an absent id stays silent")

        start = bus.enable(ids)
        check(all(state.mode == sdk.MODE_RUN for state in start.values()),
              "enable reaches run mode")

        worst_error = 0
        cycles = 0
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            for motor_id in ids:
                state = bus.command(motor_id, pos=start[motor_id].pos, kp=300,
                                    kd=40 if motor_id == 4 else 80)
                check(state is not None, "command gets a reply")
                if state:
                    worst_error = max(worst_error,
                                      abs(state.pos - start[motor_id].pos))
            cycles += 1
        print(f"hold: largest error {worst_error} counts, "
              f"{cycles / 2:.0f} exchanges/s per motor")
        check(worst_error < 600, "position hold stays on target")
        check(cycles / 2 > 100, "polling rate above 100 per second")

        bus.disable()
        idle = bus.read(ids[0])
        check(idle is not None and idle.mode == sdk.MODE_IDLE,
              "disable returns to idle")

        after = bus.restart([ids[0]])
        check(ids[0] in after, "motor answers after restart")
        if ids[0] in after:
            check(after[ids[0]].mode == sdk.MODE_IDLE
                  and not after[ids[0]].fault,
                  "restarted motor is idle with status 0")

    print("hardware_test: " + ("all checks passed" if _failures == 0
                                else f"{_failures} check(s) failed"))
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
