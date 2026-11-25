# G1 arm motor bus (observed 2026-11-25)

Passive capture on one arm chain of a Unitree G1, FT232H adapter. Nothing transmitted.
Status: field names are inferred from the public GO-M8010-6 layout and from how
values moved between robot modes. Scales and units are not known yet.

## Physical

- RS-485, 6,000,000 baud, 8N1 (4M / 4.8M / 5M / 8M / 12M all give garbage)
- 5 motors, ids 0-4, polled in turn, about 1,300 command/reply pairs per second per motor
- pyserial cannot set 6M on this machine; `scripts/sniff.py` sets it with TCSETS2/BOTHER

## Command, host -> motor, 20 bytes (little-endian)

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | header | `FE EE` |
| 2 | 1 | id / mode | bits 0-3 id, bits 4-6 mode (0 = idle, 1 = running), bit 7 flag set a few seconds into boot |
| 3 | 1 | unknown | motors 0,1,4: toggles `80`/`90`; motors 2,3: low bits count up by 1 about every 8 frames, bit 7 toggles |
| 4 | 2 | torque ff (int16) | always 0 so far |
| 6 | 2 | velocity (int16) | always 0 so far |
| 8 | 4 | position (int32) | same units as reply position |
| 12 | 2 | kp (int16) | |
| 14 | 2 | kd (int16) | |
| 16 | 4 | checksum | Unitree `crc32_core` over bytes 0-15 (header included) |

## Reply, motor -> host, 26 bytes

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | header | `FC EE` (public SDK uses `FD EE`) |
| 2 | 1 | id / mode | bits 0-3 id, bits 4-6 mode the motor is in (0 or 1) |
| 3 | 1 | coil (winding) temperature, deg C (uint8) | 28-39 seen; rises about 3 over 5 min on the robot. Identification from the user (2026-11-25) |
| 4 | 1 | shell temperature, deg C (uint8) | never above byte 3 in our data: 0-5 lower. 4-5 lower just after a cold power-on; equal to byte 3 right after a mode-7 restart |
| 5 | 1 | bus voltage (uint8), 0.5 V/count | 96-97 on the robot (about 48 V); 48 on a bench arm fed 24 V |
| 6 | 2 | torque (int16) | guess; small while holding, up to 87 on motor 0 |
| 8 | 2 | velocity (int16) | non-zero only while position moves; one segment gave about 194 position counts per velocity count per second |
| 10 | 4 | position (int32) | matches command position when holding |
| 14 | 4 | status / fault word | 0 normally; `0x8` latched after the 12.3x abort (see bench notes); `00 80 xx xx` / `.. .. 04 00` during robot boot |
| 18 | 2 | unknown (uint16) | 2,950-3,040 on motors 0-3, 2,610-2,730 on motor 4; noisy, slow drift |
| 20 | 1 | unknown | 0 |
| 21 | 1 | unknown | 0 on motors 0,1,4; changes every frame on motors 2,3 |
| 22 | 4 | checksum | Unitree `crc32_core` over bytes 2-21 (header NOT included) |

## Checksum

Same `crc32_core` as the public A1/B1 SDK: init `0xFFFFFFFF`, polynomial `0x04C11DB7`,
no reflection, no final XOR, fed 32-bit little-endian words most-significant bit first
(equal to CRC-32/MPEG-2 with the bytes of each word reversed). Stored little-endian.
Checked on capture 122254: 1,969,826 of 1,969,826 commands and 1,969,825 of 1,969,826
replies match. Vectorised implementation: `crc32_words` in `scripts/monitor.py`.

## Boot sequence (capture 124403_boot_6000000, times from first bus activity)

| Time | What the controller sends | Motor replies |
|---|---|---|
| 0 s | 18 frames (3-4 per motor): byte 2 = id, byte 3 = `00`, torque field = `80 ff` (-128), rest 0 | mode 0. Motors 0,1,4: normal telemetry. Motors 2,3: a fixed blob in bytes 3-13, bytes 14-15 = `00 80` |
| 0 - 7.0 s | silence | |
| 7.0 s | continuous polling starts: byte 2 = id (mode 0), payload 0, byte 3 starts toggling/counting | mode 0 |
| 7.0 - 7.1 s | motors 2 and 3 only: about 114 frames carrying a fixed 12-byte payload (below) | blob until 7.1 s, then normal telemetry; byte 16 = `04` until 7.5 s |
| 9.6 s | byte 2 = `0x80 + id` (bit 7 set, mode still 0) | mode 0 |
| 33.2 s | byte 2 = `0x90 + id` (mode 1) on all five motors in the same cycle | byte 2 = `0x10 + id` from the very next frame |

Fixed payloads (command bytes 4-15) and reply blobs (reply bytes 3-13) in this boot:

- motor 2: payload `bf 72 d1 6f 56 15 42 2b 41 51 c8 df`, blob `ff ff ef 25 fc 2f f2 1a d4 fd d5`
- motor 3: payload `2f a2 9a dd 55 f5 4c 09 03 f5 04 7e`, blob `ff e8 12 eb d7 f7 ef 5d 67 ff b5`

Not known yet: whether these are the same on every boot (static key or calibration data)
or a challenge and response. A second boot capture answers that.

## Robot modes seen (capture 122254_powercycle_6000000)

| Mode | Command content |
|---|---|
| Zero torque (after boot) | all payload fields 0 |
| Damping | kd only: m0 = 25, m1 = 25, m3 = 1, m2 = m4 = 0 |
| Locked standing | position = current position, kp/kd: m0 450/10, m1 300/5, m2 100/2, m3 300/5, m4 20/1 |

No extra frames at any mode change; only the payload fields change.

## Open

- Meaning of the motor 2/3 boot payload and blob; of command bit 7; of the -128 probe.
- Command byte 3, reply unknowns, absolute scales and gear ratios.

## Bench arm without controller (2026-11-25)

Separate arm, 24 V supply, no controller. Bus silent. Polled ids 0-14 with the idle frame
(mode 0, all fields zero, byte 3 = 0) using `scripts/scan.py`: ids 4, 5 and 6 answered with
normal telemetry in mode 0, first try, no init needed. So a motor answers a bare mode-0 poll.

### Driving the bench arm (same day)

- Mode 1 with all fields zero = zero torque; joints move freely by hand. Entered with the
  controller's order (mode 0, flag bit, mode 1) and byte 3 toggling `80`/`90` every 8 polls.
  About 340 polls/s per motor was enough. Whether byte 3 or the flag bit are required is untested.
- End stops found by hand (counts): id 4 -95,045..235,962; id 5 322..426,057; id 6 -190,234..236,866.
- Position control works: mode 1, position ramp + kp/kd (+ velocity = ramp speed / 128).
  kp 20/kd 1 and kp 100/kd 2 were too weak for ids 5, 6; kp 300/kd 5 swept all three joints
  over 80% of their range at 50,000 counts/s. Peak lag / torque reading: id 4 1,132 / 13,
  id 5 4,661 / 59, id 6 8,251 / 94. Scripts: `scripts/zero_torque.py`, `scripts/sweep.py`.
- Reported torque is not proportional to kp x error across these runs (kp 20: ~9 counts at
  5,000 error; kp 100: ~24), so the gain scale is still open.
- Smoothness tuning on an 8 s sine (peak about 134,000 counts/s), kp 300, velocity ripple as %
  of peak speed for ids 4 / 5 / 6: kd 5: 13.7 / 8.2 / 11.5; kd 10: 8.1 / 6.7 / 9.6;
  kd 20: 5.3 / 4.9 / 6.0; kd 40: 4.2 / 4.0 / 4.7; kd 80: 3.6 / 3.2 / 3.5. At kd 80, id 4's
  torque reading chatters (peak 37 vs 14-20), so kd 40 / 80 / 80 was kept: 3.8 / 3.0 / 3.7.
  The velocity feed-forward must be the exact derivative of the target; differencing targets
  over uneven poll times adds ripple. Report tool: `scripts/smoothness.py`.
- Friction feed-forward through the command torque field works, and its units match the reply
  torque reading closely enough to use fitted values directly. Fit on the kd 40/80/80 sine
  (torque counts): id 4 3.7 sign(v); id 5 18.7 sign(v) + 1.5 v/1e5; id 6 11.6 sign(v) + 6.0 v/1e5
  (v in position counts/s). Applied as tanh(v/5000) for the sign. Lag rms without / half / full
  feed-forward: id 4 719 / 546 / 387; id 5 2,213 / 1,570 / 972; id 6 1,707 / 1,102 / 544.
  Velocity ripple did not change (about 3-4%). The remaining lag ripple repeats with position
  from one pass to the next (correlation 0.83 / 0.80 / 0.99), so it is position-dependent.
- Position-dependent feed-forward: `scripts/build_ff_table.py` averages the reported torque in
  1,024-count position bins per direction from a slow sine (16 s period) and writes
  `calibration/ff_table_ids456.json`; `sweep.py --ff-table` applies it. On the 8 s sine, lag rms
  with the simple friction model vs the table: id 4 387 / 438, id 5 972 / 375, id 6 544 / 161.
  Id 6 has a stiff region needing about 80 torque counts.
- Speed: with kp 300, kd 40/80/80, table + viscous feed-forward, the full-range sine ran cleanly
  at periods 4 s, 2.4 s and 1.6 s (peak 670,000 counts/s, about 20 rotor turns/s on the assumed
  scale). At 1.6 s: lag rms 649 / 1,086 / 808, peak 1,137 / 2,578 / 1,703, peak torque reading
  38 / 62 / 90, velocity ripple 3.9 / 2.4 / 2.8 %. On abort, sweep.py now brakes with pure
  damping for about 0.5 s before going idle.
- Speed ceiling on 24 V: at sine periods 0.8 s and 0.65 s all three motors peaked at about
  1,275,000 position counts/s (velocity reading about 9,960; roughly 39 rotor turns/s on the
  assumed scale) whatever was commanded (up to 2.0 M). Same value on ids 4, 5, 6, so it is a
  motor/drive limit (supply voltage or a firmware cap, untested which). Period 0.8 s (10x) still
  tracked (peak lag 8,656); 0.65 s aborted on lag (25,608) and the damping brake stopped it cleanly.
- Fault latch: after the 12.3x run (lag abort, then the 0.5 s kd-80 damping brake from about
  1.27 M counts/s) all three bench motors report status word `0x8` and refuse run mode: mode-1
  frames are answered in mode 0 whatever byte 3 and the flag bit are (2,000 frames each, seven
  patterns tried on id 4). Idle polling does not clear it. Not known which event set it; hard
  regenerative braking raising the 24 V bench supply is one candidate. Needs a power cycle test.
- Power cycle (24 V off about 10 s) cleared status `0x8` on all three motors. Positions read
  87,953 / 175,252 / 79,648 afterwards (multi-turn values, inside the old end stops); whether the
  count survives power-off unchanged is being re-checked with a new hand sweep. SDK `enable`,
  zero torque, 2 s position hold (error at most 18 counts) and `disable` then worked.

### Fault 0x8: cause and what does not clear it (2026-11-25, second occurrence)

- Reproduced with a 20x full-range sine (lag abort) followed by the kd-80 damping brake. Logged per
  frame: the status word turned `0x8` on the first or second brake frame, on all three motors within
  2 ms. In those frames the torque reading jumped to 419 / 689 / 494 counts and the bus voltage
  reading DROPPED from 24.0 to 19.0 / 15.0 / 22.0 V, then recovered within about 10 ms. So the fault
  follows a supply sag under a large current demand (under-voltage or over-current; not over-voltage
  from regeneration). Up to then the run itself (speed at the ceiling, lag 25,000) had not faulted.
- Braking torque scale from the same log: torque reading is about 9e-4 x kd x velocity counts.
- Did NOT clear it (status and enable checked after each, all three motors): run-mode damping
  (0x9N, kd 25) for 1 s; idle + flag with kd 25 for 1 s; the boot probe (0x0N, byte 3 = 0,
  torque -128) for 4 rounds and for 1 s; boot probe then 7 s of silence; 15 s of silence.
  Script: `scripts/fault_clear_test.py`. Mode values 2-7 were not tried.
- Position zero after power-on: id 4's end stops moved from -95,045..235,962 to 430,029..760,923,
  a shift of 525,074 counts = 16.02 rotor turns (16 turns = 524,288). Ids 5 and 6 did not shift.
  The earlier power cycle shifted nothing. Reading: position at power-on is only known modulo
  16 rotor turns on id 4, i.e. probably one output revolution with a 16:1 gear (one data point).
  End stops are now measured after every power-on and stored in `calibration/limits.json`.

### Mode values probed on bench id 4 (2026-11-25), zero payload, one at a time

| Mode | Result |
|---|---|
| 2 | ignored, reply stays mode 0, no motion (40 frames, watched 6 s) |
| 3, 4, 5 | accepted from the first frame (reply reports the mode), back to 0 on the next idle frame; telemetry unchanged, no motion; function unknown |
| 6 | ignored, reply stays mode 0 |
| 7 | restart: answers for about 5 ms, silent until about 302 ms, then mode 0 with status 0. One frame, with or without the flag bit. Cleared `0x8` on ids 4, 5, 6 |

- After a mode-7 restart, id 4's position reading dropped by 524,764 (16 turns = 524,288) and id 6's
  rose by 819,199 (25 turns = 819,200); id 5 did not shift. Temperature B read equal to temperature A
  right after the restart. Gear ratios inferred: id 4 16:1, id 6 25:1 (id 5 presumably 25:1, same travel).
- `scripts/sweep.py` now shifts the stored end stops by whole output turns to match the reading and
  aborts if they still do not fit. SDK: `MotorBus.restart()`, `g1motor restart`.
- Scripts: `scripts/mode_probe.py`.

### Modes 3, 4, 5 characterised on bench id 4 (2026-11-25)

Eight short tests from rest at mid-travel (`scripts/mode_characterize.py`), then a coast-down from
about 800 velocity counts with an all-zero command (`scripts/mode_coastdown.py`).

| Mode | Torque +/-15 | Position step (kp 100) | Velocity +/-300 (kd 20) | Coast-down to <50 | Torque reading at rest |
|---|---|---|---|---|---|
| 0 | - | - | - | 10-11 ms | about -2 (noise) |
| 1 | reaches 1,200+ vel counts in 20 ms | moves 1,180 / 3,400 | tracks, 211-308 | 25 / 64 ms | -0.8 |
| 3 | no response | no response | no response | 9-11 ms | 0.0 |
| 4 | no response | no response | no response | 25 / 64 ms | -0.8 to -0.9 |
| 5 | no response | no response | no response | 10 ms | 0.0 |

- So 0, 3, 5 brake the joint passively; 4 leaves it free like mode 1 with a zero command; none of
  3-5 uses the command fields. In the braked modes the joint still crept slowly under load in a few
  tests (about -150 velocity counts with a small opposing torque reading).
- The mode-1 reference repeated after these tests matched the first run to within 1-2 %, so sending
  payloads in modes 3-5 did not change the motor's behaviour.

### Zeroing, gear ratios, units (2026-11-25)

- No zeroing command found (modes 0-7 leave the reading unchanged apart from the restart shift; the
  robot's controller sent no other frame type in our captures). SDK keeps the zero in software:
  `Joint` (gear ratio, end stops, zero, `sync()` by whole output turns), checked on the bench
  including a restart of id 6 that shifted the reading back by one turn.
- Output-side figures: travel 227 / 187 / 187 deg (ids 4 / 5 / 6); speed ceiling at 24 V about
  15.3 rad/s (id 4) and 9.8 rad/s (ids 5, 6).
- Position scale of 32,768 counts per rotor turn is now supported by the exact 16- and 25-turn jumps.

## Calibration on the robot (capture 153457_g1_calibration_6000000, 2026-11-25)

Listen-only capture, 39 s, arm chain ids 0-4, robot put into its calibration mode by the user.
206,932 pairs, all but one pass both checksums. Times from the start of the capture.

| Time | Controller | Motors |
|---|---|---|
| 0 - 19.42 s | normal run mode, zero command (0x9N) | mode 1 |
| 19.42 - 22.71 s | silent | |
| 22.709 s | one idle poll to id 0 | reply byte 2 = `0x80 + id` (bit 7 set; seen only in these first replies) |
| 22.710 - 22.976 s | 10 frames per motor, about 26 ms apart: byte 2 = `0x10 + id` (mode 1, no flag), byte 3 = 0, torque +16 (id 3: -16), kd 2, rest 0 | mode 0 then 1; joints press against an end stop, torque reading 14 (-17 on id 3) |
| 23.047 - 23.114 s | ONE frame per motor: byte 2 = `0x60 + id` (mode 6), byte 3 = 0, bytes 4-5 = `00 fc`, bytes 6-9 = int32 angle, bytes 10-15 = `4d 1a 0d 42 07 6b` | reply reports mode 6, with `00 fc` echoed in the torque field |
| 23.146 - 23.150 s | one mode-7 frame per motor (`0x70 + id`, all zero) | restart; bus silent 0.72 s |
| 23.869 s | idle polls | ids 0, 1, 4 normal; ids 2, 3 send a new blob with bytes 14-15 = `00 80` |
| 27.192 s | ids 2, 3: 12-byte payload (about 100 ms of frames) | normal telemetry from 27.290 s |
| 29.830 s | flag bit set (0x8N); still mode 0 at the end of the capture (39.2 s) | mode 0 |

The mode-6 angle is the joint angle at that end stop, int32 with 2^31 = one output turn:

| id | value | angle | G1 joint limit it matches | position reading at that moment | reading / angle |
|---|---|---|---|---|---|
| 0 | 912,689,835 | +2.6704 rad (153.00 deg) | shoulder pitch upper | 222,816 | 524,268 per turn |
| 1 | 769,510,605 | +2.2515 rad (129.00 deg) | shoulder roll upper | 187,856 | 524,252 |
| 2 | 894,801,238 | +2.6180 rad (150.00 deg) | shoulder yaw upper | 218,448 | 524,266 |
| 3 | -357,875,712 | -1.0471 rad (-59.99 deg) | elbow lower | -87,376 | 524,312 |
| 4 | 674,082,725 | +1.9723 rad (113.00 deg) | wrist roll upper | 164,560 | 524,253 |

- So mode 6 is the zeroing command: "you are now at this output angle". It needs the constant in
  bytes 10-15 (a mode-6 frame with a zero payload was ignored on the bench) and is followed by a restart.
- The position reading did not change across this calibration (the robot was already calibrated):
  the reading is the joint angle, 524,288 counts per output turn (16 x 32,768) with 0 at the joint's
  zero, on all five motors of this chain.
- The joint-limit names are from memory of Unitree's published G1 limits, not checked today.
- Challenge and response confirmed for ids 2 and 3: blob and 12-byte payload both differ from the
  boot capture earlier today (id 2 blob `c8 54 2c dc fe dc ba 9a d4 80 af` -> payload byte 3 = `c1`,
  `cf 15 f4 86 ef 3e ac 58 e0 07 6d 03`; id 3 blob `66 dd b7 3e 05 6a 2d ee a6 d7 85` -> `ca`,
  `19 7a 32 74 a0 a2 c5 3a 97 4c 34 e0`). The algorithm is unknown, so those two motors cannot be
  brought up without the robot's controller.
- Not reproduced by us. Unknown: what `00 fc` means, whether the 6-byte constant is the same on
  other robots or firmware, and whether the zero is stored permanently (the restart suggests so).

## Full re-test of the guide (2026-11-25)

`scripts/verify_captures.py` (28 checks against the robot captures) and `scripts/verify_bench.py`
(34 live checks on the bench wrist, including sine runs through `sweep.py`).

Corrections that came out of it:
- Damping torque is 0.005 x KD x velocity reading, not 0.0009. Steady speed under constant torque on
  id 4: (T, KD) = (20,10) (20,20) (20,40) (40,40) (40,80) (60,80) gave 0.00486-0.00512 with 4 counts
  of friction. The earlier 0.0009 came from the hard-brake log, where the motors were at their torque
  limit (400-700 counts; the damping term asked for about 4,000). The first torque-limited brake used
  the wrong constant; it now uses 0.005 and a 200-count budget, and stopped all three joints from
  about 1.2 M counts/s without a fault.
- The reply torque is the net applied torque: with a 20-60 count feed-forward and damping it read
  about +/-4 (friction) at steady speed.
- Robot bus voltage: typically 47.0-48.5 V (median 48.5 at midday, 47.0 by the calibration capture),
  extremes 44.5-50 V. The guide had said 48.0-48.5.
- FLAG and the sequence byte are not needed: from idle, MODE 1 with FLAG clear and byte 3 = 0 gave run
  mode on the second reply.
- Watchdog: after 0.3 s and 0.5 s without polling a motor was still in run mode; after 1.0 s and 1.2 s
  it was in mode 0 with status 0.
- "KP 20 / KD 1 does not move ids 5, 6" is pose-dependent: in the re-test id 6 drifted 15,000 counts
  under that gain. Reworded to "too weak to track".
- Right after a restart one motor reported coil/shell 0/0 in its first reply.
Not re-run: the deliberate hard-brake fault (`verify_bench.py --fault`); it was seen twice earlier today.

### Stopping from top speed: three ways compared (2026-11-25)

All from a 0.8 s full-range sine aborted at peak speed (about 1.03 / 1.27 / 1.27 M counts/s), 24 V bench supply.

| Stop | Time to stop | Bus voltage reading | Status |
|---|---|---|---|
| Damping, KD 80 | about 25 ms | drops at once to 15-19 V | `0x8`, latched |
| Damping limited to about 180 torque counts | 27-36 ms | rises to 30 V within 6 ms, then falls to 10-11 V at about 130 ms, back to 24 V by 560 ms | `0x8` from 130 ms; motors reset by themselves (about 28 lost replies each) and came back with 0 |
| Straight to idle (mode 0) | 18 / 27 / 24 ms | 24.0-24.5 V throughout | 0, every poll answered |

- So `0x8` tracks the voltage reading falling to about 15.5 V: an under-voltage flag. The damping brakes
  cause it through the supply (regeneration pushes it up until it cuts out; the hardest brake pulls it down).
- Idle is the stop to use. `sweep.py` aborts and SDK `disable()` now go straight to idle.
