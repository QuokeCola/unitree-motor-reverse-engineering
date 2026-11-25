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
| 3 | 1 | temperature A? (uint8) | 33-39, rises about 3 over 5 min; likely deg C |
| 4 | 1 | temperature B? (uint8) | tracks byte 3, 1-2 lower |
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
