# AI Racing League final project - Corkscrew driver

## Author and members
Pumelela Mapukata (individual project). GitHub: `pumelela1`.

## Problem statement
The starter driver in `snakeoil3_gym.py` only reacts to what its sensors see a moment ahead, so it
is slow through corners and never uses the width of the track. This project replaces it with a
driver that knows the Corkscrew layout: it follows a pre-computed racing line and a speed plan for
every metre of the lap, and controls steering, brakes, throttle and gears to stay on that plan.
The car model is unchanged; only the Python driver logic is different.

## Environment and setup
- OS: Windows 11 (64-bit)
- Python: Python 3 (3.12 or newer), standard library only
- TORCS and challenge kit: the organisers' Windows kit (`torcs.zip` from the challenge Box folder),
  TORCS with the `scr_server` driver, car `car1-ow1`. The kit is not included in this repository.
- Required local files: `corkscrew_driver.py`, `corkscrew_line.py`, `snakeoil3_gym.py` (all here)
- External dependencies: none

## Run
From this folder. Start the simulator first, then the client.

1. Start TORCS (`wtorcs.exe`): Race > Quick Race > New Race, with track **Corkscrew**, driver
   **scr_server 1** and **1 lap**. TORCS waits for the Python client.
2. Windows command: `python corkscrew_driver.py`
   macOS/Linux command: `python3 corkscrew_driver.py`

`python snakeoil3_gym.py` starts the same driver, and `-p <port>` works with both.
The terminal prints the lap time and damage; `telemetry.csv` is written for analysis.

## Results
Real runs on the author's laptop, 9 October 2026. Track Corkscrew, Quick Race, 1 lap, standing start.

| Run | Script | Lap time | Damage | Top speed |
|---|---|---|---|---|
| Starter driver | `snakeoil3_gym.py` `drive_example()` | not recorded | | |
| 1 | `corkscrew_driver_safe.py` (v1, sensor-only) | 1:40.220 | 0 | 235 km/h |
| 2 | `corkscrew_driver_v2.py` (v2, sensor-only) | 1:37.424 | 0 | 245 km/h |
| 3 | `corkscrew_driver.py` (final) | 1:12.478 | 0 | 284 km/h |
| 4 | `corkscrew_driver.py` (final, repeat run on the video) | about 1:12.4, exact time is in the video | 0 | 284 km/h |
| 5 | `corkscrew_driver_bob.py` (IBM Bob's experiment, `CROSS_GAIN` 0.035) | about 1:12.4, exact time not printed | 0 | 284 km/h |

See `experiment_results.csv`, `evidence/telemetry_final_lap.csv` (run 4) and
`evidence/telemetry_bob_experiment.csv` (run 5).

## Enhancements
1. **Racing line and speed plan for the whole lap** (`corkscrew_line.py`). For every metre of
   Corkscrew the driver has a target position and a target speed. The line is a minimum-curvature
   path inside the track edges, weighted so slow corners get the widest arc. The speed plan uses
   the simulator's tyre model (grip falls as load rises), downforce, banking and crests, with
   braking worked backwards and acceleration forwards from every corner.
   Evidence (enhancements 1 and 2 together): 1:37.424 (v2) to 1:12.478 (final), damage 0 in both.
2. **Control that follows the plan.** Pure-pursuit steering on the racing line, braking by the
   planned deceleration with ABS, and a throttle limit from the friction circle plus traction
   control, so the rear tyres are not asked to corner and accelerate beyond their grip.
   Evidence: the car stayed within 78% of the half-width of the track for the whole lap
   (`trackPos` in the telemetry), with zero damage.
3. **Gear shifting by engine speed.** The starter driver shifts at fixed road speeds and is in
   6th gear from 170 km/h. This driver shifts up near 18,500 rpm and down when the lower gear
   would still be below 16,300 rpm, using all seven gears, so the engine stays near peak power.
   Evidence: not measured on its own. Shifting by rpm was already part of v1 (1:40.220), and the
   starter driver's lap was not recorded on this laptop.

## Use of IBM Bob
IBM Bob (free trial version, Bob IDE) was used at the end of the project to review the final driver.

- **Review (Ask mode).** Bob read `corkscrew_driver.py` and the telemetry of the final lap
  (`evidence/telemetry_final_lap.csv`). It explained how the driver decides steering, speed,
  braking and gears, and pointed to where the car runs furthest off the racing line: about 0.62 m
  at 2,665-2,727 m, at roughly 180 km/h. Its hypothesis was that the cross-track steering
  correction is too weak. It proposed testing `CROSS_GAIN = 0.035` instead of `0.0152` and
  comparing lap time and the time spent more than 0.4 m off the line.
- **Change (Agent mode).** Bob created `corkscrew_driver_bob.py`, a copy of the driver with only
  that value changed. The diff was checked: one line.
- **Verification.** The copy was run on the same track and settings and compared with the final
  lap, using the two telemetry files in `evidence/`:

  | | Final driver (0.0152) | Bob's experiment (0.035) |
  |---|---|---|
  | Damage | 0 | 0 |
  | Lap time at the 3,600 m mark | 72.26 s | 72.27 s |
  | Time more than 0.4 m off the line | 2.9 s | 2.8 s |
  | Largest line error | 0.72 m | 0.78 m |
  | Section Bob targeted (2,665-2,730 m) | 0.62 m for 1.3 s | 0.65 m for 1.4 s |

- **Decision.** No measurable gain, and the targeted section did not improve, so the change was
  rejected and `CROSS_GAIN` stays at `0.0152`. The likely cause of that line error is the grip
  limit in the corner, not the steering gain.

Bob was not used to build the racing line, the speed plan or the controller. See the next section.

## Other AI assistance
The racing line, the speed plan and the controller tuning in `corkscrew_driver.py` and
`corkscrew_line.py` were developed with the Claude AI assistant. To test changes without running
hundreds of laps by hand, the same TORCS physics, car and track files were compiled into a
headless test build that runs a lap in about a second (scripts in `tools/`). That build reproduced
the real 1:40.220 lap as 1:40.09 and predicted 1:12.3 for the final driver. Settings were searched
automatically, and every candidate had to finish with zero damage with 5% less grip, with delayed
controls and with noisy controls.

## Demonstration
Video of the final lap (Corkscrew, one lap from a standing start):
https://drive.google.com/file/d/1HKhP8QZ6GjFDsN0Fd8saVOVOmVpUj8gg/view?usp=sharing

## Limitations
- Corkscrew only. The line and speed plan are a table for this track; the driver does not work on
  another track without generating a new table.
- No awareness of other cars: no overtaking or avoidance logic.
- Tested for one standing-start lap in Quick Race. No pit, fuel or multi-lap strategy.
- The starter driver's lap was not recorded on this laptop, so there is no measured baseline.

## Credits and licensing
- `snakeoil3_gym.py` is the Snake Oil client by Chris X Edwards as supplied in the organisers'
  gym_torcs kit. Only its `__main__` block was changed, so that it starts this driver.
- TORCS and the SCR server patch are not included. `tools/sim/` holds only small build scripts and
  stub headers used to compile TORCS 1.3.7 headless.
- `car1-ow1.rgb` is the car livery from the kit, unchanged.

## Files
| File | What it is |
|---|---|
| `corkscrew_driver.py` | the final racing driver |
| `corkscrew_line.py` | racing line and speed plan for Corkscrew (generated by `tools/`) |
| `corkscrew_driver_v2.py`, `corkscrew_driver_safe.py` | earlier sensor-only drivers (v2, v1) |
| `corkscrew_driver_bob.py` | IBM Bob's experiment: the final driver with `CROSS_GAIN` 0.035 (rejected) |
| `snakeoil3_gym.py` | TORCS client library from the kit |
| `car1-ow1.rgb` | car livery |
| `experiment_results.csv`, `evidence/` | run log, and telemetry of the final lap and of Bob's experiment |
| `SUBMISSION.md` | submission summary |
| `tools/` | line generator, test harness and tuning scripts |
