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

See `experiment_results.csv` and `evidence/telemetry_final_lap.csv` (telemetry of run 4).

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
[To be completed by the author: what IBM Bob proposed or generated for this project, and what was
reviewed, tested or rejected.]

## Other AI assistance
The racing line, the speed plan and the controller tuning in `corkscrew_driver.py` and
`corkscrew_line.py` were developed with the Claude AI assistant. To test changes without running
hundreds of laps by hand, the same TORCS physics, car and track files were compiled into a
headless test build that runs a lap in about a second (scripts in `tools/`). That build reproduced
the real 1:40.220 lap as 1:40.09 and predicted 1:12.3 for the final driver. Settings were searched
automatically, and every candidate had to finish with zero damage with 5% less grip, with delayed
controls and with noisy controls.

## Demonstration
[Video link to be added]

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
| `snakeoil3_gym.py` | TORCS client library from the kit |
| `car1-ow1.rgb` | car livery |
| `experiment_results.csv`, `evidence/` | run log and telemetry of the final lap |
| `SUBMISSION.md` | submission summary |
| `tools/` | line generator, test harness and tuning scripts |
