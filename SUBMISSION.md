# AI Racing League project submission

**Name:** Pumelela Mapukata
**Project members:** individual project
**GitHub link:** https://github.com/pumelela1/ai-racing-league
**Video demonstration:** https://drive.google.com/file/d/1HKhP8QZ6GjFDsN0Fd8saVOVOmVpUj8gg/view?usp=sharing

## Project description / problem statement
The starter driver only reacts to what its sensors see a moment ahead, so it is slow through
corners and does not use the width of the track. My driver knows the Corkscrew layout: it follows
a pre-computed racing line and a speed plan for every metre of the lap and controls steering,
brakes, throttle and gears to stay on that plan. The car model is unchanged.

## Result
Corkscrew, Quick Race, 1 lap, standing start, on my laptop: **1:12.478, damage 0**
(earlier versions: 1:40.220 and 1:37.424, both damage 0).

## Enhancements
1. Racing line and speed plan for the whole lap (`corkscrew_line.py`). Together with
   enhancement 2 this took the lap from 1:37.424 to 1:12.478.
2. Control that follows the plan: pure-pursuit steering, planned braking with ABS, friction-circle
   throttle limit with traction control. Zero damage, car always inside the track edges.
3. Gear shifting by engine speed over all seven gears, instead of the starter's fixed road-speed
   shift points (not measured on its own).

## Use of IBM Bob (free trial version)
IBM Bob (Bob IDE) was used at the end of the project to review the final driver. In Ask mode it
read `corkscrew_driver.py` and the telemetry of my final lap, explained the control logic, and
pointed to where the car runs furthest off the racing line (about 0.62 m at 2,665-2,727 m). It
proposed testing `CROSS_GAIN = 0.035` instead of `0.0152`. In Agent mode it created
`corkscrew_driver_bob.py` with only that value changed, and I checked the one-line diff.

I ran the copy on the same track and settings: zero damage, lap time unchanged (72.27 s against
72.26 s at the 3,600 m mark), and the section Bob targeted did not improve (0.65 m for 1.4 s
against 0.62 m for 1.3 s). I rejected the change and kept the original value.

Bob was not used to build the racing line, the speed plan or the controller.

## Other AI assistance
The racing line, speed plan and controller tuning were developed with the Claude AI assistant and
tested in a headless build of the same simulator. Details are in `README.md`.
