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
[To be completed by the author: actual Bob contribution and how it was verified.]

## Other AI assistance
The racing line, speed plan and controller tuning were developed with the Claude AI assistant and
tested in a headless build of the same simulator. Details are in `README.md`.
