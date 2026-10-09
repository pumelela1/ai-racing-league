# Tools

Scripts used to build `corkscrew_line.py` and to tune the driver. They are kept for reference;
paths inside them point at the machine they were run on.

* `sim/` - a headless build of TORCS 1.3.7 with the SCR server: `build.sh`, the small
  `main_sim.cpp` (runs a race from the console, or dumps the track geometry) and stub headers
  for the graphics libraries. Car, track and race configuration files came from the game folder.
* `trk.py` - reads the track geometry dump.
* `gen.py` - racing line (bounded least squares on curvature) and speed plan.
* `racer.py` - the driver logic in a form the test harness can re-parameterise.
* `harness.py` - runs one lap against the headless build and returns lap time, damage and
  how close the car came to the track edges; can add control delay, noise or lower grip.
* `opt2.py`, `ev.py`, `exp.py`, `bounds.py`, `final_line.py`, `battery.py`, `build_pkg.py` - search, stress-test and packaging scripts.
