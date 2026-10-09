#!/bin/bash
# usage: run.sh <driver.py> [timeout_s]  -> runs one headless race with the given Python client
D=${1:-drv/corkscrew_driver.py}; T=${2:-180}
pkill -x simtorcs 2>/dev/null; sleep 0.2
cd /home/claude/sim/rt
timeout $T /home/claude/sim/build/simtorcs /home/claude/sim/rt/ /home/claude/sim/rt/config/raceman/quickrace.xml 2000000 > /home/claude/sim/build/sim.log 2>&1 &
SP=$!
sleep 0.8
( cd /home/claude/sim/$(dirname $D) && timeout $T python3 -W ignore $(basename $D) $EXTRA 2>&1 | grep -v "^$" | tail -6 )
sleep 0.3; kill $SP 2>/dev/null; wait $SP 2>/dev/null
