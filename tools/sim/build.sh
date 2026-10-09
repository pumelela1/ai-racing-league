#!/bin/bash
# Headless TORCS 1.3.7 + SCR server build (physics, track, race engine only)
S=/home/claude/sim/t137/src; B=/home/claude/sim/build; ST=/home/claude/sim/stub
INC="-I$ST -I/home/claude/sim/t137 -I$S/interfaces -I$S/libs/tgf -I$S/libs/txml -I$S/libs/robottools -I$S/libs/portability -I$S/libs/tgfclient -I$S/libs/racescreens -I$S/libs/client -I$S/libs/confscreens -I$S/libs -I$S/libs/raceengineclient -I$S/linux -I$S/modules/simu/simuv2/SOLID-2.0/include -I$S/modules/simu/simuv2 -I$S/libs/math"
CXX="g++ -std=gnu++98 -fpermissive -w -O2 -fPIC -DHAVE_CONFIG_H -D_SVID_SOURCE -D_BSD_SOURCE -D_DEFAULT_SOURCE -DSHM -include stdint.h $INC"
CC="gcc -std=gnu89 -w -O2 -fPIC -DHAVE_CONFIG_H $INC"
mkdir -p $B/core $B/track $B/simu $B/solid $B/scr
fail=0
comp(){ # outdir file
  o=$1/$(basename ${2%.*}).o
  if [ ! -f $o ] || [ $2 -nt $o ]; then
    case $2 in *.c) $CC -c $2 -o $o 2>>$B/err.log || { echo "FAIL $2"; fail=1; } ;; *) $CXX $3 -c $2 -o $o 2>>$B/err.log || { echo "FAIL $2"; fail=1; } ;; esac
  fi
}
: > $B/err.log
for f in $S/libs/txml/xmltok.c $S/libs/txml/xmlrole.c $S/libs/txml/xmlparse.c $S/libs/txml/hashtable.c $S/libs/txml/xml.cpp $S/libs/tgf/*.cpp $S/linux/linuxspec.cpp $S/libs/robottools/*.cpp $S/libs/raceengineclient/*.cpp /home/claude/sim/main_sim.cpp; do comp $B/core $f & if (( $(jobs -r | wc -l) >= 2 )); then wait -n; fi; done; wait
for f in $S/modules/track/*.cpp; do comp $B/track $f; done
for f in $S/modules/simu/simuv2/*.cpp; do comp $B/simu $f & if (( $(jobs -r | wc -l) >= 2 )); then wait -n; fi; done; wait
for f in $S/modules/simu/simuv2/SOLID-2.0/src/*.cpp; do comp $B/solid $f "-DNDEBUG -Wno-deprecated -I$S/modules/simu/simuv2/SOLID-2.0/src -I$S/modules/simu/simuv2/SOLID-2.0/include/3D" & if (( $(jobs -r | wc -l) >= 2 )); then wait -n; fi; done; wait
for f in $S/drivers/scr_server/*.cpp; do comp $B/scr $f; done
echo "compile done; errors: $(grep -c 'error' $B/err.log)"
