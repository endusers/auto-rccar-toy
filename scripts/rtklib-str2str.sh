#!/bin/bash

cd ~/RTKLIB/app/consapp/str2str/gcc
./str2str -in ntrip://[user[:passwd]@]addr[:port][/mntpnt] -out serial://ttyACM0:460800 -b 1
