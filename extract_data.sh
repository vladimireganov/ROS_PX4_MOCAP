#!/bin/bash
# NOTE : Quote it else use array to avoid problems #
FILES="/Volumes/GAMEZ/px4_data/faster_logging/log/2022-06-20/*"
for f in $FILES
do
  ulog2csv $f
  # take action on each file. $f store current file name
#   cat "$f"
done