#!/bin/bash
# NOTE : Quote it else use array to avoid problems #
# FILES="/Volumes/GAMEZ/px4_data/faster_logging/log/2022-06-20/*"
FILES="/Volumes/GAMEZ/px4_data/faster_logging/log/2022-08-18/*"
for f in $FILES
do
  echo "$f"
  base=${f%.ulg}
  echo "$base/"
  ulog2csv -o $base/ $f 
done