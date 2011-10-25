#!/bin/sh

rm -f *.png
for i in *.viz
do
    echo $i
    dot -Tpng $i -o`basename $i .viz`.png
done

# -r framerate
# -b bitrate
#
rm -f ecto.mp4
ffmpeg -r 20 -b 1800 -i ecto_%04d.png ecto.mp4
