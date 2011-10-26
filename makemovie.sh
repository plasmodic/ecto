#!/bin/sh -ex

rm -f *.png *.viz *.svg
../test/scripts/test_movies.py

for i in *.viz
do
    echo $i
    dot -Tpng $i -o`basename $i .viz`.png
    # dot -Tsvg $i -o`basename $i .viz`.svg
done

# -r framerate
# -b bitrate
#
rm -f ecto.mp4
ffmpeg -r 20 -b 1800 -i ecto_%04d.png ecto.mp4

mplayer ecto.mp4