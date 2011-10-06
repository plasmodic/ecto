#!/usr/bin/env python
import ecto
from ecto_X import Source
from ecto_opencv.highgui import VideoCapture

video_cap = VideoCapture(video_device=0, width=640, height=480)
source = Source(port=2932)

plasm = ecto.Plasm()
plasm.connect(video_cap['image'] >> source['in'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Run a simple ecto video capture server.')