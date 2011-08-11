#!/usr/bin/env python
import ecto
from ecto_X import Sink
from ecto_opencv.highgui import imshow, FPSDrawer

video = Sink(url='localhost', port=2932)
fps = FPSDrawer()
video_display = imshow(name='video_cap', waitKey=0)

plasm = ecto.Plasm()
plasm.connect(video[:] >> fps['image'],
              fps['image'] >> video_display['input'],
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture a video from the device and display it.')
