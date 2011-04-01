#!/bin/python
import ecto
from ecto.doc import printModuleDoc, graphviz
import imageproc
#import orb as imageproc
import calib

debug = True

plasm = ecto.Plasm()
rows = 5
cols = 3
square_size = 40 # in millis
pattern_show = ecto.make(imageproc.imshow, name="pattern", waitKey=10, autoSize=True)
rgb2gray = ecto.make(imageproc.cvtColor, flag=7)
video = ecto.make(imageproc.VideoCapture, video_device=0)
circle_detector = ecto.make(calib.PatternDetector, rows=rows, cols=cols)
circle_drawer = ecto.make(calib.PatternDrawer, rows=rows, cols=cols)
camera_calibrator = ecto.make(calib.CameraCalibrator, rows=rows, cols=cols, square_size=square_size)
printModuleDoc(camera_calibrator)
plasm.connect(video, "out", rgb2gray, "in")
plasm.connect(rgb2gray, "out", circle_detector, "in")
plasm.connect(video, "out", circle_drawer, "in")
plasm.connect(circle_detector, "out", circle_drawer, "points")
plasm.connect(circle_detector, "found", circle_drawer, "found")
plasm.connect(circle_drawer, "out", pattern_show, "in")
plasm.connect(video, "out", camera_calibrator, "image")
plasm.connect(circle_detector, "out", camera_calibrator,"points")
plasm.connect(circle_detector, "found", camera_calibrator, "found")

print graphviz(plasm)

while(pattern_show.o.out.get() != 27):
    plasm.markDirty(video)
    #plasm.go(lazer_show)
    plasm.go(pattern_show)
    plasm.go(camera_calibrator)

    

