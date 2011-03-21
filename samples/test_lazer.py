#!/bin/python
import ecto
from ecto.doc import printModuleDoc, graphviz
import imageproc
#import orb as imageproc
import lazer
import calib

debug = True

  
plasm = ecto.Plasm()


splitter = ecto.make(imageproc.ChannelSplitter, bgr=True)
sl_drawer = ecto.make(lazer.ScanLineDrawer);
laser_detector = ecto.make(lazer.LaserDetector)
imshowR = ecto.make(imageproc.imshow, name="red", waitKey=10, autoSize=True)
imshowB = ecto.make(imageproc.imshow, name="blue", waitKey= -1, autoSize=True)
imshowG = ecto.make(imageproc.imshow, name="green", waitKey= -1, autoSize=True)
pattern_show = ecto.make(imageproc.imshow, name="pattern", waitKey= -1, autoSize=True)
lazer_show = ecto.make(imageproc.imshow, name="lazer", waitKey= -1, autoSize=True)
rgb2gray = ecto.make(imageproc.cvtColor, flag=7)
video = ecto.make(imageproc.VideoCapture, video_device=0)
circle_detector = ecto.make(calib.CircleDetector, rows=11, cols=4)
circle_drawer = ecto.make(calib.PatternDrawer, rows=11, cols=4)
camera_calibrator = ecto.make(calib.CameraCalibrator, rows=11, cols=4)
printModuleDoc(camera_calibrator)
plasm.connect(video, "out", splitter, "in")
plasm.connect(video, "out", rgb2gray, "in")
plasm.connect(rgb2gray, "out", circle_detector, "in")
plasm.connect(video, "out", circle_drawer, "in")
plasm.connect(circle_detector, "out", circle_drawer, "points")
plasm.connect(circle_detector, "found", circle_drawer, "found")
plasm.connect(circle_drawer, "out", pattern_show, "in")
plasm.connect(splitter, "red", imshowR, "in")
plasm.connect(splitter, "blue", imshowB, "in")
plasm.connect(splitter, "green", imshowG, "in")

plasm.connect(splitter, "red", sl_drawer, "in")
#plasm.connect(splitter, "green", lazer_show, "in")
plasm.connect(sl_drawer, "out", lazer_show, "in")

graphviz(plasm)

while(imshowR.o.out.get() != 27):
    plasm.markDirty(video)
    #plasm.go(lazer_show)
    plasm.go(pattern_show)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(imshowB)
    plasm.go(imshowG)
    plasm.go(imshowR)

    

