#!/bin/python
import ecto
from ecto.doc import printModuleDoc, graphviz
import imageproc
#import orb as imageproc
import orb

plasm = ecto.Plasm()

pyramid = orb.Pyramid()
pyramid.Config(3,1.2,0)
printModuleDoc(pyramid)

FAST = orb.FAST()
printModuleDoc(FAST)

Harris = orb.Harris()
printModuleDoc(Harris)

DrawKeypoints = orb.DrawKeypoints()
printModuleDoc(DrawKeypoints)

video = imageproc.VideoCapture()
video.Config(0)
printModuleDoc(video)

imshow = imageproc.ImageShower()
printModuleDoc(imshow)

rgb2gray = imageproc.Rgb2Gray()
printModuleDoc(rgb2gray)

plasm.connect(video, "out", rgb2gray , "in")
plasm.connect(video, "out", DrawKeypoints, "image")
plasm.connect(DrawKeypoints, "image", imshow, "in")
plasm.connect(rgb2gray, "out", FAST, "in")
plasm.connect(rgb2gray, "out", pyramid, "in")
plasm.connect(FAST, "out", Harris, "kpts")
plasm.connect(rgb2gray, "out", Harris, "image")
plasm.connect(Harris, "out", DrawKeypoints, "kpts")
plasm.connect(Harris, "out", DrawKeypoints, "kpts")

imshows = []
for i in range(0,3):
  x = imageproc.ImageShower()
  x.Config("pyr:%d"%i,1,True)
  plasm.connect(pyramid, "out:%d"%i, x, "in")
  imshows.append(x)
  
graphviz(plasm)

while(imshow.outputs["out"].value() != '27'):
    plasm.markDirty(video)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    for x in imshows:
      plasm.go(x)
    plasm.go(imshow)
    
