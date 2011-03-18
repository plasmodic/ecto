#!/bin/python
import ecto
from ecto.doc import printModuleDoc, graphviz
import imageproc
#import orb as imageproc
import orb

def hookUpORB(plasm, image,image_key, imshow):
  FAST = orb.FAST()
  Harris = orb.Harris()
  DrawKeypoints = orb.DrawKeypoints()
  plasm.connect(image, image_key, FAST, "in")
  plasm.connect(FAST, "out", Harris, "kpts")
  plasm.connect(image, image_key, Harris, "image")
  plasm.connect(Harris, "out", DrawKeypoints, "kpts")
  plasm.connect(image, image_key, DrawKeypoints, "image")
  plasm.connect(DrawKeypoints, "image", imshow, "in")
  #plasm.connect(image, image_key, imshow, "in")
  
plasm = ecto.Plasm()

n=5
s=1.3
m=0
pyramid = orb.Pyramid()
pyramid.Config(n,s,m)
printModuleDoc(pyramid)

imshow = imageproc.ImageShower()
rgb2gray = imageproc.Rgb2Gray()

video = imageproc.VideoCapture()
video.Config(0)

plasm.connect(video, "out", imshow , "in")
plasm.connect(video, "out", rgb2gray , "in")
plasm.connect(rgb2gray, "out", pyramid, "in")

imshows = []
try:
  for i in range(0,n+2):
    x = imageproc.ImageShower()
    x.Config("pyr:%d"%i,1,True)
    hookUpORB(plasm,pyramid, "out:%d"%i, x)
    imshows.append(x)
except RuntimeError,e:
  print "Good, caught exception..."
  print e
  
graphviz(plasm)

while(imshow.outputs["out"].get() != 27):
    plasm.markDirty(video)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    for x in imshows:
      plasm.go(x)
    plasm.go(imshow)
    
