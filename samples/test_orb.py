#!/bin/python
import ecto
from ecto.doc import printModuleDoc, graphviz
import imageproc
#import orb as imageproc
import orb

def hookUpORB(plasm, image, image_key, imshow):
    FAST = ecto.make(orb.FAST)
    Harris = ecto.make(orb.Harris)
    
    FAST.p.N_max = 1500
    Harris.p.N_max = 300
    DrawKeypoints = ecto.make(orb.DrawKeypoints)
    plasm.connect(image, image_key, FAST, "image")
    plasm.connect(FAST, "out", Harris, "kpts")
    plasm.connect(image, image_key, Harris, "image")
    plasm.connect(Harris, "out", DrawKeypoints, "kpts")
    plasm.connect(image, image_key, DrawKeypoints, "image")
    plasm.connect(DrawKeypoints, "image", imshow, "in")
    return Harris
  
plasm = ecto.Plasm()

pyramid = ecto.make(orb.Pyramid)
printModuleDoc(pyramid)

imshow = ecto.make(imageproc.imshow)
ecto.config(imshow, name="video", waitKey=10,autoSize=True)
rgb2gray = ecto.make(imageproc.Rgb2Gray)

video = ecto.make(imageproc.VideoCapture)


plasm.connect(video, "out", rgb2gray , "in")
plasm.connect(rgb2gray, "out", pyramid, "in")

rescale = ecto.make(orb.PyramidRescale)
printModuleDoc(rescale)
imshows = []
harrises = []
for i in pyramid.outputs:
    if( "out" in i.key()):
        x = ecto.make(imageproc.imshow)
        ecto.config(x, name=i.key(), waitKey=10,autoSize=False)
        harrises.append(hookUpORB(plasm, pyramid, i.key(), x))
        imshows.append(x)
i = 0
for x in harrises:
    plasm.connect(pyramid, "scale_%d"%i, rescale, "scale_%d"%i)
    plasm.connect(x, "out", rescale, "kpts_%d"%i)
    i += 1

DrawKeypoints = ecto.make(orb.DrawKeypoints)
plasm.connect(rescale, "out",DrawKeypoints, "kpts")
plasm.connect(video, "out",DrawKeypoints, "image")
plasm.connect(DrawKeypoints, "image", imshow , "in")
graphviz(plasm)

while(imshow.outputs["out"].get() != 27):
    plasm.markDirty(video)
    for x in imshows:
        plasm.go(x)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(imshow)
    
