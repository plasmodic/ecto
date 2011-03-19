#!/bin/python
import ecto
from ecto.doc import printModuleDoc, graphviz
import imageproc
#import orb as imageproc
import orb

debug = True

def hookUpORB(plasm, image, image_key, imshow):
    FAST = ecto.make(orb.FAST, N_max=1500)
    Harris = ecto.make(orb.Harris, N_max=100)
    plasm.connect(image, image_key, FAST, "image")
    plasm.connect(FAST, "out", Harris, "kpts")
    plasm.connect(image, image_key, Harris, "image")
    if(debug):
        draw_kpts = ecto.make(orb.DrawKeypoints)
        plasm.connect(Harris, "out", draw_kpts, "kpts")
        plasm.connect(image, image_key, draw_kpts, "image")
        plasm.connect(draw_kpts, "image", imshow, "in")
    return Harris
  
plasm = ecto.Plasm()

levels = 3
pyramid = ecto.make(orb.Pyramid, levels=levels, magnification=0, scale_factor=1.3)
printModuleDoc(pyramid)
#rescale needs the same number of levels as the pyramid
rescale = ecto.make(orb.PyramidRescale, levels=levels)
imshow = ecto.make(imageproc.imshow)
#configure the imshow
ecto.config(imshow, name="video", waitKey=10, autoSize=True)
rgb2gray = ecto.make(imageproc.cvtColor, flag=7)
printModuleDoc(rgb2gray)


video = ecto.make(imageproc.VideoCapture,video_device=0)
printModuleDoc(video)
plasm.connect(video, "out", rgb2gray , "in")
plasm.connect(rgb2gray, "out", pyramid, "in")


imshows = []
harrises = []
for i in pyramid.outputs:
    if("out" in i.key()):
        x = ecto.make(imageproc.imshow)
        ecto.config(x, name=i.key(), waitKey=10, autoSize=False)
        harrises.append(hookUpORB(plasm, pyramid, i.key(), x))
        imshows.append(x)
i = 0
for x in harrises:
    plasm.connect(pyramid, "scale_%d" % i, rescale, "scale_%d" % i)
    plasm.connect(x, "out", rescale, "kpts_%d" % i)
    i += 1

draw_kpts = ecto.make(orb.DrawKeypoints)
plasm.connect(rescale, "out", draw_kpts, "kpts")
plasm.connect(video, "out", draw_kpts, "image")
plasm.connect(draw_kpts, "image", imshow , "in")
graphviz(plasm)

while(imshow.o.out.get() != 27):
    plasm.markDirty(video)
    if debug:
        for x in imshows:
            plasm.go(x)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(imshow)
    

