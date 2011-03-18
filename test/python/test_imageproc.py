#!/usr/bin/env python

import ecto
from ecto.doc import printModuleDoc,graphviz
import imageproc as im

video = im.VideoCapture()
video
video.Config()
printModuleDoc(video)

imshow = im.ImageShower()
printModuleDoc(imshow)

sobelShower = im.ImageShower()
sobelShower.Config("sobel",1,True)

grayShower = im.ImageShower()
grayShower.Config("gray",1,True)

sobelX = im.Sobel()
sobelX.Config(1,0)
sobelY = im.Sobel()
sobelY.Config(0,1)

rgb2gray = im.Rgb2Gray()

adder = im.ImageAdder()
printModuleDoc(adder)
abs1 = im.AbsNormalized()
abs2 = im.AbsNormalized()

plasm = ecto.Plasm()

plasm.connect(video, "out", imshow, "in")
plasm.connect(video, "out", rgb2gray , "in")
plasm.connect(rgb2gray, "out", sobelX, "in")
plasm.connect(rgb2gray, "out", sobelY, "in")
plasm.connect(rgb2gray, "out", grayShower,"in")
plasm.connect(sobelX, "out", abs1, "in")
plasm.connect(sobelY, "out", abs2, "in")
plasm.connect(abs1, "out", adder , "a")
plasm.connect(abs2, "out", adder , "b")
plasm.connect(adder, "out", sobelShower, "in")

graphviz(plasm)

while(imshow.outputs["out"].get() != 27):
    plasm.markDirty(video)
    # TODO just call go on the whole plasm, to trigger all leaves being called. 
    plasm.go(sobelShower)
    plasm.go(grayShower)
    plasm.go(imshow)




