#!/usr/bin/env python

import ecto
from ecto.doc import printModuleDoc,graphviz
import imageproc as im

video = ecto.make(im.VideoCapture)
printModuleDoc(video)

imshow = ecto.make(im.imshow,name="video",waitKey=10)
printModuleDoc(imshow)

sobelShower = ecto.make(im.imshow,name="sobel",waitKey=-1)

grayShower = ecto.make(im.imshow,name="gray",waitKey=-1)


sobelX = ecto.make(im.Sobel, x= 1, y = 0)
sobelY = ecto.make(im.Sobel, x= 0, y = 1)

rgb2gray = ecto.make(im.cvtColor, flag=7)

adder = ecto.make(im.ImageAdder)
printModuleDoc(adder)
abs1 = ecto.make(im.AbsNormalized)
abs2 = ecto.make(im.AbsNormalized)

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




