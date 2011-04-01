#!/bin/python
import ecto
from ecto.doc import printModuleDoc, graphviz
import pcl

import time
debug = True

plasm = ecto.Plasm()


kinect_grabber = ecto.make(pcl.KinectGrabber);
printModuleDoc(kinect_grabber)
cloud_viewer = ecto.make(pcl.CloudViewer);
printModuleDoc(cloud_viewer)

plasm.connect(kinect_grabber, "out", cloud_viewer, "input")

print plasm.viz()

prev = time.time()
count = 0
while(cloud_viewer.o.stop.get() is False):
    #plasm.markDirty(kinect_grabber)
    plasm.go(cloud_viewer)
    now = time.time()
    if(count == 30):
        print "%f fps" % (1/ ((now - prev) / count))
        prev = now
        count = 0
    count += 1

print "Done"

