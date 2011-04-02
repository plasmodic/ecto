#!/bin/python
import ecto
from ecto.doc import printModuleDoc, graphviz
import pcl

import time
debug = True

plasm = ecto.Plasm()

kinect_grabber = ecto.make(pcl.KinectGrabber);
cloud_viewer = ecto.make(pcl.CloudViewer);
voxel = ecto.make(pcl.VoxelGrid, leaf_size=0.1)


plasm.connect(kinect_grabber, "out", voxel, "in")
plasm.connect(kinect_grabber, "out", cloud_viewer, "input")
#plasm.connect(voxel, "out", cloud_viewer, "input")
print plasm.viz()

prev = time.time()
count = 0
while(cloud_viewer.o.stop.get() is False):
    plasm.markDirty(kinect_grabber)
    plasm.go(cloud_viewer)
    now = time.time()
    if(count == 30):
        print "%f fps" % (1 / ((now - prev) / count))
        prev = now
        count = 0
    count += 1

print "Done"

