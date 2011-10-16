Motivation
==========

Ecto started in 2010 as a response to real architectural difficulties
in computer vision and perception teams at Willow Garage.  The main
issue appeared to be that the preferred Model of Computation (MoC) of
perception groups, namely dataflow programming in a single process,
was difficult to implement using a distributed message passing
framework (ROS).

.. figure:: images/sync_constraint.png
   :scale: 50%

   What do we know about the timing of A, B, and C?  D and E?



