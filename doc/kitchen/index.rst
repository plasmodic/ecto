.. image:: art/ecto_4x4.svg
  :align: center

Ecto - a framework for perception
=================================

Initially aimed at computer vision and perception research tasks, Ecto is a hybrid C++/Python framework for organizing computations as directed acyclic graphs of computing ‘cells’ connected by typed edges. These graphs are typically constructed via Python script and executed in a single process (and possibly multiple threads) by external schedulers. The computing nodes are written in C++ against a simple interface that naturally creates libraries of self-documenting, scriptable components and smooths the path from prototyping to testing to deployment.

Ecto itself is small, has minimal dependencies (C++, Boost, Python) and works with or without OpenCV, PCL, and ROS in any combination. We believe that Ecto allows vision and perception researchers to express their computational models in a natural fashion, obviating e.g. ROS [1] time synchronizers and ROS nodelets in most cases.

As of 2012, Ecto is officially released. It is being used by researchers at Willow Garage and in industry for prototype applications of object capture and modelling, object recognition, pose estimation and refinement, projector-based augmented reality and chess playing.

.. rubric:: Ecto modules

To understand ecto, you probably want to go over the Ecto docs first:

.. toctree::
   :maxdepth: 1

   links/ecto/index.rst 

Ecto has several modules that wrap different libraries or functionalities:

.. toctree::
   :maxdepth: 1

   links/ecto/index.rst
   links/ecto_image_pipeline/index.rst
   links/ecto_openni/index.rst
   links/ecto_opencv/index.rst
   links/ecto_pcl/index.rst
   links/ecto_ros/index.rst

.. rubric:: Install

If you are on ROS Fuerte or above, you probably just want to install the packages available in the ROS repositories.

If you want to install from source, you will have to get them from https://github.com/plasmodic/ecto:
::

  mkdir ecto_kitchen && cd ecto_kitchen
  git clone http://github.com/willowgarage/catkin.git
  git clone http://github.com/plasmodic/ecto.git
  ln -s catkin/toplevel.cmake CmakeLists.txt

Then get the ecto modules you want:
::

  git clone http://github.com/plasmodic/ecto_image_pipeline.git
  git clone http://github.com/plasmodic/ecto_openni.git
  git clone http://github.com/plasmodic/ecto_opencv.git
  git clone http://github.com/plasmodic/ecto_pcl.git
  git clone http://github.com/plasmodic/ecto_ros.git

And you're then good to go for the usual cmake install:
::

  mkdir build && cd build && cmake ../ && make

.. rubric:: IRC

Drop in on us on ``#ecto`` on ``irc.oftc.net``

.. rubric:: Email List

Also feel free to join the email list:

* site: http://groups.google.com/a/plasmodic.org/group/ecto-dev
* email: **ecto-dev@plasmodic.org**
