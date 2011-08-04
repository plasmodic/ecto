How To Get Cooking in the Ecto Kitchen
======================================

.. highlight:: ectosh

* Clone this project::

    % git clone git://github.com/plasmodic/ecto_kitchen.git
    Cloning into ecto_kitchen...
    remote: Counting objects: 52, done.
    remote: Compressing objects: 100% (39/39), done.
    remote: Total 52 (delta 20), reused 42 (delta 10)
    Receiving objects: 100% (52/52), 6.92 KiB, done.
    Resolving deltas: 100% (20/20), done.
  
* Update the submodules::

    % cd ecto_kitchen 

    % git submodule init
    Submodule 'ecto' (git://github.com/plasmodic/ecto.git) registered for path 'ecto'
    Submodule 'opencv' (git://github.com/plasmodic/ecto_opencv.git) registered for path 'opencv'
    Submodule 'pcl' (git://github.com/plasmodic/ecto_pcl.git) registered for path 'pcl'
    Submodule 'ros' (git://github.com/plasmodic/ecto_ros.git) registered for path 'ros'

    % git submodule update
    Cloning into ecto...
    remote: Counting objects: 6149, done.
    remote: Compressing objects: 100% (1974/1974), done.
    remote: Total 6149 (delta 4014), reused 5943 (delta 3811)
    Receiving objects: 100% (6149/6149), 2.13 MiB | 1.87 MiB/s, done.
    Resolving deltas: 100% (4014/4014), done.
    Submodule path 'ecto': checked out '87da4ed04ba46e9e852d82f5c7a2c9015a888389'
    Cloning into opencv...
    remote: Counting objects: 1310, done.
    remote: Compressing objects: 100% (536/536), done.
    remote: Total 1310 (delta 864), reused 1185 (delta 739)
    Receiving objects: 100% (1310/1310), 377.27 KiB, done.
    Resolving deltas: 100% (864/864), done.
    Submodule path 'opencv': checked out '2a43200bec3b3c599a64d84fbc64f0e973e5306a'
    Cloning into pcl...
    remote: Counting objects: 273, done.
    remote: Compressing objects: 100% (207/207), done.
    remote: Total 273 (delta 165), reused 161 (delta 53)
    Receiving objects: 100% (273/273), 56.69 KiB, done.
    Resolving deltas: 100% (165/165), done.
    Submodule path 'pcl': checked out '2377ba7bc459bc66faed520820e895941c649eef'
    Cloning into ros...
    remote: Counting objects: 335, done.
    remote: Compressing objects: 100% (176/176), done.
    remote: Total 335 (delta 206), reused 265 (delta 136)
    Receiving objects: 100% (335/335), 56.51 KiB, done.
    Resolving deltas: 100% (206/206), done.
    Submodule path 'ros': checked out '10f0715db9455887934f6855edaa1ab3aea71001'
    
* Source your ROS env (optional, modify to suit your shell)::

    % . /opt/ros/unstable/setup.zsh

* Do the usual cmake rigamarole::

    % mkdir build
  
    % cd build
  
    % cmake ..
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Check for working C compiler: /home/troy/bin/gcc
    -- Check for working C compiler: /home/troy/bin/gcc -- works
  
      [ tons more output from configuration of all submodules ]
  
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /home/troy/Projects/ecto_kitchen/build
  
* And run make::

    % make 
    [  0%] Building CXX object ecto/src/lib/CMakeFiles/ecto_cpp.dir/abi.cpp.o
    [  1%] Building CXX object ecto/src/lib/CMakeFiles/ecto_cpp.dir/tendril.cpp.o
  
      [ lots more output ]
  
    [100%] Building CXX object ros/src/CMakeFiles/ecto_std_msgs_ectomodule.dir/wrap_std_msgs_Time.cpp.o
    [100%] Building CXX object ros/src/CMakeFiles/ecto_std_msgs_ectomodule.dir/wrap_std_msgs_Byte.cpp.o
    Linking CXX shared library ../../lib/ecto_std_msgs.so
    [100%] Built target ecto_std_msgs_ectomodule
    
* Having built, source your python_path.sh::

    % . ./python_path.sh

* Everything should be accessible, ready to run scripts::

    % python
    Python 2.7.1+ (r271:86832, Apr 11 2011, 18:13:53) 
    [GCC 4.5.2] on linux2
    Type "help", "copyright", "credits" or "license" for more information.
    >>> import ecto, ecto_pcl, ecto_opencv.calib, ecto_ros
    >>> 
    
* And you should be ready to go to the tutorial [FIXME WHERE IS THE TUTORIAL]

