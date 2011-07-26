How To Get Cooking in the Ecto Kitchen
======================================

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

* Build::

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
  
    % make 
    [  0%] Building CXX object ecto/src/lib/CMakeFiles/ecto_cpp.dir/abi.cpp.o
    [  1%] Building CXX object ecto/src/lib/CMakeFiles/ecto_cpp.dir/tendril.cpp.o
  
      [ lots more output ]
  
    [100%] Building CXX object ros/src/CMakeFiles/ecto_std_msgs_ectomodule.dir/wrap_std_msgs_Time.cpp.o
    [100%] Building CXX object ros/src/CMakeFiles/ecto_std_msgs_ectomodule.dir/wrap_std_msgs_Byte.cpp.o
    Linking CXX shared library ../../lib/ecto_std_msgs.so
    [100%] Built target ecto_std_msgs_ectomodule
    
* Source your python_path.sh::

    % . ./python_path.sh

* Everything should be accessible, ready to run scripts::

    % python
    Python 2.7.1+ (r271:86832, Apr 11 2011, 18:13:53) 
    [GCC 4.5.2] on linux2
    Type "help", "copyright", "credits" or "license" for more information.
    >>> import ecto, ecto_pcl, ecto_opencv.calib, ecto_ros
    >>> 
    

Doing Development
=================

The first thing you need is remotes that you can push to.  Assuming
that you have a github account and your own fork of ecto_kitchen and
each submodule, there is a script that will add these, run it with
your github username as an argument::

  % ./util/add_submodule_remotes.sh straszheim
  Adding remotes for plasmodic and straszheim
  ========= ecto_kitchen ==========
  origin	git://github.com/plasmodic/ecto_kitchen.git (fetch)
  origin	git://github.com/plasmodic/ecto_kitchen.git (push)
  plasmodic	git@github.com:plasmodic/ecto_kitchen.git (fetch)
  plasmodic	git@github.com:plasmodic/ecto_kitchen.git (push)
  straszheim	git@github.com:straszheim/ecto_kitchen.git (fetch)
  straszheim	git@github.com:straszheim/ecto_kitchen.git (push)
  ========= ecto ==========
  origin	git://github.com/plasmodic/ecto.git (fetch)
  origin	git://github.com/plasmodic/ecto.git (push)
  plasmodic	git@github.com:plasmodic/ecto.git (fetch)
  plasmodic	git@github.com:plasmodic/ecto.git (push)
  straszheim	git@github.com:straszheim/ecto.git (fetch)
  straszheim	git@github.com:straszheim/ecto.git (push)
  ========= ecto_opencv ==========
  origin	git://github.com/plasmodic/ecto_opencv.git (fetch)
  origin	git://github.com/plasmodic/ecto_opencv.git (push)
  plasmodic	git@github.com:plasmodic/ecto_opencv.git (fetch)
  plasmodic	git@github.com:plasmodic/ecto_opencv.git (push)
  straszheim	git@github.com:straszheim/ecto_opencv.git (fetch)
  straszheim	git@github.com:straszheim/ecto_opencv.git (push)
  ========= ecto_pcl ==========
  origin	git://github.com/plasmodic/ecto_pcl.git (fetch)
  origin	git://github.com/plasmodic/ecto_pcl.git (push)
  plasmodic	git@github.com:plasmodic/ecto_pcl.git (fetch)
  plasmodic	git@github.com:plasmodic/ecto_pcl.git (push)
  straszheim	git@github.com:straszheim/ecto_pcl.git (fetch)
  straszheim	git@github.com:straszheim/ecto_pcl.git (push)
  ========= ecto_ros ==========
  origin	git://github.com/plasmodic/ecto_ros.git (fetch)
  origin	git://github.com/plasmodic/ecto_ros.git (push)
  plasmodic	git@github.com:plasmodic/ecto_ros.git (fetch)
  plasmodic	git@github.com:plasmodic/ecto_ros.git (push)
  straszheim	git@github.com:straszheim/ecto_ros.git (fetch)
  straszheim	git@github.com:straszheim/ecto_ros.git (push)
  
Now I'll do some development.  Note that the submodules initially have
'detached heads' which can be tricky::

  % cd pcl

  % git status
  # Not currently on any branch.
  nothing to commit (working directory clean)

So check out e.g. the master branch::

  % git checkout master
  Switched to branch 'master'

  % git status
  # On branch master
  nothing to commit (working directory clean)

Or (handy trick) if you want to do this for all submodules::

  % git submodule foreach git checkout master
  Entering 'ecto'
  Previous HEAD position was 87da4ed... small ecto kitchen tweaks
  Switched to branch 'master'
  Entering 'opencv'
  Previous HEAD position was 2a43200... work under standalone build
  Switched to branch 'master'

  [ etc ]

Having checkout out master, make some changes::
 
  % emacs -nw CMakeLists.txt

  % git add CMakeLists.txt 

  % git commit -m "unimportant tweak"
  [master 5f32919] unimportant tweak
   1 files changed, 2 insertions(+), 0 deletions(-)

  % git status 
  # On branch master
  # Your branch is ahead of 'origin/master' by 1 commit.
  #
  nothing to commit (working directory clean)

And push. In this case I'll push to my clone, you'll probably push to
your clone with your name::

  % git push straszheim master
  Counting objects: 5, done.
  Delta compression using up to 8 threads.
  Compressing objects: 100% (3/3), done.
  Writing objects: 100% (3/3), 341 bytes, done.
  Total 3 (delta 2), reused 0 (delta 0)
  To git@github.com:straszheim/ecto_pcl.git
     5ce91de..5f32919  master -> master

Now I can update the kitchen.  I see that git knows something has
happened::

  % cd ..

Submodule summary shows the new commits in the submodules::

  % git submodule summary
  * pcl 2377ba7...5f32919 (1):
    > unimportant tweak

The submodule status shows that the hash for the pcl submodule has changed::

  % git submodule status
   87da4ed04ba46e9e852d82f5c7a2c9015a888389 ecto (heads/master)
   2a43200bec3b3c599a64d84fbc64f0e973e5306a opencv (heads/master)
  +5f329192a280665eb8478f889b25465025fd8203 pcl (heads/master)
   10f0715db9455887934f6855edaa1ab3aea71001 ros (heads/master)
  
And the regular old status shows that the pcl subdir has changed::

  % git status
  # On branch master
  # Your branch is ahead of 'origin/master' by 1 commit.
  #
  # Changes not staged for commit:
  #   (use "git add <file>..." to update what will be committed)
  #   (use "git checkout -- <file>..." to discard changes in working directory)
  #
  #	modified:   pcl (new commits)
  #
  no changes added to commit (use "git add" and/or "git commit -a")

I commit pcl as if it were a file::

  % git add pcl  
  % git commit -m "update pcl"
  [master d99026f] update pcl
   1 files changed, 1 insertions(+), 1 deletions(-)
  
And now I can push to my clone of the kitchen::

  % git push straszheim master
  Counting objects: 3, done.
  Delta compression using up to 8 threads.
  Compressing objects: 100% (2/2), done.
  Writing objects: 100% (2/2), 257 bytes, done.
  Total 2 (delta 1), reused 0 (delta 0)
  To git@github.com:straszheim/ecto_kitchen.git
     7fd45de..d99026f  master -> master
  
*NOTE* that the kitchen only records a repository and a hash for each
submodule, no more, and it does not verify that this hash actually
exists... that is, you have to be sure you've pushed what the
submodule refers to.

Workflow for code that is not yet in ecto_kitchen
-------------------------------------------------

Assume I have a bunch of changes on a special branch called
``awesome`` that is in my clone of ``ecto_pcl``.  Of course you are
welcome to clone ecto_kitchen itself and manage the submodules
yourself.  Perhaps more convenient (until we get our workflows
properly grokked):

* Clone ecto_kitchen::

    % git clone git://github.com/plasmodic/ecto_kitchen.git ek
    Initialized empty Git repository in /home/ek/.git/
    remote: Counting objects: 79, done.
    remote: Compressing objects: 100% (60/60), done.
    remote: Total 79 (delta 36), reused 59 (delta 16)
    Receiving objects: 100% (79/79), 13.22 KiB, done.
    Resolving deltas: 100% (36/36), done.
    
* Modify your .gitmodules appropriately.  Here I have pointed ecto_pcl
  to my clone::

    % cat .gitmodules 
    [submodule "ecto"]
    	path = ecto
    	url = git://github.com/plasmodic/ecto.git
    [submodule "pcl"]
    	path = pcl
    	url = git@github.com:straszheim/ecto_pcl.git   # <-- now git@ and straszheim
    [submodule "opencv"]
    	path = opencv
    	url = git://github.com/plasmodic/ecto_opencv.git
    [submodule "ros"]
    	path = ros
    	url = git://github.com/plasmodic/ecto_ros.git
    

* ``init`` and ``update`` as usual::

    % git submodule init
    Submodule 'ecto' (git://github.com/plasmodic/ecto.git) registered for path 'ecto'
    Submodule 'opencv' (git://github.com/plasmodic/ecto_opencv.git) registered for path 'opencv'
    Submodule 'pcl' (git@github.com:straszheim/ecto_pcl.git) registered for path 'pcl'  # <-- here
    Submodule 'ros' (git://github.com/plasmodic/ecto_ros.git) registered for path 'ros'

    % git submodule update
    Initialized empty Git repository in /home/ek/ecto/.git/
    remote: Counting objects: 6154, done.
    remote: Compressing objects: 100% (1979/1979), done.
    remote: Total 6154 (delta 4018), reused 5944 (delta 3811)
    Receiving objects: 100% (6154/6154), 2.13 MiB | 1.33 MiB/s, done.
    Resolving deltas: 100% (4018/4018), done.
    Submodule path 'ecto': checked out '904fb757e917137ee6b33d39fd4ce5eeb8d5a97a'
    Initialized empty Git repository in /home/ek/opencv/.git/
    remote: Counting objects: 1313, done.
    remote: Compressing objects: 100% (539/539), done.
    remote: Total 1313 (delta 866), reused 1186 (delta 739)
    Receiving objects: 100% (1313/1313), 377.49 KiB, done.
    Resolving deltas: 100% (866/866), done.
    Submodule path 'opencv': checked out 'a92bd1f1eba7f37875a464f75813907ac3c1d9b2'
    Initialized empty Git repository in /home/ek/pcl/.git/
    remote: Counting objects: 279, done.
    remote: Compressing objects: 100% (105/105), done.
    remote: Total 279 (delta 168), reused 268 (delta 161)
    Receiving objects: 100% (279/279), 58.56 KiB, done.
    Resolving deltas: 100% (168/168), done.
    Submodule path 'pcl': checked out '17a58172d3b40628db32d25cb34f64df49c27a5d'
    Initialized empty Git repository in /home/ek/ros/.git/
    remote: Counting objects: 335, done.
    remote: Compressing objects: 100% (176/176), done.
    remote: Total 335 (delta 206), reused 265 (delta 136)
    Receiving objects: 100% (335/335), 56.51 KiB, done.
    Resolving deltas: 100% (206/206), done.
    Submodule path 'ros': checked out '10f0715db9455887934f6855edaa1ab3aea71001'
    



Git submodule-fu
----------------

Fetch (but not pull/merge) all submodules, this is always safe to do::

  % git submodule foreach git fetch --all

Submodule status::

  % git submodule status
   08e12656332147a0831231e93098ed985af10212 ecto (heads/master)
   c0d7e0d74d047eef67452210f169a191826e3aef opencv (heads/master)
   494fa25a4cf0d719b8d7c565ca685f7235771f2d pcl (heads/master)
   ebda10bfb71c8b1c18d407e87d8ddc4f953eb897 ros (heads/master)
       
Show my local changes vs heads of all submodules (assuming you've fetched)::

  % git submodule foreach git diff --stat plasmodic/master


