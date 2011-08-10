.. include:: <s5defs.txt>

.. class:: center

   .. image:: ecto_4x4.svg
      :align: center

   a framework for perception




   troy straszheim

   and

   ethan rublee


ecto
====

Yo this is a talk about ecto

this slide is full

full of stuff

a zillion lines of codes

all kinda good stuffs


great
-----

:orange:`awesome` stuff

all kinds of hit you see case thats how we roll we get our schmoove on
and then we get paid cause thats why we make the big bucks

the design space
================

* interprocess vs intraprocess
* single- vs multi-threaded
* synchronous vs asynchronous
* compiled vs interpreted

LOC
===

Amoeba(Beta) 

Ecto core (including tests):
7k lines C++
2.8k  lines Python 

Tests:
4k lines C++
5k lines Python



28k lines C++  (2k generated in ecto_ros)
12k lines Python

Repo history goes to Aug 2010
Development starts:

commit 8edec7a5abcc3215ab8c351bd4b79ea63cd53a54
Author: Troy Straszheim <straszheim@willowgarage.com>m
Date:   Fri Mar 11 16:40:45 2011 -0800

    BA *BANG*






dig it
======

Multithreaded... or singlethreaded.  You choose.  At runtime.  Without
recompiling.




* an attack on the manycore problem

* semantics of copying


goals
=====

give researchers a mental model that makes them more productive 

support but don't force the use of technologies like opencl, CUDA, openmpi

scale to many cores w/o recompiling

make individual algorithms and *components* of algorithms easily
reusable

be more general than vision/perception/robotics... audio?  general machine learning?

portability:  unix/osx/windows

buildable and usable with standard tools... easy for experienced developers to grok

as introspectiable and self-documenting as possible

minimal system dependencies:  boost, python, cmake.

optionality of python

no environment variables beyond (PYTHON|LD_LIBRARY)_PATH

easy integration with other systems

embedding in other 


target users
------------

* researchers (vision|perception):  code in C++, run via python
* tinkerers, end-users:  script in python


future work
===========

new schedulers
processor affinity;  topology of graph on hardware
reverse edges
integration





