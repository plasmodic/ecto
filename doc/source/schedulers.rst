.. index:: scheduler, Singlethreaded, Threadpool

schedulers
==========

The schedulers operate on the plasm.  There are singlethreaded and
threadpool schedulers.  Yes, the name /scheduler/ is a little dry, and
I personally think a plasm should have a /metabolism/ or maybe a
/Krebs Cycle/ instead, but there were no motivating name collisions
(as with 'node').

The singlethreaded scheduler does *not* necessarily behave the same
way as the threadpool scheduler does if given one thread: the
threadpool scheduler allows each cell to run as soon as it its inputs
are ready and its outputs are clear, so in a multicelled plasm
multiple 'ticks' of data may be in flow at the same time.



Threadpool
==========

Threadpool has several overloads of execute().  The nullary one will
spawn one worker thread per node in the graph, up a maximum of the
hardware concurrency.


