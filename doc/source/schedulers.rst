.. index:: scheduler, Singlethreaded, Threadpool

schedulers
==========

The schedulers operate on the plasm.  There are singlethreaded and
threadpool schedulers.  Yes, the name *scheduler* is a little dry, and
some think a plasm should have a *metabolism* or *Krebs Cycle*
instead, but there were no motivating name collisions (as with
'node').

All schedulers should implement a basic interface for construction and
execution (the same as the interface as the Singlethreaded scheduler
implements); scheduler-specific functions and parameters may exist
outside this set.

Singlethreaded
--------------

.. class:: ecto.schedulers.Singlethreaded

   Singlethreaded scheduler.  This class will execute the cells in the
   plasm in topological order in a single thread.

   .. method:: __init__(plasm)

      Construct one around a plasm

   .. method:: execute(niter)

      Execute the graph, niter times, or forever if niter is not
      specified.  This call will blocks until the execution is finished.

   .. method:: execute_async(niter)

      Execute the graph, niter times, or forever if niter is not
      specified.  This call starts an execution thread in the
      background and returns immediately.

   .. method:: running()

      Returns true if the execution started by execute_async() is still running.

   .. method:: stop()

      Stops the background graph execution at the end of the current process() call.

   .. method:: wait()

      Blocks until the execution started by the last call to
      execute_async() is finished.

                  
Threadpool
----------

Threadpool has several overloads of execute().  The nullary one will
spawn one worker thread per node in the graph, up a maximum of the
hardware concurrency.


*FIXME* document me once we've got async execution in.






