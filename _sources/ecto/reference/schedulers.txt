.. index:: scheduler, Singlethreaded, Threadpool

.. _schedulers:

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

.. class:: ecto.Scheduler

   Singlethreaded scheduler.  This class will execute the cells in the
   plasm in topological order in a single thread.

   .. method:: __init__(plasm)

      Construct one around a plasm

   .. method:: execute()

      Execute the graph in an infinite loop.  Blocks.

   .. method:: execute(niter)

      Execute the graph niter times.  This call will blocks until the
      execution is finished.

   .. method:: execute_async()
   .. method:: execute_async(niter)

      These functions are the same as the blocking ones above, except
      they start an execution thread in the background and return
      immediately.

   .. method:: running()

      Returns true if the execution started by execute_async() is still running.

   .. method:: stop()

      Stops the background graph execution at the end of the current process() call.

   .. method:: wait()

      Blocks until the execution started by the last call to
      execute_async() is finished.


Reentrant running
------------------

The schedulers are fully reentrant, meaning, that when you stop(), and wait() on
a scheduler it may be executed again.

The following code is therefore valid for any ecto scheduler:

.. code-block:: python

    s = Sheduler(p)
    s.execute_async()
    time.sleep(0.5)
    s.stop()
    s.wait()
    s.execute_async()
    assert s.running()


