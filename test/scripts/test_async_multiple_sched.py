#!/usr/bin/env python
import ecto
import ecto_test
import time

def make_plasm():
    generate = ecto_test.Generate()
    param_watcher = ecto_test.ParameterWatcher(value=2)
    sleep = ecto_test.Sleep(seconds=0.1)
    plasm = ecto.Plasm()
    plasm.connect(generate["out"] >> param_watcher["input"],
                  )
    plasm.insert(sleep)
    return plasm

def test_async_multiple_sched(Sheduler):
    p1 = make_plasm()
    p2 = make_plasm()
    
    s1 = Sheduler(p1)
    s1.execute_async()

    s2 = Sheduler(p2)
    s2.execute_async()
    time.sleep(0.5)
    s2.stop()
    s2.wait()
    assert not s2.running()
    s2.execute_async()
    time.sleep(0.5)
    assert s2.running()
    time.sleep(0.5)
    s2.stop()
    s2.wait()
    assert not s2.running()
    assert s1.running()
    s1.stop()
    s1.wait()

if __name__ == '__main__':
    test_async_multiple_sched(ecto.schedulers.Singlethreaded)
    test_async_multiple_sched(ecto.schedulers.Threadpool)


