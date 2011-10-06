#!/usr/bin/env python
import ecto
import ecto_test
import time

def test_parameter_callbacks():
    generate = ecto_test.Generate()
    param_watcher = ecto_test.ParameterWatcher(value=2)
    sleep = ecto_test.Sleep()
    printer = ecto_test.Printer()
    plasm = ecto.Plasm()
    plasm.connect(generate["out"] >> param_watcher["input"],
                  param_watcher['output']>>printer[:]
                  )
    plasm.insert(sleep)
    #ecto.view_plasm(plasm)
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute_async()
    while sched.running():
        number = 1000
        param_watcher.params.value = number
        time.sleep(3.0)
        sched.stop()
        
    assert 1000 == param_watcher.outputs.value

def test_async_stop_on_destructor():
    generate = ecto_test.Generate()
    param_watcher = ecto_test.ParameterWatcher(value=2)
    sleep = ecto_test.Sleep()
    printer = ecto_test.Printer()
    plasm = ecto.Plasm()
    plasm.connect(generate["out"] >> param_watcher["input"],
                  param_watcher['output']>>printer[:]
                  )
    plasm.insert(sleep)
    #ecto.view_plasm(plasm)
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute_async()
    time.sleep(3.0)

if __name__ == '__main__':
    test_parameter_callbacks()
    test_async_stop_on_destructor()



