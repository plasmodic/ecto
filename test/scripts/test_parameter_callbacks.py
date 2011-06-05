#!/usr/bin/env python
import ecto
import ecto_test

def test_parameter_callbacks():
    generate = ecto_test.Generate()
    param_watcher = ecto_test.ParameterWatcher(value=2)
    plasm = ecto.Plasm()
    plasm.connect(generate, "out", param_watcher, "input")

    sched = ecto.schedulers.Singlethreaded(plasm)
    for i in range(0, 5):
        value = param_watcher.params.value * (i + 1);
        param_watcher.params.value = value
        sched.execute()
        print "parameter:", param_watcher.outputs.value

    result = param_watcher.outputs.output
    print result
    assert param_watcher.outputs.value == 240
    assert param_watcher.outputs.output == 1920

if __name__ == '__main__':
    test_parameter_callbacks()



