#!/usr/bin/env python
import ecto
import ecto_test

def test_parameter_callbacks():
    generate = ecto_test.Generate()
    param_watcher = ecto_test.ParameterWatcher(value=2)
    plasm = ecto.Plasm()
    plasm.connect(generate, "out", param_watcher, "input")
    
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=2)
    print "first value: ",param_watcher.params.value, param_watcher.outputs.output, generate.outputs.out
    assert param_watcher.params.value == 2
    assert param_watcher.outputs.output == 4
    assert generate.outputs.out == 2
    for i in range(0, 5):
        value = param_watcher.params.value * (i + 1);
        param_watcher.params.value = value
        print "execute..."
        sched.execute(niter=1)
        print "parameter:", param_watcher.outputs.value

    result = param_watcher.outputs.output
    print result
    assert param_watcher.outputs.value == 240
    assert param_watcher.outputs.output == 2880.0

if __name__ == '__main__':
    test_parameter_callbacks()



