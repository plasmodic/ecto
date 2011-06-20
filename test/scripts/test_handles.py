#!/usr/bin/env python
import ecto
import ecto_test

def test_parameter_callbacks():
    generate = ecto_test.Generate()
    handle_holder = ecto_test.HandleHolder(value=2)
    plasm = ecto.Plasm()
    plasm.connect(generate, "out", handle_holder, "input")

    sched = ecto.schedulers.Singlethreaded(plasm)
    for i in range(0, 5):
        value = handle_holder.params.value * (i + 1);
        handle_holder.params.value = value
        print "execute..."
        sched.execute(niter=1)
        print "parameter:", handle_holder.outputs.value

    result = handle_holder.outputs.output
    print result
    assert handle_holder.outputs.value == 240
    assert handle_holder.outputs.output == 1920

if __name__ == '__main__':
    test_parameter_callbacks()



