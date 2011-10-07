#!/usr/bin/env python

import ecto
import ecto_test

def test_dealer(Scheduler):
    plasm = ecto.Plasm()
    printer = ecto_test.Printer()
    cards = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    dealer = ecto.Dealer(tendril=printer.inputs.at('in'), iterable=cards)
    plasm.connect(dealer['out'] >> printer['in'])
    sched = Scheduler(plasm)
    sched.execute()
    assert dealer.outputs.at('out').type_name == 'double'
    assert dealer.outputs.out == 10

def test_dealer_heterogenous_type_fail(Scheduler):
    printer = ecto_test.Printer()
    cards = [1, 2, 3, 4, 5, 'hello', 7, 8, 9, 10]
    dealer = ecto.Dealer(tendril=printer.inputs.at('in'), iterable=cards)
    plasm = ecto.Plasm()
    plasm.connect(dealer['out'] >> printer['in'])
    sched = Scheduler(plasm)
    try:
        sched.execute()
        assert False == " Should have thrown."
    except ecto.FailedFromPythonConversion, e:
        print "Threw as expected:", str(e)
        assert 'cpp_typename  double' in str(e)

if __name__ == '__main__':
    test_dealer(ecto.schedulers.Singlethreaded)
    test_dealer(ecto.schedulers.Threadpool)
    test_dealer_heterogenous_type_fail(ecto.schedulers.Singlethreaded)
