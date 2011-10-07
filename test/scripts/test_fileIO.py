#!/usr/bin/env python

import ecto
import ecto_test
import StringIO

cards = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
filetext = '\n'.join([str(x) for x in cards]) + '\n'
def test_fileO(Scheduler, file_like_object, realfile=False):
    global cards, filetext
    plasm = ecto.Plasm()
    printer = ecto_test.FileO(file=ecto.ostream(file_like_object))
    dealer = ecto.Dealer(tendril=printer.inputs.at('input'), iterable=cards)
    plasm.connect(dealer['out'] >> printer['input'])
    sched = Scheduler(plasm)
    sched.execute()
    #sched.execute_async()
    #import time
    #time.sleep(0.5)
    #sched.wait()

    if not realfile:
        file_like_object.seek(0)
        result = ''.join([x for x in file_like_object])
        print result
        assert result == filetext

def test_fileI(Scheduler, file_like_object, realfile=False):
    global cards
    plasm = ecto.Plasm()
    if not realfile:
        file_like_object.writelines(filetext)
        file_like_object.seek(0)
    reader = ecto_test.FileI(file=ecto.istream(file_like_object))
    printer = ecto_test.Printer()
    plasm.connect(reader[:] >> printer[:])
    sched = Scheduler(plasm)
    sched.execute()
    assert reader.outputs.output == cards[-1]
def test_io_fake(Scheduler):
    test_fileO(Scheduler, StringIO.StringIO())
    test_fileI(Scheduler, StringIO.StringIO())
def test_io_real(Scheduler):
    with open('cards.txt', 'w') as f:
        test_fileO(Scheduler, f, realfile=True)
    with open('cards.txt', 'r') as f:
        test_fileI(Scheduler, f, realfile=True)
    import os
    os.remove('cards.txt')
def test_io_stdo(Scheduler=ecto.schedulers.Singlethreaded):
    import sys
    test_fileO(Scheduler, sys.stdout, realfile=True)
if __name__ == '__main__':
    for x in [ecto.schedulers.Singlethreaded, ecto.schedulers.Threadpool]:
        print " >>>>>>>>> Start sched >>>>>>>>>>", str(x)
        test_io_fake(x)
        test_io_real(x)
        test_io_stdo(x)
        print "<<<<<<<<<< End sched <<<<<<<<<<<", str(x)

