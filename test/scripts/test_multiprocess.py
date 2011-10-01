#!/usr/bin/env python
import ecto

import os
import sys
from multiprocessing import Process, Pipe, current_process, freeze_support
import StringIO

#http://docs.python.org/library/multiprocessing.html

class Sender(ecto.Cell):
    """ A python module that does not much."""
    @staticmethod
    def declare_params(params):
        params.declare("conn", "A pipe connection.", None)
        #params.declare("file", "A file-like object.", StringIO())

    @staticmethod
    def declare_io(params, inputs, outputs):
        #inputs.declare("file", "A file like object", None)
        pass

    def configure(self, params):
        pass

    def process(self, inputs, outputs):
        f = StringIO.StringIO()
        f.write('hello there')
        f.seek(0)
        self.params.conn.send(f)
        return 0

def f(conn):
    s = Sender(conn=conn)
    plasm = ecto.Plasm()
    plasm.insert(s)
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=10)

def g(conn):
    while True:
        file = conn.recv()
        for x in file:
            print x
            assert x == 'hello there'

if __name__ == '__main__':
    freeze_support()
    f_conn, g_conn = Pipe()
    p1 = Process(target=f, args=(f_conn,))
    p2 = Process(target=g, args=(g_conn,))

    p1.start()
    p2.start()
    p1.join()
    p2.terminate()
