#!/usr/bin/env python
import i
import ecto
import pickle

class Foo:
    def __init__(self,x):
        self.x = x
def test_pickle():
    t = ecto.Tendril()
    print pickle.dumps(Foo(5))
if __name__ == '__main__':
    test_pickle()
