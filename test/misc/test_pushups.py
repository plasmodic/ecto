#!/usr/bin/env python

import ecto
import push_ups

def test_pushups():
    plasm = ecto.Plasm()
    bd = push_ups.BigData()
    i = 0
    while (i < 1000):
        plasm.mark_dirty(bd)
        plasm.go(bd)
        i = i + 1

if __name__=="__main__":
    test_pushups()
