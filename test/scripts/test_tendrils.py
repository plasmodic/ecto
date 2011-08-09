#!/usr/bin/env python
import ecto
import ecto_test

def test_tendrils():
    t = ecto.Tendrils()
    t.declare("Hello","doc str",6)
    assert t.Hello == 6
    assert t["Hello"] == 6
    t.declare("x","a number", "str")
    assert len(t) == 2
    assert t["x"] == "str"
    assert t.x == "str"
    #test the redeclare
    try:
        t.declare("Hello","new doc", "you")
    except RuntimeError, e:
        assert('You can\'t redeclare a tendril!' in str(e))
    try:
        #read error
        t.nonexistant = 1
        assert False
    except RuntimeError,e:
        assert "'nonexistant' does not exist in this tendrils object." in str(e)
    try:
        #index error
        print t["nonexistant"]
        assert False
    except RuntimeError,e:
        assert "'nonexistant' does not exist in this tendrils object." in str(e)

    assert len(t.keys()) == 2
    assert len(t.values()) == 2
    
    print t
    #by value
    _x = t.x
    _x = 10
    assert t.x != 10
    x = t.x
    t.x = 11
    assert x != 11
    #by reference
    x = t.at("x")
    t.x = 13
    assert x.val == 13
    
    t.x = 17
    assert t.x == 17
    t.x = 199
    t.x = 15
    print t.x
    assert t.x == 15
    
if __name__ == '__main__':
    test_tendrils()
