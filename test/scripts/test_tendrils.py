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
        util.fail()
    except ecto.TendrilRedeclaration, e:
        print str(e)
        assert('TendrilRedeclaration' in str(e))
    try:
        #read error
        t.nonexistant = 1
        util.fail()
    except ecto.NonExistant, e:
        print str(e)
        assert "tendril_key  nonexistant" in str(e)
    try:
        #index error
        print t["nonexistant"]
        util.fail()
    except ecto.NonExistant, e:
        print str(e)
        assert "tendril_key  nonexistant" in str(e)

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
