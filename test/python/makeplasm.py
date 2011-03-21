#!/usr/bin/env python

import ecto, buster

def makeplasm():
    plasm = ecto.Plasm()
    g = buster.Generate()
    g.Params(g.params)
    g.params['step'].set(3.0)
    g.params['start'].set(4.0)
    g.Config()
    
    m = buster.Multiply()
    m.Params(m.params)
    m.params['factor'].set(13.0)
    m.Config()
    
    plasm.connect(g, 'out', m, 'in')
    
    m2 = buster.Multiply()
    m2.Params(m2.params)
    m2.params['factor'].set(14.0)
    m2.Config()
    
    plasm.connect(m, 'out', m2, 'in')

    print plasm
    return plasm

