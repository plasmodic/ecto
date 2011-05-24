#!/usr/bin/env python

import ecto
import os
import pkgutil

usage = """ Prints the doc strings for a given ecto module.

ecto_doc.py [my_ecto_module]

where my_ecto_module is in the PYTHONPATH and is a ecto extension.
"""
if __name__ == '__main__':
    if len(os.sys.argv) == 2:
        module = os.sys.argv[1]
        print "Inspecting",module
        m = __import__(module)
        ecto.list_ecto_module(m)
    else :
        print usage