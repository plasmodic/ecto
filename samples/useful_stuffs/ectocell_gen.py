#!/usr/bin/env python

import ecto
import os
import pkgutil

ecto_cell_template ='''
.. ectocell:: %(submodule_name)s %(cell_name)s
'''

usage = """ Generates an sphinx rst for that uses the ecto sphinx extension.

ectocell_gen.py [my_ecto_module] > my_cells.rst

where my_ecto_module is in the PYTHONPATH and is a ecto python module.
"""
if __name__ == '__main__':
    if len(os.sys.argv) == 2:
        module_name = os.sys.argv[1]
        module = __import__(module_name)
        all_modules = [(module_name,module)]
        for loader, submodule_name, is_pkg in  pkgutil.walk_packages(module.__path__):
            #print loader,submodule_name,is_pkg
            module = loader.find_module(submodule_name).load_module(submodule_name)
            all_modules.append((submodule_name,module))
        
        for submodule_name,module in all_modules:
            if submodule_name != module_name:
                submodule_name = '.'.join([module_name,submodule_name])
                heading = '-'
            else:
                heading ='='
            print '.. _'+submodule_name +':\n' #reference link.
            print submodule_name +'\n' + heading*len(submodule_name) + '\n'
            ecto_cells = ecto.list_all_ecto_modules(module)
            for cell in ecto_cells:
                cell_name = cell.__class__.__name__
                print ecto_cell_template%locals()
    else :
        print usage
