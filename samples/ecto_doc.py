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
        m = __import__(module)
        ms = [(module,m)]
        for loader, module_name, is_pkg in  pkgutil.walk_packages(m.__path__):
            #print loader,module_name,is_pkg
            module = loader.find_module(module_name).load_module(module_name)
            ms.append((module_name,module))
        
        ecto_cells = []
        for module_name,x in ms:
            print "..",module_name
            ecto_cells += ecto.list_ecto_module(x)
        #print ecto_cells
        
    else :
        print usage
