
print "importing MmMmodule!"
print __name__

from ecto import module_base

print "MODULE_BASE=", module_base

class module(module_base):
    def __init__(self, *args, **kwargs):
        print "Constructing module", type(self)
        module_base.__init__(self)
        self.Params(self.params)

print "here module is", module        
