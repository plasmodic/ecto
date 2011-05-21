from ecto import _module_base

class Module(_module_base):

    def __init__(self, **kwargs):

        _module_base.__init__(self)

        _module_base.initialize(self)

        for k, v in kwargs.iteritems():
            self.params[k].set(v)

        _module_base.configure(self)
