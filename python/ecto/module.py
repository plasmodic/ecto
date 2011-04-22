from ecto import _module_base

class Module(_module_base):

    def __init__(self, **kwargs):

        _module_base.__init__(self)

        self.Params(self.params)

        for k, v in kwargs.iteritems():
            self.params[k].set(v)

        self.config()
