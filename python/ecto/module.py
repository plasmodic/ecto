from ecto import module_base

class module(module_base):

    def __init__(self, *args, **kwargs):

        module_base.__init__(self)

        self.Params(self.params)

        for k, v in kwargs.iteritems():
            self.params[k].set(v)

        self.Config()
