from ecto import _module_base

class Module(_module_base):
            
    def __init__(self, **kwargs):
        _module_base.__init__(self)

        _module_base.declare_params(self)

        for k, v in kwargs.iteritems():
            self.params[k].set(v)

        _module_base.declare_io(self)
        _module_base.configure(self)
        if self.__doc__ is None:
            self.__doc__ = "TODO docstr me."
        self.__doc__ = self.gen_doc(self.__doc__)
    @classmethod
    def inspect(cls,_args,_kwargs):
        m = cls()
        return m
        