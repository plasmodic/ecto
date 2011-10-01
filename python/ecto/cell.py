from ecto import _cell_base

class Cell(_cell_base):
            
    def __init__(self, **kwargs):
        _cell_base.__init__(self)

        _cell_base.declare_params(self)

        for k, v in kwargs.iteritems():
            self.params.at(k).set(v)
        self.params.notify()
        _cell_base.declare_io(self)
        _cell_base.configure(self)
        if self.__doc__ is None:
            self.__doc__ = "TODO docstr me."
        self.__doc__ = self.gen_doc(self.__doc__)
    @classmethod
    def inspect(cls,_args,_kwargs):
        m = cls()
        return m

