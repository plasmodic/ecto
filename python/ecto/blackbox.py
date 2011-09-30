import ecto

class BlackBox(object):
    '''
    The BlackBox may be used as an encapsulation idiom within ecto, to declare reusable plasms.
    
    Users should inherit from BlackBox, and likely will wish to implement a few functions that
    describe their reusable plasm.
    '''

    def __init__(self, *args, **kwargs):
        ''' The BlackBox must be created with the plasm it is intended to be connected
        to.
        '''
        self._cell = None
        self._is_plasm_connected = False
        self.niter = kwargs.get('niter', 1)
        self._parameters = ecto.Tendrils()
        self.declare_params(self._parameters)
        for key, value in self._parameters.iteritems():
            value.val = kwargs.get(key, value.val)

        self.init_cells(self._parameters)

        cellparams = self.expose_parameters()
        
        self._connect()
        
        #forward all of the parameters to our cell.
        for key, value in self._parameters.iteritems():
            self._cell.params.declare(key, value)

        #assign any kwargs to our parameters
        for key, value in self._cell.params.iteritems():
            value.val = kwargs.get(key, value.val)

        self._cell.name(self.__class__.__name__)
        self._gen_doc()
        if len(args) > 0:
            self._cell.name(args[0])

    def __getitem__(self, key):
        ''' This acts just like any other module when asking for a spec,
        e.g. spec = m['key1','key2']
        '''
        self._connect()
        return self._cell[key]

    def _gen_doc(self):
        short_doc = ''
        if self.__doc__:
            short_doc = self.__doc__
        self.__doc__ = self._cell.gen_doc(short_doc)

    def _connect(self):
        ''' Connect oneself to the plasm.
        '''
        if not self._is_plasm_connected:
            self._is_plasm_connected = True
            self._cell = self._create_cell()

    def __getattribute__(self, *args, **kwargs):
        if args[0] in ('__doc__', '_cell'):
            object.__getattribute__(self, '_connect')()
        return object.__getattribute__(self, *args, **kwargs)

    def __getattr__(self, name):
        if name in ('parameters',):
            name = 'params'
        if name in dir(self._cell):
            return getattr(self._cell, name)
        else:
            raise AttributeError(self, name)
    def __dir__(self):
        return ['outputs', 'inputs', 'params'] + self.__dict__.keys()

    def viz(self):
        ''' Display the graph viz of the Blackbox
        '''

    def view(self):
        ''' Display a GUI with the content of the
        '''

    def expose_outputs(self):
        ''' The outputs of a BlackBox should specified by returning a dictionary
         of string keys to TendrilsSpecifictation.
        {'output':self.mymodule['out_0001']}
        '''
        return {}

    def expose_inputs(self):
        ''' The inputs of a BlackBox should specified by returning a dictionary
        of string keys to TendrilsSpecifictation.
        {'input':self.mymodule['in_0001']}
        '''
        return {}

    def expose_parameters(self):
        ''' The parameters of a BlackBox should specified by returning a
        dictionary of string keys to TendrilsSpecifictation.
        {'param_01':self.mymodule['foo_param']}
        '''
        return {}

    def connections(self):
        '''This is where one should declare the graph, as a tuple of tuples, where each element 
        is (module_inst, 'output_key', module_inst, 'input_key')
        '''
        raise NotImplementedError("All BlackBox's must implement at least the connections function....")


    def declare_params(self, parameters):
        pass

    def init_cells(self, parameters):
        raise NotImplementedError('Initialize all your cells.')

    def _create_cell(self):
        plasm = ecto.Plasm()
        connections = self.connections()
        for x in connections:
            if not getattr(x, '__iter__', False):
                plasm.insert(x)
            else:
                plasm.connect(x)
        return ecto.create_black_box(plasm, niter=self.niter, parameters=self.expose_parameters(), inputs=self.expose_inputs(), outputs=self.expose_outputs())

    @classmethod
    def inspect(cls, *args, **kwargs):
        return cls()
