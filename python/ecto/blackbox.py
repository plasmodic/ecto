import ecto

class _SpecialTendrils(object):
    tt_key = {ecto.tendril_type.INPUT:'inputs', ecto.tendril_type.PARAMETER:'params', ecto.tendril_type.OUTPUT:'outputs'}

    def __init__(self, bb, tendril_type):
        self._tendrils = ecto.Tendrils()
        self.tt = tendril_type
        self.tt_key = _SpecialTendrils.tt_key[self.tt]
        self.bb = bb
        self.forwards = {}


    def __getattribute__(self, name):
        #forward to the member _tendrils
        if name != '_tendrils' and name in dir(self._tendrils):
            return getattr(self._tendrils, name)
        else:
            return object.__getattribute__(self, name)

    def _get_cell_type(self, cell_name):
        cell_type = getattr(self.bb.__class__, cell_name, False)
        if not cell_type:
            raise RuntimeError("You must have a class variable called " + cell_name + " in your BlackBox.")
        return cell_type

    def _get_cell_template(self, cell_name):
        cell_type = self._get_cell_type(cell_name)
        return cell_type.inspect((), {})

    def _get_cell(self, cell_name):
        cell = getattr(self.bb, cell_name)
        cell_type = getattr(self.bb.__class__, cell_name)
        if not isinstance(cell, cell_type):
            raise RuntimeError("You must have a member variable of type " + self.bb.__class__.__name__ + " called " + cell_name)
        return cell

    def _append(self, cell_name, key, cell_key):
        l = [(key, cell_key)]
        if cell_name in self.forwards :
            self.forwards[cell_name] += l
        else:
            self.forwards[cell_name] = l

    def forward_all(self, cell_name):
        inst = self._get_cell_template(cell_name)
        for key, val in getattr(inst, self.tt_key):
            self._append(cell_name, key, key)
            self._tendrils.declare(key, val)

    def forward(self, key, cell_name, cell_key=None, doc=None):
        inst = self._get_cell_template(cell_name)
        tendrils = getattr(inst, self.tt_key)
        if not cell_key:
            cell_key = key
        tendril = tendrils.at(cell_key)
        if doc:
            tendril.doc = doc
        self._append(cell_name, key, cell_key)
        self._tendrils.declare(key, tendril)

    def solidify_forward_declares(self):
        tendrils = ecto.Tendrils()
        for cell_name, keys in self.forwards.iteritems():
            cell = self._get_cell(cell_name)
            ctendrils = getattr(cell, self.tt_key)
            for key, cell_key in keys:
                tendrils.declare(key, ctendrils.at(cell_key))
                setattr(tendrils, key, self._tendrils[key])#set the cells to parameters.
        for key, tendril in self._tendrils:
            if not key in tendrils:
                tendrils.declare(key, tendril)
        self._tendrils = tendrils


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
        self.niter = kwargs.get('niter', 1)

        self._params = _SpecialTendrils(self, ecto.tendril_type.PARAMETER)
        self._inputs = _SpecialTendrils(self, ecto.tendril_type.INPUT)
        self._outputs = _SpecialTendrils(self, ecto.tendril_type.OUTPUT)

        self.declare_params(self._params)

        #set the parameters from the kwargs.
        for key, value in self._params.iteritems():
            value.val = kwargs.get(key, value.val)

        self.declare_io(self._params, self._inputs, self._outputs)
        self._configure(self._params, self._inputs, self._outputs)
        self._connect()
        self._cell.name(self.__class__.__name__)
        self._gen_doc()
        if len(args) > 0:
            self._cell.name(args[0])

    def __getitem__(self, key):
        ''' This acts just like any other module when asking for a spec,
        e.g. spec = m['key1','key2']
        '''
        return self._cell[key]

    def _gen_doc(self):
        short_doc = ''
        if self.__doc__:
            short_doc = self.__doc__
        self.__doc__ = self._cell.gen_doc(short_doc)

    def _connect(self):
        ''' Connect oneself to the plasm.
        '''
        plasm = ecto.Plasm()
        connections = self.connections()
        for x in connections:
            if not getattr(x, '__iter__', False):
                plasm.insert(x)
            else:
                plasm.connect(x)
        self._cell = ecto.create_black_box(plasm,
                                           niter=self.niter,
                                           parameters=self._params._tendrils,
                                           inputs=self._inputs._tendrils,
                                           outputs=self._outputs._tendrils)

    def __getattribute__(self, *args, **kwargs):
        return object.__getattribute__(self, *args, **kwargs)

    def __getattr__(self, name):
        if name in ('parameters',):
            name = 'params'
        if name not in ('_cell') and name in dir(self._cell):
            return getattr(self._cell, name)
        else:
            raise AttributeError(self, name)
    def __dir__(self):
        return ['outputs', 'inputs', 'params'] + self.__dict__.keys()

    @classmethod
    def inspect(cls, *args, **kwargs):
        '''This emulates the inspect method that each ecto cell has, which creates a light
        version of the cell, where the default values for p,i,o are acceptable.
        '''
        return cls()

    def declare_params(self, p):
        '''The implementer of a BlackBox should create this method to declare any 
        parameters or otherwise forward declare internal parameters.
        
        When this function is calls, p will be an empty tendrils like object,
        with two additional functions, forward_all and forward.  This object
        is aware that is refers contextually to the parameters of the cell.
        
        forward has the signature:
            def forward(self, key, cell_name, cell_key=None, doc=None):
        
        :key: the externally usable name of the given tendril
        :cell_name: is the name of a class variable that refers to a cell like type.
        :cell_key: is cell_name's tendril that will be remapped to key, 
            if None then it is assumed that the key is also the cell_key
        :doc: If not None, this overrides the given tendril's doc string.
        
        forward_all has the signature:
            def forward_all(self, cell_name):
        This function has the behavor that all of the inputs, outputs, or parameters
        will be forwared, with their original keys, docs, etc..
        '''
        pass

    def declare_io(self, p, i, o):
        '''
        This function will be called after declare_params, and before configure.
        Here all inputs and outputs should be forward declared.  This parameters
        may be looked at here to determine custom declaration code, etc...
        
        See the declare_params for docs on what p, i, and o are. Each one corresponds to
        the parameters, inputs, and outputs of the black box.
        '''
        pass

    def _configure(self, p, i, o):
        self.configure(p, i, o)
        #take  all of the forward declares and make them point to the actual
        #underlying stuffs.
        p.solidify_forward_declares()
        i.solidify_forward_declares()
        o.solidify_forward_declares()

    def configure(self, p, i, o):
        ''' 
        This function should be used to allocate all cells that are forward declared.
        After this function exits, the BlackBox will try to solidify all the forward declarations.
        '''
        pass

    def connections(self):
        '''The return value of this function should be an iterable of tendril connections.
        '''
        raise NotImplementedError("All BlackBox's must implement atleast the connections function....")
