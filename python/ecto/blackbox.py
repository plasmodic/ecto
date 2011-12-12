# 
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
import ecto

class BlackBoxTendrils(object):
    '''These look like tendrils with a few extra bits of functionality for BlackBoxes.
    '''
    tt_key = { ecto.tendril_type.INPUT: 'inputs',
               ecto.tendril_type.PARAMETER: 'params',
               ecto.tendril_type.OUTPUT: 'outputs' }

    def __init__(self, bb, tendril_type):
        self._tendrils = ecto.Tendrils()
        self.tt = tendril_type
        self.tt_key = BlackBoxTendrils.tt_key[self.tt]
        self.bb = bb
        self.forwards = {}

    def __getattribute__(self, name):
        #forward to the member _tendrils
        if name != '_tendrils' and name in dir(self._tendrils):
            return getattr(self._tendrils, name)
        else:
            return object.__getattribute__(self, name)

    def __get_cell_type(self, cell_name):
        cell_type = getattr(self.bb.__class__, cell_name, False)
        if not cell_type:
            cell_type = getattr(self.bb, cell_name, False)
            if cell_type:
                return cell_type
            raise RuntimeError("You must have a class variable called " + cell_name + " in your BlackBox.")
        return cell_type

    def __get_cell_template(self, cell_name):
        cell = getattr(self.bb, cell_name)
        cell_type = getattr(self.bb.__class__, cell_name, False)
        if cell == cell_type:
            cell = cell_type.inspect()
        return cell

    def __get_cell(self, cell_name):
        cell = self.__get_cell_template(cell_name)
        setattr(self.bb, cell_name, cell)
        return cell

    def __append(self, cell_name, key, cell_key):
        l = [(key, cell_key)]
        if cell_name in self.forwards :
            self.forwards[cell_name] += l
        else:
            self.forwards[cell_name] = l

    def forward_all(self, cell_name):
        '''
        Given the name of a class variable that is a cell, forward declare all
        its tendrils (params, inputs,outputs depending on context).
        '''
        inst = self.__get_cell_template(cell_name)
        for key, val in getattr(inst, self.tt_key):
            self.__append(cell_name, key, key)
            self._tendrils.declare(key, val)

    def forward(self, key, cell_name, cell_key=None, doc=None):
        inst = self.__get_cell_template(cell_name)
        tendrils = getattr(inst, self.tt_key)
        if cell_key:
            cell_keys = cell_key
        else:
            cell_keys = key
        # convert the inputs to arrays
        if type(key) is str:
            keys = [ key ]
            cell_keys = [ cell_keys ]
        else:
            keys = key
            cell_keys = cell_keys
        if doc:
            if type(doc) is str:
                docs = [ doc ]
            else:
                docs = doc
        else:
            docs = [ None ] * len(keys)
        if len(keys) != len(cell_keys):
            raise RuntimeError('keys and cell_keys must be arrays of the same length')
        if len(keys) != len(docs):
            raise RuntimeError('keys and docs must be arrays of the same length')

        for sub_key, sub_cell_key, sub_doc in zip(keys, cell_keys, docs):
            tendril = tendrils.at(sub_cell_key)
            if sub_doc:
                tendril.doc = sub_doc
            self.__append(cell_name, sub_key, sub_cell_key)
            self._tendrils.declare(sub_key, tendril)

    def solidify_forward_declares(self):
        tendrils = ecto.Tendrils()
        for cell_name, keys in self.forwards.iteritems():
            cell = self.__get_cell(cell_name)
            ctendrils = getattr(cell, self.tt_key)
            for key, cell_key in keys:
                tendrils.declare(key, ctendrils.at(cell_key))
                tendrils.at(key).copy_value(self._tendrils.at(key))#set the cells to parameters.
        for key, tendril in self._tendrils:
            if not key in tendrils:
                tendrils.declare(key, tendril)
        self._tendrils = tendrils


class BlackBox(object):
    __looks_like_a_cell__ = True
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
        self.__impl = None

        self.__params = BlackBoxTendrils(self, ecto.tendril_type.PARAMETER)
        self.__inputs = BlackBoxTendrils(self, ecto.tendril_type.INPUT)
        self.__outputs = BlackBoxTendrils(self, ecto.tendril_type.OUTPUT)

        self.declare_params(self.__params)

        #set the parameters from the kwargs.
        for key, value in self.__params.iteritems():
            value.val = kwargs.get(key, value.val)

        self.declare_io(self.__params, self.__inputs, self.__outputs)
        self.__configure(self.__params, self.__inputs, self.__outputs)
        self.__connect()
        self.__impl.name(self.__class__.__name__)
        self.__gen_doc()
        if len(args) > 0:
            self.__impl.name(args[0])

    def __getitem__(self, key):
        ''' This acts just like any other module when asking for a spec,
        e.g. spec = m['key1','key2']
        '''
        return self.__impl[key]

    def __gen_doc(self):
        short_doc = ''
        if self.__doc__:
            short_doc = self.__doc__
        self.__doc__ = self.__impl.gen_doc(short_doc)

    def __connect(self):
        ''' Connect oneself to the plasm.
        '''
        plasm = ecto.Plasm()
        connections = self.connections()
        for x in connections:
            if not getattr(x, '__iter__', False):
                plasm.insert(x)
            else:
                plasm.connect(x)
        self.__impl = ecto.create_black_box(plasm,
                                           niter=self.niter,
                                           parameters=self.__params._tendrils,
                                           inputs=self.__inputs._tendrils,
                                           outputs=self.__outputs._tendrils)

    def __getattribute__(self, *args, **kwargs):
        return object.__getattribute__(self, *args, **kwargs)

    def __getattr__(self, name):
        if name in ('parameters',):
            name = 'params'
        if '__impl' not in  name and name in dir(self.__impl):
            return getattr(self.__impl, name)
        if name == '__impl':
            return self.__impl
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

    def __configure(self, p, i, o):
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

