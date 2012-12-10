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
import inspect

class BlackBoxAllTendrils(object):
    """
    This class contains the 3 different BlackBoxTendrils for a given BlackBox
    """
    def __init__(self, bb, bb_params):
        """
        :param bb: the BlackBox the tendrils are related to
        :param bb_params: the parameters with which the BlackBox constructor is called
        """
        self.bb = bb
        # for this dictionary, key=cell name, value= [cell class, parameters as dict]
        # the parameters are just the ones that will be sent to the constructor. They can
        # override the default ones in two ways: forwards and cell_params when doing a forward
        self.cell_definitions = {}

        # this dict has: key: BlackBox cell name, val: BlackBoxAllTendrils object
        self.blackbox_tendrils = {}

        # create the parameters
        self.params = BlackBoxTendrils(self.bb, ecto.tendril_type.PARAMETER, self)
        self.bb.declare_params(self.params)

        # override the values of the parameters from the kwargs
        for key, value in self.params.iteritems():
            value.val = bb_params.get(key, value.val)

        # update the parameters used to construct cells
        for cell_name, forward_list in self.params.forwards.items():
            cell_params = self.cell_definitions[cell_name][1]
            for key, cell_key in forward_list:
                if hasattr(self.params, key) and cell_key not in cell_params:
                    cell_params[cell_key] = self.params.at(key).val

        # deal with the inputs/outputs
        self.inputs = BlackBoxTendrils(self.bb, ecto.tendril_type.INPUT, self)
        self.outputs = BlackBoxTendrils(self.bb, ecto.tendril_type.OUTPUT, self)

    def update_cell_definition(self, cell_name, cell_class=None, cell_params={}):
        """
        Update the dictionary of cell definitions
        :param cell_name: the key in the dictionary
        :param cell_class: a cell class (Python or C++)
        :param cell_params: the parameters that should be used on the cell constructor
        """
        if cell_name in self.cell_definitions:
            if inspect.isclass(cell_class) and cell_class != self.cell_definitions[cell_name][0]:
                raise RuntimeError('%s was previously defined as %s but it is now %s' %
                                 (cell_name, self.cell_definitions[cell_name][0], cell_class))
        else:
            self.cell_definitions[cell_name] = [cell_class, {}, None]

        # now, update the parameters with cell_params
        for key, val in cell_params.items():
            self.cell_definitions[cell_name][1][key] = val

        # in case of a BlackBox, update the Tendrils too
        if hasattr(self.cell_definitions[cell_name][0], 'declare_params'):
            if self.cell_definitions[cell_name][2] is None:
                self.cell_definitions[cell_name][2] = BlackBoxAllTendrils(cell_class, self.cell_definitions[cell_name][1])
            for key, val in cell_params.items():
                setattr(self.cell_definitions[cell_name][2].params._tendrils, key, val)

    def create_cell(self, cell_name):
        """
        Based on information about cells we have gathered in cell_definitions, simply create a cell
        :param cell_name: the name of the cell to initialize
        """
        cell_class, cell_params, _cell_all_tendrils = self.cell_definitions[cell_name]
        return cell_class(cell_name, **cell_params)

    def get_blackbox_param_tendrils(self, cell_name):
        """
        :param cell_name: the name of the cell to look for the Tendrils in cell_definitions
        :return: a triplet param_tendrils, input_tendrils, output_tendrils
        """
        # update the parameters according to the constructor parameters
        for key, value in self.cell_definitions[cell_name][2].params.iteritems():
            value.val = self.cell_definitions[cell_name][1].get(key, value.val)
        return self.cell_definitions[cell_name][2].params

    def get_blackbox_io_tendrils(self, cell_name):
        """
        :param cell_name: the name of the cell to look for the Tendrils in cell_definitions
        :return: a triplet param_tendrils, input_tendrils, output_tendrils
        """
        # update the parameters according to the constructor parameters
        self.cell_definitions[cell_name][2].inputs._tendrils = ecto.Tendrils()
        self.cell_definitions[cell_name][2].outputs._tendrils = ecto.Tendrils()
        self.cell_definitions[cell_name][0].declare_io(self.get_blackbox_param_tendrils(cell_name), self.cell_definitions[cell_name][2].inputs, self.cell_definitions[cell_name][2].outputs)
        return self.cell_definitions[cell_name][2].inputs, self.cell_definitions[cell_name][2].outputs

    def solidify_forward_declares(self):
        # First, go over the different cell definitions and create a cell if needed
        for cell_name, _properties in self.cell_definitions.items():
            # make sure the BlackBox has a cell of the corresponding name/class
            if not hasattr(self.bb, cell_name):
                setattr(self.bb, cell_name, self.create_cell(cell_name))

        for blackbox_tendrils in [self.params, self.inputs, self.outputs]:
            tendrils = ecto.Tendrils()
            # First, go over the diferent forwards and make sure there is a cell for each
            for cell_name, keys in blackbox_tendrils.forwards.iteritems():
                cell = getattr(self.bb, cell_name)

                ctendrils = getattr(cell, blackbox_tendrils.tt_key)
                for key, cell_key in keys:
                    tendrils.declare(key, ctendrils.at(cell_key))
                    tendrils.at(key).doc = blackbox_tendrils._tendrils.at(key).doc
                    tendrils.at(key).copy_value(blackbox_tendrils._tendrils.at(key))#set the cells to parameters.
            # add the currently defined tendrils
            for key, tendril in blackbox_tendrils._tendrils:
                if not key in tendrils:
                    tendrils.declare(key, tendril)
            blackbox_tendrils._tendrils = tendrils


class BlackBoxTendrils(object):
    '''These look like tendrils with a few extra bits of functionality for BlackBoxes.
    '''
    tt_key = { ecto.tendril_type.INPUT: 'inputs',
               ecto.tendril_type.PARAMETER: 'params',
               ecto.tendril_type.OUTPUT: 'outputs' }

    def __init__(self, bb, tendril_type, all_tendrils=None):
        """
        :param bb: the BlackBox the tendrils are related to
        :param tendril_type: one of the tt_key
        :param all_tendrils: the params BlackBoxTendrils of the BlackBox
        """
        self._tendrils = ecto.Tendrils()
        self.tt = tendril_type
        self.tt_key = BlackBoxTendrils.tt_key[self.tt]
        self.bb = bb
        self.all_tendrils = all_tendrils
        self.forwards = {}

    def __getattribute__(self, name):
        #forward to the member _tendrils
        if name != '_tendrils' and name in dir(self._tendrils):
            return getattr(self._tendrils, name)
        else:
            return object.__getattribute__(self, name)

    def __append(self, cell_name, cell_class, key, cell_key):
        l = [(key, cell_key)]
        if cell_name in self.forwards :
            self.forwards[cell_name] += l
        else:
            self.forwards[cell_name] = l

    def __get_class_tendrils(self, cell_name=None, cell_class=None, cell_params={}):
        """
        Given an ecto cell class, get the tendrils that can be used for forwarding
        :param cell_name: the name of the cell to fetch from the BlackBox.
                          If not specified, you need a cell_class
        :param cell_class: a cell class but it can be an instance of that class too
        :param cell_params: if cell_params is given (a dictionary), then a cell of type cell_class
                       is instantiated
        :return: a tuple (cell class, cell tendrils)
        """
        if cell_class == None:
            # if no class is given, get it from the BlackBox using its name
            if not hasattr(self.bb.__class__, 'declare_cell_classes'):
                raise RuntimeError('The BlacBox of type %s must implement the declare_cell_classes method '
                                   'to define the class of the cell named "%s", or use the cell_class argument.' %
                                   (str(self.bb.__class__), cell_name))

            if self.tt == ecto.tendril_type.PARAMETER:
                cell_classes = self.bb.__class__.declare_cell_classes(self)
            else:
                cell_classes = self.bb.__class__.declare_cell_classes(self.all_tendrils.params)

            if cell_name not in cell_classes:
                raise RuntimeError('The cell %s must be defined in declare_cell_classes.' % cell_name)

            # make cell_class be a class object
            cell_class = cell_classes[cell_name]

        self.all_tendrils.update_cell_definition(cell_name, cell_class, cell_params)

        # if the class is a C++ class, it has no accessible declare_*
        if hasattr(cell_class, 'params'):
            if self.tt == ecto.tendril_type.PARAMETER:
                cell_tendrils = cell_class.params
            else:
                # the cell might already have been created but it can also be a class
                if inspect.isclass(cell_class):
                    cell_class = self.all_tendrils.create_cell(cell_name)
                # use the tendrils straight up if they are present
                if self.tt == ecto.tendril_type.INPUT:
                    cell_tendrils = cell_class.inputs
                elif self.tt == ecto.tendril_type.OUTPUT:
                    cell_tendrils = cell_class.outputs
        # otherwise, call the declare_params/declare_io
        elif self.tt == ecto.tendril_type.PARAMETER:
            cell_tendrils = self.all_tendrils.get_blackbox_param_tendrils(cell_name)
        else:
            # we are forwarding an INPUT or an OUTPUT
            input_tendrils, output_tendrils = self.all_tendrils.get_blackbox_io_tendrils(cell_name)
            if self.tt == ecto.tendril_type.INPUT:
                cell_tendrils = input_tendrils
            else:
                cell_tendrils = output_tendrils

        # now, return the class of the object
        if inspect.isclass(cell_class):
            return cell_class, cell_tendrils
        else:
            return cell_class.__class__, cell_tendrils

    def forward_all(self, cell_name, cell_class=None, cell_params={}):
        """
        Given the name of a class variable that is a cell, forward declare all
        its tendrils (params, inputs,outputs depending on context).
        :param cell_name: the name of the cell to forward tendrils from
        :param cell_class: the class of the cell to forward tendrils from.
                           It can also be defines in the declare_cell_classes() BlackBox function
        :param cell_params: the parameters to initialize the cell with (as a dict). This can be useful if
                           that cell is not initialized using public/forwarded parameters but
                           initialized in the configure function of the BlackBox. This needs to be specified
                           for inputs/ouputs only (not parameters).
        """
        if cell_params and self.tt == ecto.tendril_type.PARAMETER:
            raise RuntimeError('parameters for a cell can only be specified in declare_io')

        cell_class, cell_tendrils = self.__get_class_tendrils(cell_name, cell_class, cell_params)

        for key, val in cell_tendrils.iteritems():
            self.__append(cell_name, cell_class, key, key)
            self._tendrils.declare(key, val)

    def forward(self, key, cell_name, cell_key=None, doc=None, cell_params={}, cell_class=None):
        """
        If you forward a parameter, you probably do not want to create an instance of the forwarded
        class in your configure (or anywhere else). If it does not exist, it will automatically be
        created for you, using the initial values of the forwarded parameters.
        :param cell_name: the name of the cell to forward tendrils from
        :param cell_key: the specific tendril from that cell to forward. It can also be an array
        :param doc: the doc string to associate to that tendril (if None, the original ones are kept)
        :param cell_params: the parameters to initialize the cell with (as a dict). This can be useful if
                           that cell is not initialized using public/forwarded parameters but
                           initialized in the configure function of the BlackBox. This needs to be specified
                           for inputs/ouputs only (not parameters).
        :param cell_class: the class of the cell to forward tendrils from.
                           It can also be defines in the declare_cell_classes() BlackBox function
        """
        if cell_params and self.tt == ecto.tendril_type.PARAMETER:
            raise RuntimeError('parameters for a cell can only be specified in declare_io')

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

        # find the tendrils of the forwarded class
        cell_class, cell_tendrils = self.__get_class_tendrils(cell_name, cell_class, cell_params)

        # assign them temporarily to the final tendrils
        # they don't need to be linked to a cell yet
        for sub_key, sub_cell_key, sub_doc in zip(keys, cell_keys, docs):
            tendril = cell_tendrils.at(sub_cell_key)
            if sub_doc:
                tendril.doc = sub_doc
            self.__append(cell_name, cell_class, sub_key, sub_cell_key)
            self._tendrils.declare(sub_key, tendril)

class BlackBox(object):
    __looks_like_a_cell__ = True
    '''
    The BlackBox may be used as an encapsulation idiom within ecto, to declare reusable plasms.
    
    Users should inherit from BlackBox, and likely will wish to implement a few functions that
    describe their reusable plasm.
    '''

    def __init__(self, *args, **kwargs):
        self.niter = kwargs.get('niter', 1)
        self.__impl = None

        self.__all_tendrils = BlackBoxAllTendrils(self, kwargs)

        self.declare_io(self.__all_tendrils.params, self.__all_tendrils.inputs, self.__all_tendrils.outputs)
        self.__configure(self.__all_tendrils.params, self.__all_tendrils.inputs, self.__all_tendrils.outputs)
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
                                           parameters=self.__all_tendrils.params._tendrils,
                                           inputs=self.__all_tendrils.inputs._tendrils,
                                           outputs=self.__all_tendrils.outputs._tendrils)

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
        return cls(*args, **kwargs)

    @staticmethod
    def declare_params(p):
        '''The implementer of a BlackBox should create this method to declare any
        parameters or otherwise forward declare internal parameters.
        
        When this function is called, p will be an empty tendrils like object,
        with two additional functions, forward_all and forward.  This object
        is aware that is refers contextually to the parameters of the cell.
        
        forward has the signature:
            def forward(self, key, cell_name, cell_key=None, doc=None, cell_params=None):
        
        :param cell_name: the name of the cell to forward tendrils from
        :param cell_key: the specific tendril from that cell to forward
        :param doc: the doc string to associate to that tendril (if None, the original ones are kept)
        :param cell_params: the parameters to initialize the cell with (as a dict). This can be useful if
                           that cell is not initialed using public/forwarded parameters but
                           initialized in the configure function of the BlackBox
        :param cell_class: the class of the cell to forward tendrils from.
                           It can also be defines in the declare_cell_classes() BlackBox function
        
        forward_all has the signature:
            def forward_all(self, cell_name):

        This function has the behavor that all of the inputs, outputs, or parameters
        will be forwared, with their original keys, docs, etc..
        '''
        pass

    @staticmethod
    def declare_io(p, i, o):
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
        #underlying stuffs. This actually creates cells if needed
        p.all_tendrils.solidify_forward_declares()

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

    def cell(self):
        '''
        Return an instance of the cell that backs this
        BlackBox. Useful for ecto.If, or other places that expect a
        cell
        '''
        return self.__impl
