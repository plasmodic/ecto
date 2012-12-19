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
import collections

# a BlackBoxForward is how forwarded tendrils are defined for a given cell: the tendril of name
# 'key' in the cell is forwarded as 'new_key' in the BlackBox and has the documentation 'doc'
# 'new_key' and 'doc' can be set to '' to indicate they use the same values as the original tendril
BlackBoxForward = collections.namedtuple('BlackBoxForward', ['key', 'new_key', 'doc'])

# The info of a class
# 'params' is a dictionary of parameters that will be sent to the
# constructor of the cell (with the forwarded parameters if any
# forwards is either a list of BlackBoxForward objects to indicate which parameters
# will be forwarded to the cell or the string 'all' is all parameters are forwarded
BlackBoxCellInfo = collections.namedtuple('BlackBoxCellInfo', ['python_class', 'params', 'forwards'])

def _deep_copy_tendrils(tendrils_in, values):
    """
    Given some tendrils and their values, deep copy them
    :param tendrils_in: an ecto.Tendrils()
    :param values: a dictionary {'tendril_key': tendril_value}
                  if 'tendril_key' is not in 'tendrils_in', it will be ignored
    :return: a deep copy of 'tendrils_in' with the values from 'values'
    """
    tendrils_out = ecto.Tendrils()
    for key, tendril in tendrils_in:
        new_tendril = ecto.Tendril.createT(tendril.type_name)
        if key in values:
            new_tendril.set(values[key])
        else:
            new_tendril.copy_value(tendril)
        new_tendril.doc = tendril.doc
        tendrils_out.declare(key, new_tendril)

    return tendrils_out

def _get_param_tendrils(cell_info, p):
    """
    :param cell_info: a BlackBoxCellInfo object
    :param p: tendrils
    :return: params Tendrils of a class with values overriden from p
    """
    # define the values of the parameters of the cell in cell_info
    # based on what will be sent to the constructor through
    # cell_info.params and what p already contains
    params_values = cell_info.params

    if cell_info.forwards == 'all':
        for key, tendril in p:
            params_values[key] = tendril.val
    else:
        for forward in cell_info.forwards:
            new_key = forward.new_key
            if new_key == '':
                new_key = forward.key
            if new_key in p:
                params_values[forward.key] = p.at(new_key).val

    if hasattr(cell_info.python_class, 'params'):
        tendrils = _deep_copy_tendrils(cell_info.python_class.params, params_values)
    else:
        # otherwise, call the declare_params function
        tendrils = ecto.Tendrils()
        cell_info.python_class.declare_params(tendrils, **params_values)

    return tendrils

def _deep_copy_tendrils_to_tendrils(cell_tendrils, forwards, tendrils):
    """
    Copies some tendrils from "cell_tendrils" to "tendrils" according to the
    BlackBoxForward's in "forwards".
    Contrary to _copy_tendrils, new tendrils are created and their attributes
    are copied over
    :param cell_tendrils: some ecto.Tendrils object from a cell. Values from
                          it will be copied to "tendrils"
    :param forwards: 
    """
    if forwards == 'all':
        for key, tendril in cell_tendrils:
            if key not in tendrils:
                new_tendril = ecto.Tendril.createT(tendril.type_name)
                new_tendril.copy_value(tendril)
                new_tendril.doc = tendril.doc

                tendrils.declare(key, new_tendril)
            else:
                tendrils.at(key).copy_value(tendril)
            tendrils.at(key).doc = tendril.doc
    else:
        for forward in forwards:
            key, new_key, doc = forward
            if key not in cell_tendrils:
                raise RuntimeError('No tendril "%s" found when declaring forward %s' %
                                    (key, str(forward)))
            if new_key == '':
                new_key = key
            # declare the new forwarded tendril
            tendril = cell_tendrils.at(key)
            if new_key not in tendrils:
                new_tendril = ecto.Tendril.createT(tendril.type_name)
                new_tendril.copy_value(tendril)
                new_tendril.doc = tendril.doc

                tendrils.declare(new_key, new_tendril)
            else:
                tendrils.at(new_key).copy_value(tendril)

            # update the docs if needed
            if doc:
                tendrils.at(new_key).doc = doc

def _copy_tendrils_to_tendrils(cell_tendrils, forwards, tendrils):
    """
    Copies some tendrils from "cell_tendrils" to "tendrils" according to the
    BlackBoxForward's in "forwards".
    :param cell_tendrils: some ecto.Tendrils object from a cell. Values from
                          it will be copied to "tendrils"
    :param forwards: 
    """
    if forwards == 'all':
        for key, tendril in cell_tendrils:
            tendrils.declare(key, tendril)
    else:
        for forward in forwards:
            key, new_key, doc = forward
            if key not in cell_tendrils:
                raise RuntimeError('No tendril "%s" found when declaring forward %s' %
                                    (key, str(forward)))
            if new_key == '':
                new_key = key
            # declare the new forwarded tendril
            tendrils.declare(new_key, cell_tendrils.at(key))

            # update the docs if needed
            if doc:
                tendrils.at(new_key).doc = doc

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

        self.__params = ecto.Tendrils()
        self.__inputs = ecto.Tendrils()
        self.__outputs = ecto.Tendrils()

        self.__configure(**kwargs)
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
                                           parameters=self.__params,
                                           inputs=self.__inputs,
                                           outputs=self.__outputs)

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
    def declare_direct_params(p, **kwargs):
        """
        This function can be overriden by a child class
        This function declares normal parameters for the BlackBox, i.e. you need to declare them
        using p.declare('key_name', 'key_docs', default_value) like you would in a normal
        Python cell
        :param p: an ecto.Tendrils() object
        :param kwargs: any sets of parameters that will be given to the BlackBox constructor
                       and that can be used to change the parameters accordingly. For example,
                       you can check the value of some kwargs and decide to declare some parameters
                       or others
        """
        pass

    @classmethod
    def declare_cells(cls, p):
        """
        This function can be overriden by a child class
        Given some parameters, define the characteristics of the cells
        :param p: an ecto.Tendrils() object
        :return: a dictionary of the form:
                    {'cell_name': BlackBoxCellInfo}
        """
        return {}

    @classmethod
    def declare_params(cls, p, **kwargs):
        """
        This method should not be overridden by a child class
        :param p: an ecto.Tendrils() object
        :param kwargs: any sets of parameters that will be given to the BlackBox constructor
                        and that can be used to change the parameters accordingly
        """
        # First, figure out the direct parameters
        cls.declare_direct_params(p, **kwargs)

        # complete the p using the default values given in **kwargs
        for key, val in kwargs.items():
            if key in p:
                p.at(key).set(val)

        # now that we know the parameters, define the cells accordingly
        cell_infos = cls.declare_cells(p)

        # parse the cell BlackBoxForwards and add them to the tendrils p calling
        # the declare_params from the cells or their .params to get tendril info
        for cell_name, cell_info in cell_infos.items():
            tendrils_params = _get_param_tendrils(cell_info, p)
            _deep_copy_tendrils_to_tendrils(tendrils_params, cell_info.forwards, p)

        # and update the values of the newly added tendrils by using
        # the default values given in **kwargs
        for key, val in kwargs.items():
            if key in p:
                p.at(key).set(val)

    @classmethod
    def declare_forwarded_io(cls, p):
        """
        This function can be overriden by a child class
        :return: a tuple of two dictionaries definining the input/output forwarded to/from the inner cells.
                    It has the format:
                    {'cell_name': 'all', 'cell_name': [BlackBoxForward1, BlackBoxForward2]}
        When actually called, that function will be overriden in __get_attribute__ so that io ecto.Tendrils()
        are actually filled with the information from the dictionary and from the cells (obtained with declare_cells)
        """
        return ({}, {})

    @classmethod
    def declare_io(cls, p, i, o):
        """
        This function has the same meaning as in C++ and should not be overriden by a child class
        """
        cell_infos = cls.declare_cells(p)
        forwarded_io = cls.declare_forwarded_io(p)
        for i in [0,1]:
            for cell_name, forwards in forwarded_io[i].items():
                cell_info = cell_infos[cell_name]
                forwarded_io[i][cell_name] = ecto.BlackBox.BlackBoxCellInfo(cell_info.python_class,
                                        cell_info.params, forwards)

        # go over the two sets of tendrils: i and o
        for info_tuple in [(forwarded_io[0], i, 'inputs'), (forwarded_io[1], o, 'outputs')]:
            cell_infos, tendrils, tendril_type = info_tuple
            for cell_name, info in cell_infos.items():
                python_class, _cell_params, cell_forwards = info
                if hasattr(python_class, tendril_type):
                    # get the tendrils from the class if it has them
                    cell_tendrils = _deep_copy_tendrils(getattr(python_class, tendril_type), {})
                else:
                    # in case the cell has no 'inputs'/'outputs' attribute (e.g. if it is a BlackBox
                    # or a pure Python cell)
                    cell_params = _get_param_tendrils(cell_info, p)
                    cell_tendrils = ecto.Tendrils()
                    if tendril_type == 'inputs':
                        python_class.declare_io(cell_params, cell_tendrils, ecto.Tendrils())
                    else:
                        python_class.declare_io(cell_params, ecto.Tendrils(), cell_tendrils)

                _deep_copy_tendrils_to_tendrils(cell_tendrils, cell_forwards, tendrils)

    def __configure(self, **kwargs):
        """
        Private implementation that generates the cells/tendrils inside the BlackBox
        """
        p = ecto.Tendrils()
        self.declare_params(p, **kwargs)

        # create the default cells
        cell_infos = self.declare_cells(p)
        for cell_name, cell_info in cell_infos.items():
            tendrils_params = _get_param_tendrils(cell_info, p)
            # convert it to a dictionary
            params = {}
            for key, tendril in tendrils_params.items():
                params[key] = tendril.val
            # add the cell to the BlackBox
            setattr(self, cell_name, cell_info.python_class(**params))

        # redefine the parameters so that they are linked to tendrils of actual cells
        self.__params = ecto.Tendrils()
        self.declare_direct_params(self.__params, **kwargs)
        # complete the params using the default values given in **kwargs
        for key, val in kwargs.items():
            if key in self.__params:
                self.__params.at(key).set(val)

        for cell_name, cell_info in cell_infos.items():
            cell = getattr(self, cell_name)
            _copy_tendrils_to_tendrils(cell.params, cell_info.forwards, self.__params)

        # redefine the io so that they are linked to tendrils of actual cells
        self.__inputs = ecto.Tendrils()
        self.__outputs = ecto.Tendrils()
        forwarded_io = self.declare_forwarded_io(self.__params)
        for cell_name, forwards in forwarded_io[0].items():
            cell = getattr(self, cell_name)
            _copy_tendrils_to_tendrils(cell.inputs, forwards, self.__inputs)
        for cell_name, forwards in forwarded_io[1].items():
            cell = getattr(self, cell_name)
            _copy_tendrils_to_tendrils(cell.outputs, forwards, self.__outputs)

        # call the configure from the user
        self.configure(self.__params, self.__inputs, self.__outputs)

    def configure(self, p, i, o):
        """
        This function has the same meaning as in C++ and can be overriden by a child class
        This function should be used to allocate all cells internal to a BlackBox
        """
        pass

    def connections(self):
        """
        This function has to be overriden by a child class
        The return value of this function should be an iterable of tendril connections.
        """
        raise NotImplementedError("All BlackBox's must implement atleast the connections function....")

    def cell(self):
        '''
        Return an instance of the cell that backs this
        BlackBox. Useful for ecto.If, or other places that expect a
        cell
        '''
        return self.__impl
