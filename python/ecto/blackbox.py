#!/bin/python
import ecto
import time

class DeclareModuleError(Exception):
    def __init__(self):
        pass
    def __str__(self):
        return 'need to declare modules'

class DefineInputOutputError(Exception):
    def __init__(self):
        pass
    def __str__(self):
        return 'need to define the inputs and outputs of the black box'

class BlackBox:
    """
    Base class implementing a meta module
    """
    # static dictionaries that have to be filled by each inheriting class
    # the key is the name of an input/output/parameter (as it will be used by other modules/black boxes),
    # and the value is the type of it. E.g. { "K_awesome": calib.CameraIntrinsics.outputs.K }
    PARAMETERS = {}
    INPUTS = {}
    OUTPUTS = {}
    def __init__(self, plasm):
        # contains the different Ecto modules. The key is the name of the module, the value the module itself
        self._modules = {}

        self.connections = {}

        self._declare_modules()
        self._define_input_ouput()
        self._connect(plasm)

    def __getitem__(self, names):
        """
        Get a specific module
        """
        return [ self._modules[name] for name in names ]

    def _connect(self, plasm):
        """
        Define the connections between the ecto modules in self._modules and the main plasm
        """
        raise ConnectError()

    def _declare_modules(self):
        """
        Pure virtual function to declare the different member ecto modules, without their interactions
        It basically fills self._modules
        """
        raise DeclareModuleError()

    def _define_input_ouput():
        """
        Pure virtual function to define the main outputs/inputs of the black box
        """
        raise DefineInputOutputError()
