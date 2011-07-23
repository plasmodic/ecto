import ecto
           
class BlackBox(object):
    '''
    The BlackBox may be used as an encapsulation idium within ecto, to declare reusable plasms.
    
    Users should inherit from BlackBox, and likely will wish to implement a few functions that
    describe their reusable plasm.
    '''

    def __init__(self, plasm):
        ''' The BlackBox must be created with the plasm it is intended to be connected
        to.
        '''
        self.tendrils = None
        self._plasm = plasm
        self._is_plasm_connected = False

    def _get_spec(self,key):
        ''' Constructs a TendrilSpecification given a single str key.
        If the same key exists in the inputs and the outputs, a dichotomous spec is generated,
        otherwise, a spec is just retrieve from the mapping. 
        '''
        i = self.expose_inputs()
        o = self.expose_outputs()
        p = self.expose_parameters()
        exist_i = key in i.keys()
        exist_o = key in o.keys()
        #dichotomous spec 
        if exist_i and exist_o:
            ospec = o[key]
            ispec = i[key]
            return ecto.TendrilSpecification(ospec.module_out, ispec.module_in, key)
        elif exist_i:
            return i[key].to_spec()
        elif exist_o:
            return o[key].to_spec()
        else:
            raise RuntimeError(key + " does not exist!")
        
    def __getitem__(self, key):
        ''' This acts just like any other module when asking for a spec,
        e.g. spec = m["key1","key2"]
        '''
        l = []
        if type(key) == str:
            l.append(self._get_spec(key))
        elif type(key) in (list,tuple):
            for x in key:
                l.append(self._get_spec(x))
        else:
            raise TypeError( "May only use a tuple, list, or single string")
        
        self.connect()
        return ecto.TendrilSpecifications(l)
    
    def connect(self):
        ''' Connect oneself to the plasm.
        '''
        if self._plasm and not self._is_plasm_connected:
            self._plasm.connect(self.connections())
            self._is_plasm_connected = True
    
    def _getTendrils(self, name):
        ''' The blackbox should look like a module, so it needs to have the parameters, inputs, outputs as
        member fields.
        '''
        if self.tendrils is None:
            #transforms the TendrilsSpecifications to actual tendrils.
            self.tendrils = { 
                             "outputs" : ecto.TendrilSpecifications.to_tendrils(self.expose_outputs(), 0),
                             "inputs" : ecto.TendrilSpecifications.to_tendrils(self.expose_inputs(), 1),
                             "parameters" : ecto.TendrilSpecifications.to_tendrils(self.expose_parameters(), 2)
                            }
        return self.tendrils[name]
    
    def __getattr__(self, name):
        if name in ("outputs", "inputs", "parameters"):
            return self._getTendrils(name)
        else:
            return object.__getattr__(self, name)
    
    def viz(self):
        ''' Display the graph viz of the Blackbox
        '''
        
    def view(self):
        ''' Display a GUI with the content of the
        '''
        
    def expose_outputs(self):
        ''' The outputs of a BlackBox should specified by returning a dictionary of string keys to TendrilsSpecifictation.
        {"output":self.mymodule["out_0001"]}
        '''
        return {}
    
    def expose_inputs(self):
        ''' The inputs of a BlackBox should specified by returning a dictionary of string keys to TendrilsSpecifictation.
        {"input":self.mymodule["in_0001"]}
        '''
        return {}
    
    def expose_parameters(self):
        ''' The parameters of a BlackBox should specified by returning a dictionary of string keys to TendrilsSpecifictation.
        {"param_01":self.mymodule["foo_param"]}
        '''
        return {}
    
    def connections(self):
        '''This is where one should declare the graph, as a tuple of tuples, where each element 
        is (module_inst, 'output_key', module_inst, 'input_key')
        '''
        raise NotImplementedError("All BlackBox's must implement at least the connections function....")
