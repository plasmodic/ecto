import ecto
           
class BlackBox(object):
    def __init__(self):
        self.tendrils = None
    def get_spec(self,key):
        i = self._inputs()
        o = self._outputs()
        exist_i = key in i.keys()
        exist_o = key in o.keys()
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
        l = []
        if type(key) == str:
            l.append(self.get_spec(key))
        elif type(key) in (list,tuple):
            for x in key:
                l.append(self.get_spec(x))
        else:
            raise TypeError( "May only use a tuple, list, or single string")
        return ecto.TendrilSpecifications(l)
     
    def getTendrils(self, name):
        if self.tendrils is None:
            self.tendrils = { 
                             "outputs" : ecto.TendrilSpecifications.to_tendrils(self._outputs(), 0),
                             "inputs" : ecto.TendrilSpecifications.to_tendrils(self._inputs(), 1),
                             "parameters" : ecto.TendrilSpecifications.to_tendrils(self._parameters(), 2)
                            }
        return self.tendrils[name]
    
    def __getattr__(self, name):
        cbs = {"outputs":self._outputs,
               "inputs":self._inputs,
               "parameters":self._parameters
               }
        if name in ("outputs", "inputs", "parameters"):
            return self.getTendrils(name)
        else:
            return object.__getattr__(self, name)
    def _outputs(self):
        return {}
    
    def _inputs(self):
        return {}
    
    def _parameters(self):
        return {}
    
    def connections(self):
        return []