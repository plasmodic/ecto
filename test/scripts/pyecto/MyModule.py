import ecto

class MyModule(ecto.Module):
    """ A python module that does not much."""
    def __init__(self, *args, **kwargs):
        ecto.Module.__init__(self, **kwargs)
    
    @staticmethod
    def declare_params(params):
        params.declare("text", "a param.","hello there")

    @staticmethod
    def declare_io(params, inputs, outputs):
        inputs.declare("input","aye", 2)
        outputs.declare("out", "i'll give you this", "hello")
    
    def configure(self,params):
        self.text = params.text

    def process(self,inputs, outputs):
        c = int(inputs.input)
        outputs.out = c * self.text