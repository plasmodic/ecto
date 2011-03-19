class Tendrils(object): pass

def set_tendril_attribute(sub, tendrils):
    for el in tendrils:
        #print el.key(), el.data()
        setattr(sub, el.key(), el.data())

def in_out_magic(module):
    module.i = Tendrils()
    module.o = Tendrils()
    module.p = Tendrils()
    set_tendril_attribute(module.i, module.inputs)
    set_tendril_attribute(module.o, module.outputs)
    set_tendril_attribute(module.p, module.params)

def config(module,**kwargs):
    """
    Given an ecto module, and kward args, it sets the params
    and configures the module
    code sample::
        s = ecto.make(MyModule)
        ecto.config(s, n=5, who="what")
    """
    for k,v in kwargs.iteritems():
        module.params[k].set(v)
    module.Config()
    in_out_magic(module)

def make(module_t,**kwargs):
    m = module_t()
    module_t.Params(m.params)
    config(m,**kwargs)
    return m

def keys(tendrils):
    x = []
    for el in tendrils:
        x.append(el.key())
    return x
