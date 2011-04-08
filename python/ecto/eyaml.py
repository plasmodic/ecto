from yaml import load, dump
try:
    from yaml import CLoader as Loader
    from yaml import CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
import ecto

def dump_plasm_params(plasm):
    vertices = plasm.vertices()

    params = {}
    for k,(x,t,name,tendril) in vertices.items():
        if(t == ecto.vertex_t.root):
            pl = {}
            for p in x.params:
                pl[p.key()] = p.data().val
            params["%s_%d"%(x.Name(),k)] = pl
    output = dump(params,Dumper=Dumper)
    return output

def load_plasm_params(plasm, stream):
    vertices = plasm.vertices()
    data = load(stream, Loader=Loader)
    params = {}
    for key,params in data.iteritems():
        n = int(key.split('_')[-1])
        module = vertices[n][0]
        for p,v in params.items():
            module.params[p].set(v)
    output = dump(params,Dumper=Dumper)
    return output

def config_plasm(plasm):
    """Configure all modules in a plasm"""
    vertices = plasm.vertices()
    for k,(x,t,name,tendril) in vertices.items():
        if(t == ecto.vertex_t.root):
            x.Config()