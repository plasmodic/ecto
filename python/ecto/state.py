import ecto

def freeze_tendril(key,t):
    return dict(
        name=key,
        typename=t.type_name,
        val=str(t.val),
        doc=t.doc
        )

def freeze_params(plasm):
    '''Return a dict of cell parameters.'''
    cells = plasm.cells()
    params=[]
    for c in cells:
        p=dict(name=c.name(),
               typename=c.typename(),
               params=[freeze_tendril(x.key(),x.data()) for x in c.params]
               )
        params.append(p)
    return params
