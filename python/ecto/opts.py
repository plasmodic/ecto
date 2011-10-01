'''
Set of helper functions for ecto scripts, that expose a few common args for
the schedulers.
'''

import ecto
from copy import copy


possible_schedulers = [ x for x in ecto.schedulers.__dict__.keys() if x[0] != '_']

def use_ipython(options, sched, plasm, locals={}):
    '''Launch a plasm using ipython, and a scheduler of choice.
      -- options are from scheduler_options
      -- sched is an already initialized scheduler for plasm.
      -- plasm to execute
      -- locals are a dictionary of locals to forward to the ipython shell, use locals()
    '''
    #expose the locals to the ipython prompt.
    for key, val in locals.items():
        vars()[key] = val

    if type(sched) == ecto.schedulers.Singlethreaded:
        sched.execute_async(options.niter)
    else:
        sched.execute_async(options.niter, options.nthreads)

    import IPython
    if IPython.__version__ < '0.11':
        from IPython.Shell import IPShellEmbed
        #the argv is important so that the IPShellEmbed doesn't use the global
        #Also fancy colors!!!!
        ipython_argv = ['-prompt_in1', 'Input <\\#>', '-colors', 'LightBG']
        ipshell = IPShellEmbed(ipython_argv)
        ipshell()
    else:
        from IPython import embed
        embed() # this call anywhere in your program will start IPython

def run_plasm(options, plasm, locals={}):
    ''' run the plasm given the options from the command line parser.
    '''
    if options.scheduler_type not in possible_schedulers:
        msg = "You must supply a valid scheduler type, not \'%s\'\n" % options.scheduler_type
        msg += 'Valid schedulers are:\n\t' + '\n\t'.join(possible_schedulers) + '\n'
        raise RuntimeError(msg)
    if options.graphviz:
        ecto.view_plasm(plasm)
    if len(options.dotfile) > 0:
        print >> open(options.dotfile, 'wt'), plasm.viz()
    if len(options.logfile) > 0:
        ecto.log_to_file(options.logfile)
    sched = ecto.schedulers.__dict__[options.scheduler_type](plasm)
    if options.ipython:
        use_ipython(options, sched, plasm, locals)
    else:
        if options.scheduler_type == 'Singlethreaded':
            sched.execute(options.niter)
        else:
            sched.execute(options.niter, options.nthreads)

class CellFactory(object):
    '''A factory for cells that are created from command line args.'''
    def __init__(self, CellType, CellProtoType, prefix):
        '''
        cellType a type of ecto cell to create
        prefix the prefix used by the parser
        '''
        self.cellType = CellType
        self.cellProtoType = CellProtoType
        self.prefix = prefix
    def __call__(self, args, cellname=''):
        '''
        args from a parser where cell_options was used to add arguments.
        cellname option cellname
        '''
        params = {}
        prototype = self.cellProtoType
        for key, value in args.__dict__.iteritems():
            if key.startswith(self.prefix + '_'):
                p = ''.join(key.split(self.prefix + '_')[:])
                t = type(prototype.params[p])
                params[p] = t(value)
        nkwargs = ()
        if len(cellname) > 0:
            nkwargs = (cellname,) #tuple
        return self.cellType(*nkwargs, **params)

class CellYamlFactory(object):
    '''A factory for cells that are created from command line args.'''
    def __init__(self, CellOrCellType, prefix):
        cell_type, cell = _cell_type_instance(CellOrCellType)
        self.cell_type = cell_type
        self.cell = cell
        self.prefix = prefix
        params = cell.params
        c = {}
        for x in params:
            c[x.key()] = x.data().get()
            c[x.key()+'__doc__'] = x.data().doc
        self.params = c

    def dump(self, stream=None):
        from yaml import dump
        return dump({self.prefix:self.params}, default_flow_style=False,stream=stream)

    def load(self, parsed, cellname=''):
        params = parsed[self.prefix]
        nkwargs = ()
        params_clean = {}
        for x in self.cell.params:
            if x.key() in params:
                params_clean[x.key()] = params[x.key()]
        if len(cellname) > 0:
            nkwargs = (cellname,) #tuple
        cell = self.cell_type(*nkwargs, **params_clean)
        return cell

def _cell_type_instance(CellOrCellType):
    c = CellOrCellType
    cell_type = CellOrCellType
    if isinstance(cell_type, object.__class__):
        c = cell_type.inspect((), {})
    else:
        cell_type = c.__class__
    return (cell_type, c)



def cell_options(parser, CellType, prefix):
    '''Creates an argument parser group for any cell.
    CellType may be either a __class__ type, or an instance object.
    '''
    group = parser.add_argument_group('%s options' % prefix)
    cell_type, cell = _cell_type_instance(CellType)

    params = cell.params
    for x in params:
        dest = '%s_%s' % (prefix, x.key())
        group.add_argument('--%s' % dest, metavar='%s' % dest.upper(),
                           dest=dest, type=type(x.data().get()),
                           default=x.data().get(),
                           help=x.data().doc + ' ... (default: %(default)s)'
                           )
    factory = CellFactory(cell_type, cell, prefix)
    return factory

def scheduler_options(parser, default_scheduler='Singlethreaded',
                      default_nthreads=0,
                      default_niter=0,
                      default_shell=False,
                      default_graphviz=False):
    '''Creates an argument parser for ecto schedulers.  Operates inplace on the
    given parser object.
    '''
    parser.add_argument('--scheduler', metavar='SCHEDULER_TYPE',
                        dest='scheduler_type', type=str, default=default_scheduler,
                        choices=possible_schedulers,
                        help='The scheduler to execute the plasm with. (default: %(default)s)'
                        )
    parser.add_argument('--nthreads', metavar='NUMBER_OF_THREADS',
                        dest='nthreads', type=int, default=default_nthreads,
                        help='''For schedulers that use threading, this specifies
                        the number of threads, 0 defaults to hardware concurrency information.
                        (default: %(default)s)'''
                        )
    parser.add_argument('--niter', metavar='ITERATIONS', dest='niter',
                        type=int,
                        default=default_niter,
                        help='''Run the graph for niter iterations.
                        0 means run until stopped by a cell or external forces.
                        (default: %(default)s)'''
                        )
    parser.add_argument('--shell', dest='ipython', action='store_const',
                        const=True, default=default_shell,
                        help=''''Bring up an ipython prompt,
                        and execute asynchronously.(default: %(default)s)
                        '''
                        )
    parser.add_argument('--logfile', metavar='LOGFILE', dest='logfile', type=str,
                        default='',
                        help='''Log to the given file, use tail -f LOGFILE to see the
                       live output. May be useful in combination with --shell'''
                       )
    parser.add_argument('--graphviz', dest='graphviz',
                        action='store_const',
                        const=True, default=default_graphviz,
                        help='Show the graphviz of the plasm. (default: %(default)s)'
                        )
    parser.add_argument('--dotfile', dest='dotfile', type=str, default='',
                        help='''Output a graph in dot format to the given file.
                        If no file is given, no output will be generated. (default: %(default)s)'''
                        )

def doit(plasm, description="An ecto graph.", locals={}, args=None, namespace=None, default_scheduler='Singlethreaded', default_nthreads=0, default_niter=0, default_shell=False, default_graphviz=False):
    '''doit is a short hand for samples that will just add
    ecto scheduler options, and nothing else.
       
        Use locals to forward any local variables to the ipython
        shell. Suggest either vars() or locals() to do this.
    '''
    import argparse
    parser = argparse.ArgumentParser(description=description)
    scheduler_options(parser, default_scheduler=default_scheduler,
              default_nthreads=default_nthreads, default_niter=default_niter,
              default_shell=default_shell, default_graphviz=default_graphviz)
    options = parser.parse_args(args, namespace)
    run_plasm(options, plasm, locals)

if __name__ == '__main__':
    import ecto_test
    import yaml
    import argparse
    parser = argparse.ArgumentParser(description='My awesome program thing.')
    parser.add_argument('-i,--input', metavar='IMAGE_FILE', dest='imagefile',
                        type=str, default='', help='an image file to load.')
    group = parser.add_argument_group('ecto scheduler options')
    scheduler_options(group, default_niter=2)


    multiply_factory = cell_options(parser, ecto_test.Multiply, prefix='mult')
    const_factory = cell_options(parser, ecto.Constant(value=0.50505), prefix='const')

    #parser.print_help()
    options = parser.parse_args()

    c = const_factory(options)
    m = multiply_factory(options)
    cyaml = CellYamlFactory(c,'const')
    print cyaml.dump()
    c = cyaml.load(yaml.load(cyaml.dump()))
    pr = ecto_test.Printer()
    plasm = ecto.Plasm()
    plasm.connect(c[:] >> m[:],
                  m[:] >> pr[:]
                  )

    run_plasm(options, plasm, locals=vars())
