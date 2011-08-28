'''
Set of helper functions for ecto scripts, that expose a few common args for
the schedulers.
'''

import argparse
import ecto

possible_schedulers = [ x for x in ecto.schedulers.__dict__.keys() if x[0] != '_']

def use_ipython(options, sched, plasm, locals={}):
    '''Launch a plasm using ipython, and a scheduler of choice.
     
     options are from scheduler_options
     
     sched is an already initialized scheduler for plasm.
     
     plasm to execute
     
     locals are a dictionary of locals to forward to the ipython shell, use locals()
    '''
    #expose the locals to the ipython prompt.
    for key, val in locals.items():
        vars()[key] = val

    from IPython.Shell import IPShellEmbed
    if type(sched) == ecto.schedulers.Singlethreaded:
        sched.execute_async(options.niter)
    else:
        sched.execute_async(options.niter, options.nthreads)

    #the argv is important so that the IPShellEmbed doesn't use the global
    #Also fancy colors!!!!
    ipython_argv = ['-prompt_in1', 'Input <\\#>', '-colors', 'LightBG']
    ipshell = IPShellEmbed(ipython_argv)
    ipshell()

def run_plasm(options, plasm, locals={}):
    ''' run the plasm given the options from the command line parser.
    '''
    if options.scheduler_type not in possible_schedulers:
        msg = "You must supply a valid scheduler type, not \'%s\'\n" % options.scheduler_type
        msg += 'Valid schedulers are:\n\t' + '\n\t'.join(possible_schedulers) + '\n'
        raise RuntimeError(msg)
    if options.graphviz:
        ecto.view_plasm(plasm)
    sched = ecto.schedulers.__dict__[options.scheduler_type](plasm)
    if options.ipython:
        use_ipython(options, sched, plasm, locals)
    else:
        if options.scheduler_type == 'Singlethreaded':
            sched.execute(options.niter)
        else:
            sched.execute(options.niter, options.nthreads)

def scheduler_options(parser):
    '''Creates an argument parser for ecto schedulers.  Operates inplace on the
    given parser object.
    '''
    parser.add_argument('--scheduler', metavar='SCHEDULER_TYPE',
                        dest='scheduler_type', type=str, default='Singlethreaded',
                   help='Scheduler may be either ' + '|'.join(possible_schedulers) + '. Default is \'Singlethreaded\'')
    parser.add_argument('--nthreads', metavar='NUMBER_OF_THREADS', dest='nthreads', type=int, default=0,
                   help='For schedulers that use threading, this specifies the number of threads, 0 defaults to hardware concurrency information.')
    parser.add_argument('--niter', metavar='ITERATIONS', dest='niter', type=int, default=0,
                   help='Run the graph for niter iterations. 0, default, means run until stopped by a cell or external forces.')
    parser.add_argument('--shell', dest='ipython', action='store_const',
                        const=True, default=False,
                        help='Bring up an ipython prompt, and execute asynchronously.')
    parser.add_argument('--graphviz', dest='graphviz', action='store_const',
                        const=True, default=False,
                        help='Show the graphviz of the plasm.')

def doit(plasm, description="An ecto graph.", locals={}, args=None, namespace=None):
    '''doit is a short hand for samples that will just add
    ecto scheduler options, and nothing else.
       
        Use locals to forward any local variables to the ipython
        shell. Suggest either vars() or locals() to do this.
    '''
    parser = argparse.ArgumentParser(description=description)
    scheduler_options(parser)
    options = parser.parse_args(args, namespace)
    run_plasm(options, plasm, locals)

if __name__ == '__main__':
    import ecto_test
    parser = argparse.ArgumentParser(description='My awesome program thing.')
    parser.add_argument('-i,--input', metavar='IMAGE_FILE', dest='imagefile',
                        type=str, default='', help='an image file to load.')
    group = parser.add_argument_group('ecto scheduler options')
    scheduler_options(group)

    options = parser.parse_args()
    c = ecto.Constant(value=0.50505)
    m = ecto_test.Multiply(factor=3.3335)
    pr = ecto_test.Printer()
    plasm = ecto.Plasm()
    plasm.connect(c[:] >> m[:],
                  m[:] >> pr[:]
                  )

    run_plasm(options, plasm, locals=vars())
