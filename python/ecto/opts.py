import argparse
import ecto

possible_schedulers = [ x for x in ecto.schedulers.__dict__.keys() if x[0] != '_']


def scheduler_options(parser):
    '''Creates an argument parser for ecto schedulers.
    
    '''
    parser.add_argument('--scheduler', metavar='SCHEDULER_TYPE', dest='scheduler_type', type=str, default='Singlethreaded',
                   help='Scheduler may be either '+ '|'.join(possible_schedulers) + '. Default is \'Singlethreaded\'')
    parser.add_argument('--nthreads', metavar='NUMBER_OF_THREADS', dest='nthreads', type=int, default=0,
                   help='For schedulers that use threading, this specifies the number of threads, 0 defaults to hardware concurrency information.')
    parser.add_argument('--niter', metavar='ITERATIONS', dest='niter', type=int, default=0,
                   help='Run the graph for niter iterations. 0, default, means run until stopped by a cell or external forces.')
    parser.add_argument('--shell', dest='ipython', action='store_const',
                        const=True, default=False,
                        help='Bring up an ipython prompt, and execute asynchronously.')

def use_ipython(options, sched, plasm):
    '''launch a plasm using ipython, and the scheduler of choice.
    @param options: from scheduler_options
    @par 
    '''
    from IPython.Shell import IPShellEmbed
    if type(sched) == ecto.schedulers.Singlethreaded:
        sched.execute_async(options.niter)
    else:
        sched.execute_async(options.niter, options.nthreads)
    ipshell = IPShellEmbed(['-prompt_in1', 'Input <\\#>', '-colors', 'LightBG'])
    ipshell()

def run_plasm(options, plasm):
    if options.scheduler_type not in possible_schedulers:
        msg = "You must supply a valid scheduler type, not \'%s\'\n" % options.scheduler_type
        msg += 'Valid schedulers are:\n\t' + '\n\t'.join(possible_schedulers) + '\n'
        raise RuntimeError(msg)
    sched = ecto.schedulers.__dict__[options.scheduler_type](plasm)
    if options.ipython:
        use_ipython(options, sched, plasm)
    else:
        if options.scheduler_type == 'Singlethreaded':
            sched.execute(options.niter)
        else:
            sched.execute(options.niter, options.nthreads)


if __name__ == '__main__':
    import ecto_test
    parser = argparse.ArgumentParser(description='My awesome program thing.')
    parser.add_argument('-i,--input', metavar='IMAGE_FILE', dest='imagefile', type=str, default='', help='an image file to load.')
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

    run_plasm(options, plasm)
