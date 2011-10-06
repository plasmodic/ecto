import sys, traceback

def fail(s='(unspecified failure)'):
    print "****"
    print "****", s
    print "****"  
    print sys.exc_info()
    traceback.print_stack()
    sys.exit(666)
