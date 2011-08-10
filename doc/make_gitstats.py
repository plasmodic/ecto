#!/usr/bin/env python

import sys, os
from subprocess import check_output as cmd

os.chdir(sys.argv[1])

revs = cmd('git rev-list HEAD'.split()).splitlines()
merges=cmd('git rev-list --merges HEAD'.split())

print len(revs), "commits"
print len(merges), "merges"
         
added = 0
deleted = 0
for rev in revs[:-2]:
    # print "========== %s ============" % rev
    thisone=cmd(('git --no-pager diff --numstat %s^..%s' % (rev, rev)).split())
    #print "thisone:", thisone
    lines = thisone.split('\n')
    print lines
    for line in thisone.split('\n')[:-1]:
        flds = line.split()
        print flds
        a, d, f = line.split()
        # print a, "added", d, "deleted"
        try:
            added += int(a)
            deleted += int(d)
        except ValueError:
            pass
    print "cumulative:", added, "added", deleted, "deleted"

print "total:", added, "added", deleted, " net:", added-deleted
