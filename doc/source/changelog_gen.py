import subprocess,sys

tags = ('amoeba-beta2',
'amoeba-beta3',
'amoeba-beta4',
"amoeba-beta5",
"amoeba-beta6",
"HEAD",
)

pairs = []
for i in range(0,len(tags)-1):
    pairs += [(tags[i],tags[i+1])]
print pairs
pairs.reverse()
print '''
Changelog
=========
'''
for tag1,tag2 in pairs:
    header = '%s..%s'%(tag1,tag2)
    print header
    print '^' * len(header) + '\n'
    p = subprocess.Popen(['git',
                       'log',
                       '--oneline',
                       '--no-color',
                       '%s..%s'%(tag1,tag2)], stdout=subprocess.PIPE)
    print p.communicate()[0]