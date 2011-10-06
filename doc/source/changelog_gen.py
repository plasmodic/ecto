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
pairs.reverse()
print '''
Changelog
=========
This is a generated changelog file.
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
    lines = [x.split(' ',1) for x in p.communicate()[0].strip().split('\n') ]
    # '* `%(ref)s <https://github.com/plasmodic/ecto/commit/%(ref)s>`_ %(comment)s` %s'%
    lines = ['* `%(ref)s <https://github.com/plasmodic/ecto/commit/%(ref)s>`_ %(comment)s'%dict(ref=x[0],comment=x[1]) for x in lines]
    print '\n'.join(lines)
    print '\n'