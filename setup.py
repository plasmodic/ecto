#!/usr/bin/env python

from distutils.core import setup
try:
    # Groovy and above
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup()
except:
    # For Fuerte
    def getText(nodelist):
        rc = []
        for node in nodelist:
            if node.nodeType == node.TEXT_NODE:
                rc.append(node.data)
        return ''.join(rc)

    import xml.dom.minidom
    file = open('stack.xml', 'r')
    dom = xml.dom.minidom.parseString(file.read())
    d = {}
    for tag_type in ['author', 'maintainer']:
        d[tag_type] = []
        for element in dom.getElementsByTagName(tag_type):
            d[tag_type].append(getText(element.childNodes))
        d[tag_type] = ', '.join(d[tag_type])
    for tag_type in ['name', 'license', 'url', 'version', 'description']:
        d[tag_type] = getText(dom.getElementsByTagName(tag_type)[0].childNodes)
    d['maintainer_email'] = 'vrabaud@willowgarage.com'
    d['keywords'] = ['ROS']

d['packages'] = ['ecto', 'ecto.sphinx', 'ecto.sphinx.breathe',
                'ecto.sphinx.breathe.finder', 'ecto.sphinx.breathe.finder.doxygen',
                'ecto.sphinx.breathe.parser', 'ecto.sphinx.breathe.parser.doxygen',
                'ecto.sphinx.breathe.renderer', 'ecto.sphinx.breathe.renderer.rst', 'ecto.sphinx.breathe.renderer.rst.doxygen'
               ]
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
