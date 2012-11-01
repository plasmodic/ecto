#!/usr/bin/env python

from __future__ import print_function
from distutils.core import setup




"""
THIS FILE IS A TOTAL COPY OF THE package.py IN catkin-pkg AND IS ONLY
MEANT FOR FUERTE COMPATIBILITY (WHICH DOES NOT HAVE catkin-pkg).
IT WILL BE REMOVED ONCE FUERTE IS NOT SUPPORTED ANYMORE

Library for parsing package.xml and providing an object
representation.
"""


import os
import re
import sys
import xml.dom.minidom as dom

PACKAGE_MANIFEST_FILENAME = 'package.xml'


class Package(object):
    """
    Object representation of a package manifest file
    """
    __slots__ = [
        'package_format',
        'name',
        'version',
        'version_abi',
        'description',
        'maintainers',
        'licenses',
        'urls',
        'authors',
        'build_depends',
        'buildtool_depends',
        'run_depends',
        'test_depends',
        'conflicts',
        'replaces',
        'exports',
        'filename'
    ]

    def __init__(self, filename=None, **kwargs):
        """
        :param filename: location of package.xml.  Necessary if
          converting ``${prefix}`` in ``<export>`` values, ``str``.
        """
        # initialize all slots ending with "s" with lists, all other with plain values
        for attr in self.__slots__:
            if attr.endswith('s'):
                value = list(kwargs[attr]) if attr in kwargs else []
                setattr(self, attr, value)
            else:
                value = kwargs[attr] if attr in kwargs else None
                setattr(self, attr, value)
        self.filename = filename
        # verify that no unknown keywords are passed
        unknown = set(kwargs.keys()).difference(self.__slots__)
        if unknown:
            raise TypeError('Unknown properties: %s' % ', '.join(unknown))

    def __getitem__(self, key):
        if key in self.__slots__:
            return getattr(self, key)
        raise KeyError('Unknown key "%s"' % key)

    def __iter__(self):
        for slot in self.__slots__:
            yield slot

    def __str__(self):
        data = {}
        for attr in self.__slots__:
            data[attr] = getattr(self, attr)
        return str(data)

    def validate(self):
        """
        makes sure all standards for packages are met
        :param package: Package to check
        :raises InvalidPackage: in case validation fails
        """
        errors = []
        if self.package_format:
            if not re.match('^[1-9][0-9]*$', str(self.package_format)):
                errors.append('The "format" attribute of the package must contain a positive integer if present')

        if not self.name:
            errors.append('Package name must not be empty')
        # accepting upper case letters and hyphens only for backward compatibility
        if not re.match('^[a-zA-Z0-9][a-zA-Z0-9_-]*$', self.name):
            errors.append('Package name "%s" does not follow naming conventions' % self.name)
        elif not re.match('^[a-z][a-z0-9_]*$', self.name):
            print('WARNING: Package name "%s" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits and underscores.' % self.name, file=sys.stderr)

        if not self.version:
            errors.append('Package version must not be empty')
        elif not re.match('^[0-9]+\.[0-9_]+\.[0-9_]+$', self.version):
            errors.append('Package version "%s" does not follow version conventions' % self.version)

        if not self.description:
            errors.append('Package description must not be empty')

        if not self.maintainers:
            errors.append('Package must declare at least one maintainer')
        for maintainer in self.maintainers:
            try:
                maintainer.validate()
            except InvalidPackage as e:
                errors.append(str(e))
            if not maintainer.email:
                errors.append('Maintainers must have an email address')

        if not self.licenses:
            errors.append('The package node must contain at least one "license" tag')

        if self.authors is not None:
            for author in self.authors:
                try:
                    author.validate()
                except InvalidPackage as e:
                    errors.append(str(e))

        for dep_type, depends in {'build': self.build_depends, 'buildtool': self.buildtool_depends, 'run': self.run_depends, 'test': self.test_depends}.items():
            for depend in depends:
                if depend.name == self.name:
                    errors.append('The package must not "%s_depend" on a package with the same name as this package' % dep_type)
        if errors:
            raise InvalidPackage('\n'.join(errors))


class Dependency(object):
    __slots__ = ['name', 'version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']

    def __init__(self, name, **kwargs):
        for attr in self.__slots__:
            value = kwargs[attr] if attr in kwargs else None
            setattr(self, attr, value)
        self.name = name
        # verify that no unknown keywords are passed
        unknown = set(kwargs.keys()).difference(self.__slots__)
        if unknown:
            raise TypeError('Unknown properties: %s' % ', '.join(unknown))

    def __str__(self):
        return self.name


class Export(object):
    __slots__ = ['tagname', 'attributes', 'content']

    def __init__(self, tagname, content=None):
        self.tagname = tagname
        self.attributes = {}
        self.content = content

    def __str__(self):
        txt = '<%s' % self.tagname
        for key in sorted(self.attributes.keys()):
            txt += ' %s="%s"' % (key, self.attributes[key])
        if self.content:
            txt += '>%s</%s>' % (self.content, self.tagname)
        else:
            txt += '/>'
        return txt


class Person(object):
    __slots__ = ['name', 'email']

    def __init__(self, name, email=None):
        self.name = name
        self.email = email

    def __str__(self):
        return '%s <%s>' % (self.name, self.email) if self.email is not None else self.name

    def validate(self):
        if self.email is None:
            return
        if not re.match('^[a-zA-Z0-9._%-]+@[a-zA-Z0-9._%-]+\.[a-zA-Z]{2,6}$', self.email):
            raise InvalidPackage('Invalid email "%s" for person "%s"' % (self.email, self.name))


class Url(object):
    __slots__ = ['url', 'type']

    def __init__(self, url, type_=None):
        self.url = url
        self.type = type_

    def __str__(self):
        return self.url


def parse_package_for_distutils(path=None):
    """
    Extract the information relevant for distutils from the package
    manifest.  It sets the following keys: name, version, maintainer,
    long_description, license, keywords.

    The following keys depend on information which are
    optional: author, author_email, maintainer_email, url

    :param path: The path of the package.xml file, it may or may not
    include the filename

    :returns: return dict populated with parsed fields
    :raises: :exc:`InvalidPackage`
    :raises: :exc:`IOError`
    """
    if path is None:
        path = '.'
    package = parse_package(path)

    data = {}
    data['name'] = package.name
    data['version'] = package.version

    # either set one author with one email or join all in a single field
    if len(package.authors) == 1 and package.authors[0].email is not None:
        data['author'] = package.authors[0].name
        data['author_email'] = package.authors[0].email
    else:
        data['author'] = ', '.join([('%s <%s>' % (a.name, a.email) if a.email is not None else a.name) for a in package.authors])

    # either set one maintainer with one email or join all in a single field
    if len(package.maintainers) == 1:
        data['maintainer'] = package.maintainers[0].name
        data['maintainer_email'] = package.maintainers[0].email
    else:
        data['maintainer'] = ', '.join(['%s <%s>' % (m.name, m.email) for m in package.maintainers])

    # either set the first URL with the type 'website' or the first URL of any type
    websites = [url.url for url in package.urls if url.type == 'website']
    if websites:
        data['url'] = websites[0]
    elif package.urls:
        data['url'] = package.urls[0].url

    if len(package.description) <= 200:
        data['description'] = package.description
    else:
        data['description'] = package.description[:200]
        data['long_description'] = package.description

    #data['classifiers'] = ['Programming Language :: Python']

    data['license'] = ', '.join(package.licenses)
    data['keywords'] = ['ROS']
    return data


class InvalidPackage(Exception):
    pass


def parse_package(path):
    """
    Parse package manifest.

    :param path: The path of the package.xml file, it may or may not
    include the filename

    :returns: return :class:`Package` instance, populated with parsed fields
    :raises: :exc:`InvalidPackage`
    :raises: :exc:`IOError`
    """
    if os.path.isfile(path):
        filename = path
    elif os.path.isdir(path):
        filename = os.path.join(path, PACKAGE_MANIFEST_FILENAME)
        if not os.path.isfile(filename):
            raise IOError('Directory "%s" does not contain a "%s"' % (path, PACKAGE_MANIFEST_FILENAME))
    else:
        raise IOError('Path "%s" is neither a directory containing a "%s" file nor a file' % (path, PACKAGE_MANIFEST_FILENAME))

    with open(filename, 'r') as f:
        try:
            return parse_package_string(f.read(), filename)
        except InvalidPackage as e:
            e.args = ['Invalid package manifest "%s": %s' % (filename, e.message)]
            raise


def parse_package_string(data, filename=None):
    """
    Parse package.xml string contents.

    :param data: package.xml contents, ``str``
    :param filename: full file path for debugging, ``str``
    :returns: return parsed :class:`Package`
    :raises: :exc:`InvalidPackage`
    """
    try:
        d = dom.parseString(data)
    except Exception as e:
        raise InvalidPackage('The manifest contains invalid XML:\n%s' % e)

    pkg = Package(filename)

    # verify unique root node
    nodes = _get_nodes(d, 'package')
    if len(nodes) != 1:
        raise InvalidPackage('The manifest must contain a single "package" root tag')
    root = nodes[0]

    # format attribute
    value = _get_node_attr(root, 'format', default=1)
    pkg.package_format = int(value)

    # name
    pkg.name = _get_node_value(_get_node(root, 'name'))

    # version and optional abi
    version_node = _get_node(root, 'version')
    pkg.version = _get_node_value(version_node)
    pkg.version_abi = _get_node_attr(version_node, 'abi', default=None)

    # description
    pkg.description = _get_node_value(_get_node(root, 'description'), allow_xml=True)

    # at least one maintainer, all must have email
    maintainers = _get_nodes(root, 'maintainer')
    for node in maintainers:
        pkg.maintainers.append(Person(
            _get_node_value(node, apply_str=False),
            _get_node_attr(node, 'email')
        ))

    # urls with optional type
    urls = _get_nodes(root, 'url')
    for node in urls:
        pkg.urls.append(Url(
            _get_node_value(node),
            _get_node_attr(node, 'type', default='website')
        ))

    # authors with optional email
    authors = _get_nodes(root, 'author')
    for node in authors:
        pkg.authors.append(Person(
            _get_node_value(node, apply_str=False),
            _get_node_attr(node, 'email', default=None)
        ))

    # at least one license
    licenses = _get_nodes(root, 'license')
    for node in licenses:
        pkg.licenses.append(_get_node_value(node))

    # dependencies and relationships
    pkg.build_depends = _get_dependencies(root, 'build_depend')
    pkg.buildtool_depends = _get_dependencies(root, 'buildtool_depend')
    pkg.run_depends = _get_dependencies(root, 'run_depend')
    pkg.test_depends = _get_dependencies(root, 'test_depend')
    pkg.conflicts = _get_dependencies(root, 'conflict')
    pkg.replaces = _get_dependencies(root, 'replace')

    # exports
    export_node = _get_optional_node(root, 'export')
    if export_node is not None:
        exports = []
        for node in [n for n in export_node.childNodes if n.nodeType == n.ELEMENT_NODE]:
            export = Export(str(node.tagName), _get_node_value(node, allow_xml=True))
            for key, value in node.attributes.items():
                export.attributes[str(key)] = str(value)
            exports.append(export)
        pkg.exports = exports

    errors = []
    # verify that no unsupported tags and attributes are present
    unknown_root_attributes = [attr for attr in root.attributes.keys() if str(attr) != 'format']
    if unknown_root_attributes:
        errors.append('The "package" tag must not have the following attributes: %s' % ', '.join(unknown_root_attributes))
    depend_attributes = ['version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']
    known = {
        'name': [],
        'version': ['abi'],
        'description': [],
        'maintainer': ['email'],
        'license': [],
        'url': ['type'],
        'author': ['email'],
        'build_depend': depend_attributes,
        'buildtool_depend': depend_attributes,
        'run_depend': depend_attributes,
        'test_depend': depend_attributes,
        'conflict_depend': depend_attributes,
        'replace_depend': depend_attributes,
        'export': [],
    }
    nodes = [n for n in root.childNodes if n.nodeType == n.ELEMENT_NODE]
    unknown_tags = [n.tagName for n in nodes if n.tagName not in known.keys()]
    if unknown_tags:
        errors.append('The manifest must not contain the following tags: %s' % ', '.join(unknown_tags))
    for node in [n for n in nodes if n.tagName in known.keys()]:
        unknown_attrs = [str(attr) for attr in node.attributes.keys() if str(attr) not in known[node.tagName]]
        if unknown_attrs:
            errors.append('The "%s" tag must not have the following attributes: %s' % (node.tagName, ', '.join(unknown_attrs)))
        if node.tagName not in ['description', 'export']:
            subnodes = [n for n in node.childNodes if n.nodeType == n.ELEMENT_NODE]
            if subnodes:
                errors.append('The "%s" tag must not contain the following children: %s' % (node.tagName, ', '.join([n.tagName for n in subnodes])))
    if errors:
        # for now only output a warning instead of raising an exception
        #raise InvalidPackage('\n'.join(errors))
        print('WARNING:%s' % ''.join(['\n- %s' % e for e in errors]), file=sys.stderr)

    pkg.validate()

    return pkg


def _get_nodes(parent, tagname):
    return [n for n in parent.childNodes if n.nodeType == n.ELEMENT_NODE and n.tagName == tagname]


def _get_node(parent, tagname):
    nodes = _get_nodes(parent, tagname)
    if len(nodes) != 1:
        raise InvalidPackage('The manifest must contain exactly one "%s" tags' % tagname)
    return nodes[0]


def _get_optional_node(parent, tagname):
    nodes = _get_nodes(parent, tagname)
    if len(nodes) > 1:
        raise InvalidPackage('The manifest must not contain more than one "%s" tags' % tagname)
    return nodes[0] if nodes else None


def _get_node_value(node, allow_xml=False, apply_str=True):
    if allow_xml:
        value = (''.join([n.toxml() for n in node.childNodes])).strip(' \n\r\t')
    else:
        value = (''.join([n.data for n in node.childNodes if n.nodeType == n.TEXT_NODE])).strip(' \n\r\t')
    if apply_str:
        value = str(value)
    return value


def _get_optional_node_value(parent, tagname, default=None):
    node = _get_optional_node(parent, tagname)
    if node is None:
        return default
    return _get_node_value(node)


def _get_node_attr(node, attr, default=False):
    """
    :param default: False means value is required
    """
    if node.hasAttribute(attr):
        return str(node.getAttribute(attr))
    if default is False:
        raise InvalidPackage('The "%s" tag must have the attribute "%s"' % (node.tagName, attr))
    return default


def _get_dependencies(parent, tagname):
    depends = []
    for node in _get_nodes(parent, tagname):
        depend = Dependency(_get_node_value(node))
        for attr in ['version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']:
            setattr(depend, attr, _get_node_attr(node, attr, None))
        depends.append(depend)
    return depends






d = parse_package_for_distutils()
d['packages'] = ['ecto', 'ecto.sphinx', 'ecto.sphinx.breathe',
                'ecto.sphinx.breathe.finder', 'ecto.sphinx.breathe.finder.doxygen',
                'ecto.sphinx.breathe.parser', 'ecto.sphinx.breathe.parser.doxygen',
                'ecto.sphinx.breathe.renderer', 'ecto.sphinx.breathe.renderer.rst', 'ecto.sphinx.breathe.renderer.rst.doxygen',
                'ecto_catkin'
               ]
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
