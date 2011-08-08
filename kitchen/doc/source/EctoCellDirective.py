# -*- coding: utf-8 -*-
# Copyright (c) 2010, 2011, Sebastian Wiesner <lunaryorn@googlemail.com>
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__version__ = '0.4.1'

import sys
import shlex
import os
from os import path

from docutils import nodes
from docutils.parsers import rst
from docutils.parsers.rst.directives import flag, unchanged

from sphinx import addnodes
from sphinx.util.osutil import ensuredir
from sphinx.ext.graphviz import graphviz as graphviz_node
import ecto

class ectocell(nodes.Element):
    pass


def _slice(value):
    parts = [int(v.strip()) for v in value.split(',')]
    if len(parts) > 2:
        raise ValueError('too many slice parts')
    return tuple((parts + [None]*2)[:2])

def make_ectocelldirective(modname, celltype, state):
    node = ectocell()
    node.modname = modname
    node.celltype = celltype
    env = state.document.settings.env
    targetid = 'index-%s' % env.new_serialno('index')
    targetnode = nodes.target('', '', ids=[targetid, node.modname + "_" + node.celltype])
    state.document.note_explicit_target(targetnode)
    indexnode = addnodes.index()
    indexnode['entries'] = ne = []
    indexnode['inline'] = False
    s = 'Cell; ' + celltype + ' (module ' + modname + ')'
    ne.append(('single', s, targetid, s))

    modfirstindexarg = celltype + ' (ecto cell in module ' + modname + ')'
    ne.append(('single', modfirstindexarg, 
               targetid, modfirstindexarg))
    
    return [node, indexnode]


class EctoCellDirective(rst.Directive):
    has_content = False
    final_argument_whitespace = True
    required_arguments = 2

    option_spec = dict(shell=flag, prompt=flag, nostderr=flag,
                       ellipsis=_slice, extraargs=unchanged)

    def run(self):

        return make_ectocelldirective(self.arguments[0], self.arguments[1], self.state)

def docize(CellType):
    d = {}
    inst = CellType.inspect((),{})

    def gettendril(name, tendrils, isparam):
        d = {}

        if len(tendrils) == 0:
            return nodes.paragraph()

        section = nodes.title(text=name)
        lst = nodes.bullet_list()
        section += lst

        for k, v in tendrils:
            entry = nodes.list_item()
            lst += entry
            para = nodes.paragraph()
            entry += para
            d[k] = dict(doc=v.doc,
                        type_name = v.type_name,
                        required = v.required)
            if v.has_default:
                try:
                    default = str(v.val)
                except TypeError, e:
                    default = '[not visible from python]'

            para += [nodes.strong(k, k), nodes.literal('', '   '), 
                     nodes.emphasis('', '   type: '), nodes.literal('', v.type_name + " ")]
            para += nodes.literal('', '   ')
            if isparam:
                if not v.required:
                    para += nodes.emphasis('', ' not ')
                para += nodes.emphasis('', 'required')
                para += nodes.literal('', '   ')
                if v.has_default:
                    para += [nodes.emphasis('', " default: "), nodes.literal('', default)]
                else:
                    para += nodes.emphasis('', ' no default value')

            entry += nodes.paragraph('', v.doc)

        return section

    d['name'] = CellType.__name__
    d['short_doc'] = CellType.short_doc

    cell = nodes.topic(text=CellType.__name__)
    cell['ids'].append(CellType.__name__)
    cell['names'].append(CellType.__name__)
    cell += nodes.title(text=CellType.__name__, rawtest=CellType.__name__)
    para = nodes.paragraph(text=CellType.short_doc)
    cell += para
    #params = nodes.rubric(text='parameters')
    # params += nodes.subtitle(text="parameters")
    cell += gettendril('Parameters', inst.params, True)
    cell += gettendril('Inputs', inst.inputs, False)
    cell += gettendril('Outputs', inst.outputs, False)

    return cell


def do_ectocell(app, doctree):

    for node in doctree.traverse(ectocell):
        c = __import__(node.modname, fromlist=[str(node.celltype)])
        if node.celltype not in c.__dict__:
            raise RuntimeError("Cell %s not found in module %s" % (node.celltype, str(c)))

        new_node = docize(c.__dict__[node.celltype])
        node.replace_self(new_node)


def setup(app):
    app.add_directive('ectocell', EctoCellDirective)
    app.connect('doctree-read', do_ectocell)
