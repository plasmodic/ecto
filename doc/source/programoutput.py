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


"""
    sphinxcontrib.programoutput
    ===========================

    This extension provides a directive to include the output of commands as
    literal block while building the docs.

    .. moduleauthor::  Sebastian Wiesner  <lunaryorn@googlemail.com>
"""

__version__ = '0.4.1'

import sys
import shlex
from subprocess import Popen, CalledProcessError, PIPE, STDOUT
from collections import defaultdict

from docutils import nodes
from docutils.parsers import rst
from docutils.parsers.rst.directives import flag, unchanged


class program_output(nodes.Element):
    pass


def _slice(value):
    parts = [int(v.strip()) for v in value.split(',')]
    if len(parts) > 2:
        raise ValueError('too many slice parts')
    return tuple((parts + [None]*2)[:2])


class ProgramOutputDirective(rst.Directive):
    has_content = False
    final_argument_whitespace = True
    required_arguments = 2

    option_spec = dict(shell=flag, prompt=flag, nostderr=flag,
                       ellipsis=_slice, extraargs=unchanged)

    def run(self):
        node = program_output()
        node['modname'] = self.arguments[0]
        node['celltype'] = self.arguments[1]

        return [node]


def xtract(mod):
    def gettendril(tendrils):
        d = {}

        for k, v in tendrils:
            d[k] = dict(doc=v.doc,
                        type_name = v.type_name,
                        required = v.required)
            if v.has_default:
                d[k]['default'] = v.val
        return d

    d = {}
    inst = mod.inspect((),{})

    d['name'] = mod.__name__
    d['short_doc'] = mod.short_doc
    d['params'] = gettendril(inst.params)
    d['inputs'] = gettendril(inst.inputs)
    d['outputs'] = gettendril(inst.outputs)

    # d['spect'] = inst.

    return d

def run_programs(app, doctree):

    for node in doctree.traverse(program_output):
        celltype = node['celltype']
        modname = node['modname']
        m = __import__(modname)
        d = xtract(m.__dict__[celltype])
        output = str(d)

        print "OUTPUT:", output

        new_node = nodes.paragraph(output, output)
        new_node['language'] = 'text'
        node.replace_self(new_node)


def setup(app):
    app.add_config_value('programoutput_use_ansi', False, 'env')
    app.add_config_value('programoutput_prompt_template',
                         '$ %(command)s\n%(output)s', 'env')
    app.add_directive('program-output', ProgramOutputDirective)
    app.add_directive('ectocell', ProgramOutputDirective)
    # app.connect('builder-inited', init_cache)
    app.connect('doctree-read', run_programs)
