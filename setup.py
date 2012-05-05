#!/usr/bin/env python
from distutils.core import setup

setup(name='ecto',
      version='1.0.0',
      description='Ecto is a hybrid C++/Python development framework for constructing and maintaining pipelines.',
      packages=['ecto', 'ecto.sphinx', 'ecto.sphinx.breathe',
                'ecto.sphinx.breathe.finder', 'ecto.sphinx.breathe.finder.doxygen',
                'ecto.sphinx.breathe.parser', 'ecto.sphinx.breathe.parser.doxygen',
                'ecto.sphinx.breathe.renderer', 'ecto.sphinx.breathe.renderer.rst', 'ecto.sphinx.breathe.renderer.rst.doxygen',
               ],
      package_dir={'':'python'}
)
