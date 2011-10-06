Installing to a virtualenv
==========================

.. highlight:: ectosh

Ecto kitchens may be installed using a virtualenv. The default location of the
virtualenv is ``/opt/ecto/PROJECT_NAME/GITTAG_SHORT``.

First you should create the virtualenv for the ecto kitchen::

  % sudo make virtualenv
  New python executable in /opt/ecto/recognition_kitchen/or-alpha_0/bin/python2.7
  Not overwriting existing python script /opt/ecto/recognition_kitchen/or-alpha_0/bin/python (you must use /opt/ecto/recognition_kitchen/or-alpha_0/bin/python2.7)
  Installing setuptools.............done.
  Installing pip...............done.
  Built target virtualenv

Make will install to this directory::

  % sudo make install
  ....
  -- Set runtime path of "/opt/ecto/recognition_kitchen/or-alpha_0/lib/python2.7/dist-packages/ecto_object_recognition/tod_detection.so" to "/opt/ecto/recognition_kitchen/or-alpha_0/lib"
  -- Installing: /opt/ecto/recognition_kitchen/or-alpha_0/lib/python2.7/dist-packages/ecto_object_recognition/tod_training.so
  -- Set runtime path of "/opt/ecto/recognition_kitchen/or-alpha_0/lib/python2.7/dist-packages/ecto_object_recognition/tod_training.so" to "/opt/ecto/recognition_kitchen/or-alpha_0/lib"
  -- Installing: /opt/ecto/recognition_kitchen/or-alpha_0/lib/python2.7/dist-packages/ecto_object_recognition/reconstruction.so
  -- Set runtime path of "/opt/ecto/recognition_kitchen/or-alpha_0/lib/python2.7/dist-packages/ecto_object_recognition/reconstruction.so" to "/opt/ecto/recognition_kitchen/or-alpha_0/lib"

If you would like to change the virtualenv path then please set the cmake cache
variable ``VIRTUALENV_DIR``::

  % cmake .. -DVIRTUALENV_DIR=~/myenv


Now you may activate the environment with::

  % . /opt/ecto/recognition_kitchen/or-alpha_0/bin/activate
  % python
  >>> ecto.version()
  'ecto amoeba-beta3'
  >>> ecto.__file__
  '/opt/ecto/or-alpha_0/local/lib/python2.7/dist-packages/ecto.so'
