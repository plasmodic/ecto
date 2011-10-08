#!/usr/bin/env python
# 
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
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
# 
import ecto
import ecto_test
import time
import sys
# Import PySide classes
from PySide.QtCore import *
from PySide.QtGui import *
import decimal

class TextSetter():
    def __init__(self, text, scalar):
        self.text = text
        self.scalar = scalar
    def onChange(self, val = None):
        if not val:
            print "wtf TextSetter"
        x = float(val) / self.scalar
        self.text.setText(str(x))

class SlideSetter():
    def __init__(self, slide, scalar):
        self.slide = slide
        self.scalar = scalar
    def onChange(self, val = None):
        if not val:
            print "wtf SlideSetter"
        nval = int(float(val) * self.scalar)
        if(nval != self.slide.value()):
            self.slide.setValue(nval)

class TendrilPoster():
    def __init__(self, tendril):
        self.tendril = tendril
    def onChange(self, val = None):
        if not val:
            print "wtf TendrilPoster"
        #use the type info from the tendril to
        #cast the incoming value to the correct
        #explicit type.
        t = type(self.tendril.val)
        try:
            val = t(val)
            self.tendril.set(val)
        except ValueError, e:
            print e
            
def widgetizeParameter_Numbers(param, edit, layout):
    if type(param.val) in (int,):
        edit.setValidator(QIntegerValidator(None))
    else:
        edit.setValidator(QDoubleValidator(None))
        
def populateParameterKnobs(cell):
    for p in cell.params:
        name = p.key()
        param = p.data()
        param_layouts = []
        label = QLabel(name)
        edit = QLineEdit(str(param.val))
        #register an change callback to the tendril.
        #lil bit of layout
        v = QVBoxLayout()
        h = QHBoxLayout()
        h.addWidget(label)
        h.addWidget(edit)#all tendrils get an edit box?
        v.addLayout(h)
        if type(param.val) in (float, int):
            widgetizeParameter_Numbers(param,edit, v)
        edit.textChanged.connect(TendrilPoster(param).onChange)
        param_layouts.append(v)
    return param_layouts
    
class DynamicReconfigureForm(QDialog):
    def __init__(self, plasm, parent=None):
        super(DynamicReconfigureForm, self).__init__(parent)
        self.plasm = plasm
        self.setWindowTitle("Dynamic Reconfigure")
        plasm.configure_all()
        self.generate_dialogs()

    def generate_dialogs(self):
        self.edits = []
        vlayout = QVBoxLayout()
        for x in self.plasm.cells():
            param_layouts = populateParameterKnobs(x)
            if len(param_layouts) > 0:
                vlayout.addWidget(QLabel(x.name()))
                vlayout.addWidget(QLabel("Parameters"))
                for x in param_layouts:
                    vlayout.addLayout(x)
        self.setLayout(vlayout)

def dynamic_reconfigure_execute(plasm):
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute_async()
     # Create a Qt application 
    app = QApplication(sys.argv)
    # Create and show the form
    form = DynamicReconfigureForm(plasm)
    form.show()
    # Run the main Qt loop
    return app.exec_()

def test_parameter_callbacks():
    generate = ecto_test.Generate()
    param_watcher = ecto_test.ParameterWatcher(value=2)
    sleep = ecto_test.Sleep()
    printer = ecto_test.Printer()
    plasm = ecto.Plasm()
    plasm.connect(generate["out"] >> param_watcher["input"],
                  param_watcher['output'] >> printer[:]
                  )
    plasm.insert(sleep)
    return dynamic_reconfigure_execute(plasm)

if __name__ == '__main__':
    sys.exit(test_parameter_callbacks())


