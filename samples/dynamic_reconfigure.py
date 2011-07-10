#!/usr/bin/env python
import ecto
import ecto_test
import time
# Import PySide classes
import sys
from PySide.QtCore import *
from PySide.QtGui import *
import decimal

class TextSetter():
    def __init__(self,text,scalar):
        self.text = text
        self.scalar = scalar
    def onChange(self, val):
        x = float(val)/self.scalar
        self.text.setText(str(x))

class SlideSetter():
    def __init__(self,slide,scalar):
        self.slide = slide
        self.scalar = scalar
    def onChange(self, val):
        nval =int(float(val)*self.scalar)
        if(nval !=self.slide.value()):
            self.slide.setValue(nval)

class TendrilPoster():
    def __init__(self,tendril):
        self.tendril = tendril
    def onChange(self, val):
        #use the type info from the tendril to
        #cast the incoming value to the correct
        #explicit type.
        t = type(self.tendril.val)
        self.tendril.set(t(val))

class Form(QDialog):
     
    def __init__(self,plasm,parent=None):
        super(Form, self).__init__(parent)
        self.plasm = plasm
        self.setWindowTitle("Dynamic Reconfigure")
        plasm.configure_all()
        self.generate_dialogs()
        
    def generate_dialogs(self):
        self.edits = []
        vlayout = QVBoxLayout()
        for x in self.plasm.cells():
            for p in x.params:
                param_layouts = []
                tags = p.data().tags()
                if ecto.Tags.Dynamic in tags and tags[ecto.Tags.Dynamic]:
                    name = p.key()
                    param = p.data()
                    label = QLabel(name)
                    edit = QLineEdit(str(param.val))
                    edit.textChanged.connect(TendrilPoster(param).onChange)
                    v = QVBoxLayout()
                    hlayout = QHBoxLayout()
                    hlayout.addWidget(label)
                    hlayout.addWidget(edit)
                    v.addLayout(hlayout)
                    if type(param.val) in (float,int):
                        if ecto.Tags.Min in tags and ecto.Tags.Max in tags:
                            slider = QSlider(Qt.Horizontal)
                            min = tags[ecto.Tags.Min]
                            max = tags[ecto.Tags.Max]
                            scalar = 1000.0/(max - min) 
                            slider.setMinimum(min * scalar)
                            slider.setMaximum(max * scalar)
                            edit.setValidator(QDoubleValidator(min,max,10,None))
                            slider.setTickInterval(1)
                            slider.valueChanged.connect(TextSetter(edit,scalar).onChange)
                            edit.editingFinished.connect(SlideSetter(slider,scalar).onChange)
                            v.addWidget(slider)
                        elif ecto.Tags.Min in tags:
                            min = tags[ecto.Tags.Min]
                            edit.setValidator(QDoubleValidator(min,decimal.Infinity,10,None))
                        elif ecto.Tags.Max in tags:
                            max = tags[ecto.Tags.Max]
                            edit.setValidator(QDoubleValidator(-decimal.Infinity,max,10,None))
                    param_layouts.append(v)
                    self.edits.append((edit,x,name))
                if len(param_layouts) > 0:
                    vlayout.addWidget(QLabel(x.name()))
                    vlayout.addWidget(QLabel("Parameters"))
                    for x in param_layouts:
                        vlayout.addLayout(x)
        self.setLayout(vlayout)

def test_parameter_callbacks():
    generate = ecto_test.Generate()
    param_watcher = ecto_test.ParameterWatcher(value=2)
    sleep = ecto_test.Sleep()
    printer = ecto_test.Printer()
    plasm = ecto.Plasm()
    plasm.connect(generate["out"] >> param_watcher["input"],
                  param_watcher['output']>>printer[:]
                  )
    plasm.insert(sleep)
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute_async()
     # Create a Qt application 
    app = QApplication(sys.argv)
    # Create and show the form
    form = Form(plasm)
    form.show()
    #timer = QTimer(app)
    #timer.timeout.connect(form.update_vals)
    #timer.start()
    # Run the main Qt loop
    sys.exit(app.exec_())

if __name__ == '__main__':
    test_parameter_callbacks()



