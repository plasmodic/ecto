#!/usr/bin/env python
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
            
def widgetizeParameter_Numbers(param, tags, edit, layout):
    if ecto.Tags.Min in tags and ecto.Tags.Max in tags:
        slider = QSlider(Qt.Horizontal)
        min = tags[ecto.Tags.Min]
        max = tags[ecto.Tags.Max]
        scalar = 1000.0 / (max - min)
        slider.setMinimum(min * scalar)
        slider.setMaximum(max * scalar)
        edit.setValidator(QDoubleValidator(min, max, 10, None))
        slider.setTickInterval(1)
        slider.valueChanged.connect(TextSetter(edit, scalar).onChange)
        edit.textChanged.connect(SlideSetter(slider, scalar).onChange)
        layout.addWidget(slider)
    elif ecto.Tags.Min in tags:
        min = tags[ecto.Tags.Min]
        edit.setValidator(QDoubleValidator(min, decimal.Infinity, 10, None))
    elif ecto.Tags.Max in tags:
        max = tags[ecto.Tags.Max]
        edit.setValidator(QDoubleValidator(-decimal.Infinity, max, 10, None))
        
def populateParameterKnobs(cell):
    for p in cell.params:
        name = p.key()
        param = p.data()
        param_layouts = []
        tags = param.tags()
        if ecto.Tags.Dynamic in tags and tags[ecto.Tags.Dynamic]:
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
                widgetizeParameter_Numbers(param,tags, edit, v)
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

