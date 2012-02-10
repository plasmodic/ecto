#!/usr/bin/env python
import ecto
import ecto_test
import time
import sys
# Import PySide classes
from PySide.QtCore import *
from PySide.QtGui import *

class TendrilThunker(object):

    def __init__(self, tendril):
        self.tendril = tendril
        self.val = None

    def update(self, val):
        self.val = val

    def commit(self):
        if self.val:
            x = type(self.tendril.val)(self.val)
            self.tendril.set(x)
            self.val = None

class DynamicReconfigureForm(QDialog):
    def __init__(self, plasm, parent=None):
        super(DynamicReconfigureForm, self).__init__(parent)
        self.plasm = plasm
        self.thunkers=[]
        self.setWindowTitle("Dynamic Reconfigure")
        plasm.configure_all()
        self.generate_dialogs()
    
    def commit(self):
        print "wooot woot"
        for thunker in self.thunkers:
            thunker.commit()

    def generate_dialogs(self):
        vlayout = QVBoxLayout()
        for cell in self.plasm.cells():
            vlayout.addWidget(QLabel(cell.name()))
            for name, tendril in cell.params:
                hlayout = QHBoxLayout()
                hlayout.addWidget(QLabel(name))
                vlayout.addLayout(hlayout)
                edit = QLineEdit(str(tendril.val))
                print type(tendril.val)
                hlayout.addWidget(edit)
                thunker = TendrilThunker(tendril)
                edit.textChanged.connect(thunker.update)

                self.thunkers.append(thunker)#need to hold onto the thunker
        commit_button = QPushButton("&Commit")
        commit_button.clicked.connect(self.commit)
        vlayout.addWidget(commit_button)
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


