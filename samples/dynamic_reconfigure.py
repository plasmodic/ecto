#!/usr/bin/env python
import ecto
import ecto_test
import time
# Import PySide classes
import sys
from PySide.QtCore import *
from PySide.QtGui import *

class Form(QDialog):
     
    def __init__(self,plasm,parent=None):
        super(Form, self).__init__(parent)
        self.plasm = plasm
        self.setWindowTitle("Dynamic Reconfigure")
        # Create widgets
        self.button = QPushButton("Commit.")
        # Set dialog layout
        # Add button signal to greetings slot
        self.button.clicked.connect(self.commit)
        plasm.configure_all()
        # Greets the user
        self.generate_dialogs()
    def generate_dialogs(self):
        self.edits = []
        vlayout = QVBoxLayout()
        for x in self.plasm.cells():
            for p in x.params:
                param_layouts = []
                dynamic = p.data().tagged(ecto.Tags.Dynamic)
                if dynamic:
                    name = p.key()
                    param = p.data()
                    label = QLabel(name)
                    edit = QLineEdit(str(param.val))
                    hlayout = QHBoxLayout()
                    hlayout.addWidget(label)
                    hlayout.addWidget(edit)
                    param_layouts.append(hlayout)
                    self.edits.append((edit,x,name))
                if len(param_layouts) > 0:
                    vlayout.addWidget(QLabel(x.name()))
                    vlayout.addWidget(QLabel("Parameters"))
                    for x in param_layouts:
                        vlayout.addLayout(x)
        vlayout.addWidget(self.button)
        self.setLayout(vlayout)
        
    def commit(self):
        for edit,cell,param_name in self.edits:
            t = type(cell.params[param_name])
            cell.params.__setattr__(param_name,t(edit.text()))
    def update_vals(self):
        pass

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



