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
        self.edit = QLineEdit("Write my name here")
        self.button = QPushButton("Show Greetings")
        # Create layout and add widgets
        layout = QVBoxLayout()
        layout.addWidget(self.edit)
        layout.addWidget(self.button)
        # Set dialog layout
        self.setLayout(layout)
        # Add button signal to greetings slot
        self.button.clicked.connect(self.greetings)
        # Greets the user
        self.generate_dialogs()
    def generate_dialogs(self):
        for x in self.plasm.cells():
            print x.name(),x.doc()
    def greetings(self):
        print ("Hello %s" % self.edit.text())
        
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
    # Run the main Qt loop
    sys.exit(app.exec_())

if __name__ == '__main__':
    test_parameter_callbacks()



