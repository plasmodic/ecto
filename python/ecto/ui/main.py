#!/usr/bin/python

import sys, inspect
from PySide import QtCore, QtGui

from uic import Ui_MainWindow
import ecto

class MyMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(MyMainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

    def init(self, plasm):
        self.ui.plasmView.add(plasm)



def main():
    app = QtGui.QApplication(sys.argv)

    from ecto.test import makeplasm
    
    plasm = makeplasm.gen_mult_mult()

    #     for modname in sys.argv[1:]:
    #         print modname
    #         m = __import__(modname)
    #         gooduns += [cls for name,cls in inspect.getmembers(m)
    #                        if inspect.isclass(cls) 
    #                    and issubclass(cls, ecto.module)]
    #     print gooduns

    myapp = MyMainWindow()
    myapp.show()
    myapp.init(plasm)
    r = app.exec_()
    sys.exit(r)
    
