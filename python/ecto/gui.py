import ecto
import sys

HAS_GUI=False

try:
    from PySide.QtCore import QTimer
    from PySide.QtGui import QWidget, QDialog, QPushButton, QApplication, \
     QVBoxLayout, QLabel, QHBoxLayout, QLineEdit, QScrollArea, QMainWindow, QSizePolicy
    HAS_GUI=True
    class TendrilWidget(QWidget):
        def __init__(self, name, tendril, parent=None):
            super(TendrilWidget,self).__init__(parent)
            hlayout = QHBoxLayout()
            hlayout.addWidget(QLabel(name))
            edit = QLineEdit(str(tendril.val))
            hlayout.addWidget(edit)

            self.thunker = TendrilThunker(tendril)
            edit.textChanged.connect(self.thunker.update)

            self.setLayout(hlayout)
            self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)

    class DynamicReconfigureForm(QWidget):
        def __init__(self, plasm, parent=None):
            super(DynamicReconfigureForm, self).__init__(parent)
            self.plasm = plasm
            self.setWindowTitle("Dynamic Reconfigure")
            plasm.configure_all()
            self.generate_dialogs()
            self.sched = ecto.Scheduler(plasm)

        def exec_one(self):
            if not self.sched.running():
                self.sched.execute_async()
            try:
                #give ecto a slice of time
                rval = self.sched.run(1000) #exec for 1000 microseconds
                if not rval: #quit condition
                    sys.exit(1)
            except KeyboardInterrupt as e: #ctrl-c
                sys.exit(1)

        def generate_dialogs(self):
            vlayout = QVBoxLayout()
            commit_button = QPushButton("&Commit")
            w_all = QWidget()
            v = QVBoxLayout()
            for cell in self.plasm.cells():
                w = QWidget()
                cvlayout = QVBoxLayout()
                cvlayout.addWidget(QLabel(cell.name()))
                for name, tendril in cell.params:
                    tw = TendrilWidget(name, tendril)
                    cvlayout.addWidget(tw)
                    commit_button.clicked.connect(tw.thunker.commit)
                w.setLayout(cvlayout)
#                w.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
                v.addWidget(w)
            w_all.setLayout(v)
            s = QScrollArea()
            s.setWidget(w_all)
            vlayout.addWidget(s)
            vlayout.addWidget(commit_button)
            self.setLayout(vlayout)


except ImportError as e:
    print e
    pass


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

def gui_execute(plasm):
    if HAS_GUI:
        # Create a Qt application
        app = QApplication(sys.argv)
        # Create and show the form
        form = DynamicReconfigureForm(plasm)
        timer = QTimer(form)
        timer.timeout.connect(form.exec_one)
        timer.start()
        form.show()
        # Run the main Qt loop
        return app.exec_()

    else:
        print >>sys.stderr, "Could not import PySide. Please install python-pyside."
        sys.exit(1)
