
import sys, inspect, math
from PySide import QtCore, QtGui

import ecto
from plasmnode import PlasmNode

class PlasmWidget(QtGui.QGraphicsView):
    def __init__(self, x):
        QtGui.QGraphicsView.__init__(self, x)
        scene = QtGui.QGraphicsScene(self)
        scene.setSceneRect(0, 0, 400, 400)
        self.setScene(scene)
        self.setCacheMode(QtGui.QGraphicsView.CacheBackground)
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setTransformationAnchor(QtGui.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtGui.QGraphicsView.AnchorViewCenter)
        
        self.scale(1, 1)
        self.setMinimumSize(400, 400)
        self.setWindowTitle(self.tr("Plasm"))
        self.coo = 50;

    def add(self, plasm):
        print "plasm!", plasm
        
        #node = PlasmNode(self, what)
        #self.scene().addItem(node)
        #node.setPos(0, self.coo)
        #self.coo -= node.boundingRect().y()

        for e in plasm.edges:
            edge = e.data()
            pnode = PlasmNode(self, e.key())
            print ">>>", edge, pnode
            self.scene().addItem(pnode)
            pnode.setPos(self.coo, 50)
            self.coo += pnode.boundingRect().x() - 50
        
    def scaleView(self, scaleFactor):
        factor = self.matrix().scale(scaleFactor, scaleFactor).mapRect(QtCore.QRectF(0, 0, 1, 1)).width()

        if factor < 0.07 or factor > 100:
            return

        self.scale(scaleFactor, scaleFactor)

    def keyPressEvent(self, event):
        key = event.key()

        if key == QtCore.Qt.Key_Escape or key == QtCore.Qt.Key_Q:
            sys.exit(0)
        if key == QtCore.Qt.Key_Up:
            self.centerNode.moveBy(0, -20)
        elif key == QtCore.Qt.Key_Down:
            self.centerNode.moveBy(0, 20)
        elif key == QtCore.Qt.Key_Left:
            self.centerNode.moveBy(-20, 0)
        elif key == QtCore.Qt.Key_Right:
            self.centerNode.moveBy(20, 0)
        elif key == QtCore.Qt.Key_Plus:
            self.scaleView(1.2)
        elif key == QtCore.Qt.Key_Minus:
            self.scaleView(1 / 1.2)
        elif key == QtCore.Qt.Key_Space or key == QtCore.Qt.Key_Enter:
            for item in self.scene().items():
                if isinstance(item, Node):
                    item.setPos(-150 + QtCore.qrand() % 300, -150 + QtCore.qrand() % 300)
        else:
            QtGui.QGraphicsView.keyPressEvent(self, event)


    def wheelEvent(self, event):
        self.scaleView(math.pow(2.0, -event.delta() / 240.0))

