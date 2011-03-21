
import sys, inspect
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
            self.scene().addItem(pnode)
            pnode.setPos(self.coo, 50)
            self.coo += pnode.boundingRect().x() - 50

