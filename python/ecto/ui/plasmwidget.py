
import sys, inspect
from PySide import QtCore, QtGui

import ecto
from plasmnode import PlasmNode

class PlasmWidget(QtGui.QGraphicsView):
    def __init__(self, x):
        QtGui.QGraphicsView.__init__(self, x)
        scene = QtGui.QGraphicsScene(self)
        scene.setSceneRect(-200, -200, 400, 400)
        self.setScene(scene)
        self.setCacheMode(QtGui.QGraphicsView.CacheBackground)
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setTransformationAnchor(QtGui.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtGui.QGraphicsView.AnchorViewCenter)
        
        self.scale(1.5, 1.5)
        self.setMinimumSize(400, 400)
        self.setWindowTitle(self.tr("Plasm"))
        self.coo = -50;

    def add(self, what):
        node = PlasmNode(self, what)
        self.scene().addItem(node)
        node.setPos(0, self.coo)
        self.coo -= node.boundingRect().y()
        
