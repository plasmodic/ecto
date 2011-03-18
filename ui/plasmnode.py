import sys
import weakref
import math
from PySide import QtCore, QtGui

class PlasmNode(QtGui.QGraphicsItem):
    Type = QtGui.QGraphicsItem.UserType + 1

    def __init__(self, graphWidget, inst):
        QtGui.QGraphicsItem.__init__(self)
        # self.graph = weakref.ref(graphWidget)
        # self.edgeList = []
        # self.newPos = QtCore.QPointF()
        # self.setFlag(QtGui.QGraphicsItem.ItemIsMovable)
        # self.setFlag(QtGui.QGraphicsItem.ItemSendsGeometryChanges)
        # self.setCacheMode(self.DeviceCoordinateCache)
        self.setZValue(-1)
        self.inst = inst

    def type(self):
        return PlasmNode.Type

    def shape(self):
        path = QtGui.QPainterPath()
        path.addRect(-40, -40, 45, 45)
        return path

    def boundingRect(self):
        adjust = 5.0
        return QtCore.QRectF(-40 - adjust, -40 - adjust,
                             40 + adjust, 40 + adjust)

    def paint(self, painter, option, widget):
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtCore.Qt.darkGray)
        painter.drawRect(-15, -15, 40, 40)

        gradient = QtGui.QRadialGradient(-3, -3, 30)
        if option.state & QtGui.QStyle.State_Sunken:
            gradient.setCenter(3, 3)
            gradient.setFocalPoint(3, 3)
            gradient.setColorAt(1, QtGui.QColor(QtCore.Qt.yellow).lighter(120))
            gradient.setColorAt(0, QtGui.QColor(QtCore.Qt.darkYellow).lighter(120))
        else:
            gradient.setColorAt(0, QtCore.Qt.yellow)
            gradient.setColorAt(1, QtCore.Qt.darkYellow)
 
        painter.setBrush(QtGui.QBrush(gradient))
        painter.setPen(QtGui.QPen(QtCore.Qt.black, 0))
        painter.drawRect(-20, -20, 40, 40)

