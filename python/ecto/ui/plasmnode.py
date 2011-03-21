import sys
import weakref
import math
from PySide import QtCore, QtGui

class PlasmNode(QtGui.QGraphicsItem):
    Type = QtGui.QGraphicsItem.UserType + 1

    def __init__(self, graphWidget, inst):
        QtGui.QGraphicsItem.__init__(self)
        self.xsize = 80
        self.ysize = 80
        self.shadowoffset = 3
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
        path.addRect(0, 0, 
                      self.xsize+self.shadowoffset, self.ysize+self.shadowoffset)
        return path

    def boundingRect(self):
        return QtCore.QRectF(-self.xsize - self.shadowoffset, -self.ysize - self.shadowoffset,
                              self.xsize + self.shadowoffset, self.ysize + self.shadowoffset)

    def drawports(self, painter, portlist, xcoord):
        painter.setBrush(QtCore.Qt.green)
        painter.setPen(QtCore.Qt.black)
        nport = len(portlist)
        npix = (self.ysize * 1.0)/ (nport+1)
        print "nport=", nport, "npix=", npix
        j = 0
        for i in portlist:
            j += 1
            n = i.key()
            e = i.data()
            print n, e
            painter.drawRect(xcoord, j*npix, 5, 5)
        

    def paint(self, painter, option, widget):
        print "paint", self.inst.Name()
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtCore.Qt.darkGray)
        painter.drawRect(self.shadowoffset, self.shadowoffset,
                         self.xsize + self.shadowoffset, 
                         self.ysize + self.shadowoffset)

        gradient = QtGui.QLinearGradient(0, 0, self.xsize, self.ysize)

        if option.state & QtGui.QStyle.State_Sunken:
            gradient.setColorAt(1, QtGui.QColor(QtCore.Qt.yellow).lighter(100))
            gradient.setColorAt(0, QtGui.QColor(QtCore.Qt.darkYellow).lighter(100))
        else:
            gradient.setColorAt(0, QtCore.Qt.yellow)
            gradient.setColorAt(1, QtCore.Qt.darkYellow)
            
        painter.setBrush(QtGui.QBrush(gradient))
        painter.setPen(QtGui.QPen(QtCore.Qt.black, 0))
        painter.drawRect(0, 0, self.xsize, self.ysize)
        painter.drawText(15,15, self.inst.Name())

        self.drawports(painter, self.inst.inputs, 0)
        self.drawports(painter, self.inst.outputs, self.xsize-5)
