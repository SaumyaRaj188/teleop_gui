from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from enum import Enum

class QJoystickAxes(Enum):
    Horizontal = 1
    Vertical = 2
    Complete = 3
    UpperHalf = 4
    LowerHalf = 5

class QJoystick(QWidget):
    def __init__(self, axes, inputHandler):
        super().__init__()

        self.axes = axes
        self.inputHandler = inputHandler
        self.setKeys(None, None, None, None)
        
        self.__stickPos = None
        self.__sticky = False
        self.__hasMouse = False
        self.__innerToOuterRatio = 0.4

        self.setMinimumSize(150, 150)
        self.setContentsMargins(5, 5, 5, 5)
        self.setFocusPolicy(Qt.StrongFocus)

    def setKeys(self, up, down, left, right):
        self.up_key = up
        self.down_key = down
        self.left_key = left
        self.right_key = right

    @property
    def value(self):
        vec = self.__stickPos - self.getDrawingCenter()
        dirn = vec.normalized()
        mag = vec.length()
        vec = dirn * mag / (self.getOuterDia() / 2)
        return QVector2D(vec.x(), -vec.y())
    
    @value.setter
    def value(self, vec):
        vec = QVector2D(vec.x(), -vec.y())
        dirn = vec.normalized()
        mag = vec.length() * (self.getOuterDia() / 2)
        self.__stickPos = self.clampToAxes(dirn * mag + self.getDrawingCenter())
        self.repaint()

    def getDrawingCenter(self):
        return QVector2D(self.rect().marginsRemoved(self.contentsMargins()).center())

    def getOuterDia(self):
        rect = self.rect().marginsRemoved(self.contentsMargins())
        outer = min(rect.width(), rect.height()) / (1 + self.__innerToOuterRatio)
        return outer
    
    def getInnerDia(self):
        return self.getOuterDia() * self.__innerToOuterRatio

    def isInsideCircle(self, vec):
        return (vec - self.getDrawingCenter()).lengthSquared() < (self.getOuterDia() / 2) ** 2
    
    def clampToAxes(self, vec):
        def clamp(x, a, b):
            return min(max(x, a), b)
        
        outerDia = self.getOuterDia()
        center = self.getDrawingCenter()
        vec -= center
        
        if self.axes == QJoystickAxes.Horizontal:
            x = clamp(vec.x(), -outerDia / 2, outerDia / 2) + center.x()
            y = center.y()
            return QVector2D(x, y)
        
        elif self.axes == QJoystickAxes.Vertical:
            x = center.x()
            y = clamp(vec.y(), -outerDia / 2, outerDia / 2) + center.y()
            return QVector2D(x, y)
        
        elif self.axes == QJoystickAxes.Complete:
            mag = vec.length()
            vec = (vec / mag) * min(mag, outerDia / 2) if mag != 0 else vec
            vec += center
            return vec
        
        elif self.axes == QJoystickAxes.UpperHalf:
            mag = vec.length()
            vec = (vec / mag) * min(mag, outerDia / 2) if mag != 0 else vec
            vec += center
            vec.setY(min(vec.y(), center.y()))
            return vec
        
        elif self.axes == QJoystickAxes.LowerHalf:
            mag = vec.length()
            vec = (vec / mag) * min(mag, outerDia / 2) if mag != 0 else vec
            vec += center
            vec.setY(max(vec.y(), center.y()))
            return vec

    def mousePressEvent(self, ev: QMouseEvent):
        if self.isInsideCircle(QVector2D(ev.localPos())):
            self.__hasMouse = True
            self.__stickPos = self.clampToAxes(QVector2D(ev.localPos()))
            self.repaint()
        return super().mousePressEvent(ev)
    
    def mouseMoveEvent(self, ev: QMouseEvent):
        if self.__hasMouse:
            self.__stickPos = self.clampToAxes(QVector2D(ev.localPos()))
            self.repaint()
        return super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev: QMouseEvent):
        self.__hasMouse = False
        self.__sticky = (ev.button() == Qt.MouseButton.MiddleButton) or (self.inputHandler[Qt.Key.Key_Shift])
        self.repaint()
        return super().mousePressEvent(ev)

    def update(self, speed, dt):
        controlled = self.__hasMouse

        if not self.__hasMouse:
            if self.up_key and self.inputHandler[self.up_key]:
                self.value += QVector2D(0, speed * dt)
                controlled = True
            elif self.down_key and self.inputHandler[self.down_key]:
                self.value -= QVector2D(0, speed * dt)
                controlled = True

            if self.left_key and self.inputHandler[self.left_key]:
                self.value -= QVector2D(speed * dt, 0)
                controlled = True
            elif self.right_key and self.inputHandler[self.right_key]:
                self.value += QVector2D(speed * dt, 0)
                controlled = True
        
        if not (controlled or self.__sticky):
            self.value *= 0.9

    def resizeEvent(self, a0: QResizeEvent):
        self.__stickPos = self.getDrawingCenter()
        self.repaint()
        return super().resizeEvent(a0)

    def paintEvent(self, a0: QPaintEvent):
        def centeredRect(center, w, h):
            return QRectF(center.x() - w / 2, center.y() - h / 2, w, h)

        outerDia, innerDia = self.getOuterDia(), self.getInnerDia()
        
        painter = QPainter(self)

        pen = QPen()
        pen.setWidth(2)
        
        brush = QBrush()
        brush.setStyle(Qt.BrushStyle.SolidPattern)

        bounds = centeredRect(self.getDrawingCenter(), outerDia, outerDia)
        pen.setColor(Qt.GlobalColor.black) ; painter.setPen(pen)
        brush.setColor(Qt.GlobalColor.lightGray) ; painter.setBrush(brush)
        painter.drawEllipse(bounds)

        bounds = centeredRect(self.__stickPos, innerDia, innerDia)
        pen.setColor(Qt.GlobalColor.black) ; painter.setPen(pen)
        brush.setColor(Qt.GlobalColor.red) ; painter.setBrush(brush) 
        painter.drawEllipse(bounds)
    
        painter.end()
            
        return super().paintEvent(a0)