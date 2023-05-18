from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from inputs import QInputHandler
from joystick import QJoystick, QJoystickAxes

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("QTTest")
        self.setGeometry(0, 0, 600, 360)
        self.centerWindow()

        self.createWidgets()
        self.arrangeWidgets()
        self.setupUpdateLoop()

    def createWidgets(self):
        smallFont = self.font() ; smallFont.setPointSize(16)
        regularFont = self.font() ; regularFont.setPointSize(20)
        titleFont = self.font() ; titleFont.setPointSize(24)

        self.grid = QGridLayout()
        self.grid.setSpacing(0)
        self.grid.setContentsMargins(0, 0, 0, 0)

        self.titleLbl = QLabel("AresBot Controller")
        self.titleLbl.setFont(titleFont)
        self.titleLbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.titleLbl.setContentsMargins(0, 5, 0, 5)

        self.angleLbl = QLabel("Angle")
        self.angleLbl.setFont(regularFont)
        self.angleLbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.angleJS = QJoystick(QJoystickAxes.Horizontal | QJoystickAxes.Vertical)
        self.angleJS.setKeys(Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D)
        
        self.speedLbl = QLabel("Speed")
        self.speedLbl.setFont(regularFont)
        self.speedLbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.speedJS = QJoystick(QJoystickAxes.Horizontal)
        self.speedJS.setKeys(Qt.Key.Key_Up, Qt.Key.Key_Down, None, None)

        self.thirdRow = QHBoxLayout()
        self.thirdRow.setContentsMargins(10, 20, 10, 10)
        
        self.topicLbl = QLabel("Topic:")
        self.topicLbl.setFont(smallFont)
        
        self.topicEdit = QLineEdit()
        self.topicEdit.setPlaceholderText("/topic_name")
        self.topicEdit.setFont(smallFont)

    def arrangeWidgets(self):
        root = QWidget()
        self.setCentralWidget(root)
        root.setLayout(self.grid)

        # Row 0
        self.grid.addWidget(self.titleLbl, 0, 0, 1, 2)

        # Row 1
        self.grid.setRowStretch(1, 1)
        self.grid.addWidget(self.angleJS, 1, 0)
        self.grid.addWidget(self.speedJS, 1, 1)
        
        # Row 2
        self.grid.addWidget(self.angleLbl, 2, 0)
        self.grid.addWidget(self.speedLbl, 2, 1)

        # Row 3
        self.thirdRow.addStretch()
        self.thirdRow.addWidget(self.topicLbl)
        self.thirdRow.addSpacing(20)
        self.thirdRow.addWidget(self.topicEdit)
        self.thirdRow.addStretch()
        self.grid.addLayout(self.thirdRow, 6, 0, 1, 2)

    def setupUpdateLoop(self):
        self.inputHandler = QInputHandler()

        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(self.onUpdate)
        self.updateTimer.start(16)

    def onUpdate(self):
        SPEED = 3
        DT = 0.016
        
        self.speedJS.update(self.inputHandler, SPEED, DT)
        self.angleJS.update(self.inputHandler, SPEED, DT)

    def closeEvent(self, a0: QCloseEvent):
        self.updateTimer.stop()
        return super().closeEvent(a0)

    def keyPressEvent(self, ev: QKeyEvent):
        if not ev.isAutoRepeat():
            self.inputHandler.keyPressed(ev.key())
        return super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev: QKeyEvent):
        if not ev.isAutoRepeat():
            self.inputHandler.keyReleased(ev.key())
        return super().keyPressEvent(ev)

    def centerWindow(self):
        rect = self.frameGeometry()
        center = QDesktopWidget().availableGeometry().center()
        rect.moveCenter(center)
        self.move(rect.topLeft())