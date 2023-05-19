from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from inputs import QInputHandler
from joystick import QJoystick, QJoystickAxes
from roslibpy import Ros
from publisher import TwistPublisher

class MainWindow(QMainWindow):
    JOYSTICK_SPEED = 3
    UPDATE_DT = 16
    PUBLISH_DT = 500
    DEFAULT_TOPIC = "/cmd_vel"
    HELP_TEXT = '\n'.join([
        "Use the angle joystick to steer the bot, and",
        "the speed joystick to drive forward and back.",
        "Press the middle mouse button or the shift key",
        "to lock either joystick at a value."
    ])

    def __init__(self):
        super().__init__()
        
        self.inputHandler = QInputHandler()

        self.ros = Ros(host='localhost', port=9090)
        self.ros.run()
        
        self.publisherNode = None
        self.createNode(MainWindow.DEFAULT_TOPIC)

        self.setWindowTitle("AresBot Controller")
        self.setGeometry(0, 0, 600, 360)
        MainWindow.centerWindow(self)
        
        self.createWidgets()
        self.arrangeWidgets()

        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(self.onUpdate)
        self.updateTimer.start(MainWindow.UPDATE_DT)

        self.publishTimer = QTimer()
        self.publishTimer.timeout.connect(self.onPublish)
        self.publishTimer.start(MainWindow.PUBLISH_DT)

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
        
        self.angleJS = QJoystick(QJoystickAxes.UpperHalf, self.inputHandler)
        self.angleJS.setKeys(Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D)
        
        self.speedLbl = QLabel("Speed")
        self.speedLbl.setFont(regularFont)
        self.speedLbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.speedJS = QJoystick(QJoystickAxes.Vertical, self.inputHandler)
        self.speedJS.setKeys(Qt.Key.Key_Up, Qt.Key.Key_Down, None, None)

        self.thirdRow = QHBoxLayout()
        self.thirdRow.setContentsMargins(10, 20, 10, 10)
        
        self.topicLbl = QLabel("Topic:")
        self.topicLbl.setFont(smallFont)
        
        self.topicEdit = QLineEdit()
        self.topicEdit.setText(MainWindow.DEFAULT_TOPIC)
        self.topicEdit.setPlaceholderText("/topic_name")
        self.topicEdit.setFont(smallFont)
        self.topicEdit.returnPressed.connect(lambda: self.createNode(self.topicEdit.text()))

        self.helpPopup = QLabel(MainWindow.HELP_TEXT)
        self.helpPopup.setWindowTitle("Help")
        self.helpPopup.setGeometry(QRect(0, 0, 400, 200))
        self.helpPopup.setAlignment(Qt.AlignmentFlag.AlignCenter)
        MainWindow.centerWindow(self.helpPopup)

        self.helpButton = QPushButton()
        self.helpButton.setText('?')
        self.helpButton.pressed.connect(self.helpPopup.show)

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
        self.thirdRow.addSpacing(40)
        self.thirdRow.addWidget(self.helpButton)
        self.thirdRow.addStretch()
        self.grid.addLayout(self.thirdRow, 6, 0, 1, 2)

    def createNode(self, topic_name):
        if self.publisherNode:
            self.publisherNode.destroy()
        self.publisherNode = TwistPublisher(self.ros, topic_name)

    def onPublish(self):
        if self.publisherNode:
            self.publisherNode.publish(angle=self.angleJS.value, speed=self.speedJS.value)

    def onUpdate(self):
        self.speedJS.update(MainWindow.JOYSTICK_SPEED, MainWindow.UPDATE_DT / 1000)
        self.angleJS.update(MainWindow.JOYSTICK_SPEED, MainWindow.UPDATE_DT / 1000)

    def closeEvent(self, a0: QCloseEvent):
        self.updateTimer.stop()
        self.publishTimer.stop()
        self.ros.terminate()
        return super().closeEvent(a0)

    def keyPressEvent(self, ev: QKeyEvent):
        if not ev.isAutoRepeat():
            self.inputHandler.keyPressed(ev.key())
        return super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev: QKeyEvent):
        if not ev.isAutoRepeat():
            self.inputHandler.keyReleased(ev.key())
        return super().keyPressEvent(ev)

    @staticmethod
    def centerWindow(window):
        rect = window.frameGeometry()
        center = QDesktopWidget().availableGeometry().center()
        rect.moveCenter(center)
        window.move(rect.topLeft())

    