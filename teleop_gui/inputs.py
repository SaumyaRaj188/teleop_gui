from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class QInputHandler:
    def __init__(self):
        self.keymap = {}

    def keyPressed(self, key):
        self.keymap[key] = True
    
    def keyReleased(self, key):
        self.keymap[key] = False

    def __getitem__(self, key):
        return self.keymap.get(key, False)