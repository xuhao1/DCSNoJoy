from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QToolTip, QPushButton, QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QBasicTimer
import sys
from aircraft_game_control import *
import mouse
import keyboard
import cv2
import win32gui

DCS_X = 0
DCS_Y = 0
DCS_W = 1920
DCS_H = 1080
DCS_CX = 1000
DCS_CY = 500

def callback(hwnd, extra):
    rect = win32gui.GetWindowRect(hwnd)
    x = rect[0]
    y = rect[1]
    w = rect[2] - x
    h = rect[3] - y
    global DCS_X, DCS_Y, DCS_W, DCS_H, DCS_CX, DCS_CY

    if win32gui.GetWindowText(hwnd) == "Digital Combat Simulator":
        DCS_X = x
        DCS_Y = y
        DCS_W = w
        DCS_H = h
        DCS_CX = x + w//2
        DCS_CY = y + h//2

        print("Window %s:" % win32gui.GetWindowText(hwnd))
        print("\tLocation: (%d, %d)" % (x, y))
        print("\t    Size: (%d, %d)" % (w, h))
        print("\t    Center: (%d, %d)" % (DCS_CX, DCS_CY))

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        self.aircraft_con = game_aircraft_control()
        super(MainWindow, self).__init__(parent)
        self.setWindowFlags(
            QtCore.Qt.Window |
            QtCore.Qt.WindowTitleHint |
            QtCore.Qt.WindowStaysOnTopHint |
            QtCore.Qt.FramelessWindowHint
        )
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.setStyleSheet("background:transparent;")
        
        self.roll = QLabel('Ail: 0%', self)
        self.roll.move(30, 50)
        self.roll.setFont(QFont('SansSerif', 15))
        self.roll.setFixedSize(400, 50)

        self.ele = QLabel('ELE: 0%', self)
        self.ele.move(30, 90)
        self.ele.setFont(QFont('SansSerif', 15))
        self.ele.setFixedSize(400, 50)

        self.rud = QLabel('Rud: 0%', self)
        self.rud.move(30, 130)
        self.rud.setFont(QFont('SansSerif', 15))
        self.rud.setFixedSize(400, 50)

        self.thr = QLabel('THR: 0%', self)
        self.thr.move(30, 180)
        self.thr.setFont(QFont('SansSerif', 15))
        self.thr.setFixedSize(400, 50)

        self.setGeometry(DCS_X, DCS_Y, DCS_W, DCS_H)

        self.load_image_label()

        self.mouse_x0 = None
        self.mouse_y0 = None
        self.vmouse_x = 0
        self.vmouse_y = 0

        self.vmouse_rate = 1
        
        self.freelook_mouse_x0 = DCS_CX
        self.freelook_mouse_y0 = DCS_CY

        self.is_free_look = False

        self.timer = QBasicTimer()
        self.timer.start(5, self)

        self.count = 0
    
    def move_aim_tgt(self, x, y):
        self.aim_tgt.move(int(x - 25 + DCS_W/2),int(y-25+ DCS_H/2))

    def load_image_label(self):
        # input_image = cv2.imread("./assets/circle_small.png", cv2.IMREAD_UNCHANGED)
        # height, width, channels = input_image.shape
        # bytesPerLine = channels * width
        # qImg = QtGui.QImage(input_image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGBA8888)
        # pixmap01 = QtGui.QPixmap.fromImage(qImg)

        pixmap_image = QtGui.QPixmap("./assets/circle_small.png")
        self.aim_tgt = QLabel("aim_tgt", self)
        self.aim_tgt.setStyleSheet("background:transparent;")
        self.aim_tgt.setPixmap(pixmap_image)
        self.aim_tgt.setAlignment(QtCore.Qt.AlignCenter)
        self.aim_tgt.setScaledContents(True)
        self.aim_tgt.setMinimumSize(1,1)
        self.aim_tgt.setFixedSize(50, 50)
        self.aim_tgt.move(DCS_CX, DCS_CY)

    def set_mouse_cur_pos(self, m):
        x = m[0]
        y = m[1]
        self.mouse = m
        if self.mouse_x0 == None:
            self.mouse_x0 = x

        if self.mouse_y0 == None:
            self.mouse_y0 = y

        if self.is_free_look:
            self.aircraft_con.set_mouse_free_look(x - self.freelook_mouse_x0, y - self.freelook_mouse_y0)
        else:
            self.aircraft_con.set_mouse_aircraft_control(x - self.mouse_x0, y - self.mouse_y0)


    def set_mouse_cur_pos_new(self, m, dt):
        x = m[0]
        y = m[1]
        self.mouse = m
        if self.mouse_x0 == None:
            self.mouse_x0 = x

        if self.mouse_y0 == None:
            self.mouse_y0 = y

        self.vmouse_x += (x - self.freelook_mouse_x0)*self.vmouse_rate
        self.vmouse_y += (y - self.freelook_mouse_y0)*self.vmouse_rate
        
        if self.is_free_look:
            self.aircraft_con.set_mouse_free_look(self.vmouse_x, self.vmouse_y)
            mouse.move(DCS_CX, DCS_CY)
        else:
            mouse.move(DCS_CX, DCS_CY)
            self.aircraft_con.set_mouse_aircraft_control(self.vmouse_x, self.vmouse_y)
        
        self.move_aim_tgt(self.vmouse_x, self.vmouse_y)


    def timerEvent(self, e):
        if self.count % 10 == 0:
            self.roll.setText('Ail: {0:.1f}%'.format(self.aircraft_con.get_ail()*100))
            self.ele.setText("ELE: {:.1f}%".format(self.aircraft_con.get_ele()*100))
            self.rud.setText("RUD: {:.1f}%".format(self.aircraft_con.get_rud()*100))
            self.thr.setText("THR: {:.1f}%".format(self.aircraft_con.get_thr()*100))
            
        _m = mouse.get_position()
        self.mouse = _m
        self.set_mouse_cur_pos_new(_m, 0.01)

        self.keyboard_watcher()

        self.aircraft_con.update()
        self.count += 1

    def keyboard_watcher(self):
        if keyboard.is_pressed("c"):
            self.is_free_look = True
        else:
            self.aircraft_con.set_mouse_free_look(0, 0)
            self.is_free_look = False

        if keyboard.is_pressed("alt+r"):
            self.mouse_x0 = None
            self.mouse_y0 = None

        if keyboard.is_pressed("shift"):
            self.aircraft_con.inc_thr(0.01)

        if keyboard.is_pressed("ctrl"):
            self.aircraft_con.dec_thr(0.01)
        
        if keyboard.is_pressed("shift+esc"):
            sys.exit(0)

        


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win32gui.EnumWindows(callback, None)
    win = MainWindow()
    print("Show main window")
    win.show()
    sys.exit(app.exec_())
