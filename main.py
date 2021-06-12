from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QToolTip, QPushButton, QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QBasicTimer
import sys
from aircraft_game_control import *
import mouse
import keyboard

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        self.aircraft_con = game_aircraft_control()
        super(MainWindow, self).__init__(parent)
        self.setWindowFlags(
            QtCore.Qt.Window |
            QtCore.Qt.WindowTitleHint |
            QtCore.Qt.WindowStaysOnTopHint
            | QtCore.Qt.FramelessWindowHint
        )
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.setStyleSheet("background:transparent;")
        
        self.roll = QLabel('Ail: 0%', self)
        self.roll.move(0, 0)
        self.roll.setFont(QFont('SansSerif', 15))
        self.roll.setFixedSize(400, 50)

        self.ele = QLabel('ELE: 0%', self)
        self.ele.move(0, 40)
        self.ele.setFont(QFont('SansSerif', 15))
        self.ele.setFixedSize(400, 50)

        self.rud = QLabel('Rud: 0%', self)
        self.rud.move(0, 80)
        self.rud.setFont(QFont('SansSerif', 15))
        self.rud.setFixedSize(400, 50)

        self.thr = QLabel('THR: 0%', self)
        self.thr.move(0, 120)
        self.thr.setFont(QFont('SansSerif', 15))
        self.thr.setFixedSize(400, 50)

        self.setGeometry(0, 500, 1024, 768)

        self.timer = QBasicTimer()
        self.timer.start(5, self)

        self.mouse_x0 = None
        self.mouse_y0 = None
        self.vmouse_x = 0
        self.vmouse_y = 0
        
        self.freelook_mouse_x0 = None
        self.freelook_mouse_y0 = None

        self.is_free_look = False

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
        
        if self.is_free_look:
            self.vmouse_x += (x - self.freelook_mouse_x0)*dt
            self.vmouse_y += (y - self.freelook_mouse_y0)*dt
            self.aircraft_con.set_mouse_free_look(self.vmouse_x, self.vmouse_y)
            mouse.move(self.freelook_mouse_x0, self.freelook_mouse_y0)
        else:
            self.vmouse_x = 0
            self.vmouse_y = 0
            self.aircraft_con.set_mouse_aircraft_control(x - self.mouse_x0, y - self.mouse_y0)


    def timerEvent(self, e):
        self.roll.setText('Ail: {0:.1f}%'.format(self.aircraft_con.get_ail()*100))
        self.ele.setText("ELE: {:.1f}%".format(self.aircraft_con.get_ele()*100))
        self.rud.setText("RUD: {:.1f}%".format(self.aircraft_con.get_rud()*100))
        self.thr.setText("THR: {:.1f}%".format(self.aircraft_con.get_thr()*100))
        
        _m = mouse.get_position()
        self.keyboard_watcher()

        self.set_mouse_cur_pos_new(_m, 0.01)
        # self.set_mouse_cur_pos(_m)
        self.aircraft_con.update()

    def keyboard_watcher(self):
        if keyboard.is_pressed("c"):
            if self.freelook_mouse_x0 == None:
                self.freelook_mouse_x0 = self.mouse[0]
                self.freelook_mouse_y0 = self.mouse[1]
            self.is_free_look = True
        else:
            self.aircraft_con.set_mouse_free_look(0, 0)
            self.freelook_mouse_x0 = None
            self.freelook_mouse_y0 = None
            self.is_free_look = False

        if keyboard.is_pressed("alt+r"):
            self.mouse_x0 = None
            self.mouse_y0 = None

        if keyboard.is_pressed("shift"):
            self.aircraft_con.inc_thr(0.01)

        if keyboard.is_pressed("ctrl"):
            self.aircraft_con.dec_thr(0.01)
        
        # if keyboard.is_pressed("esc"):
            # sys.exit(0)

        


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    win.installEventFilter(win)
    sys.exit(app.exec_())
