from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QToolTip, QPushButton, QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QBasicTimer
import sys
from DCSEasyControl.aircraft_game_control import *
import mouse
import keyboard
import win32gui
import os

DCS_X = 0
DCS_Y = 0
DCS_W = 1920
DCS_H = 1080
DCS_CX = 1000
DCS_CY = 500
DCS_WIN_NAME = "Digital Combat Simulator"
WIN_NAME = "DCSEasyControl"
from win32api import GetSystemMetrics

def callback(hwnd, extra):
    rect = win32gui.GetWindowRect(hwnd)
    x = rect[0]
    y = rect[1]
    w = rect[2] - x
    h = rect[3] - y

    if w == 0:
        w = GetSystemMetrics(0)
        h = GetSystemMetrics(1)

    global DCS_X, DCS_Y, DCS_W, DCS_H, DCS_CX, DCS_CY

    if win32gui.GetWindowText(hwnd) == DCS_WIN_NAME:
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
        import pathlib
        path = pathlib.Path(__file__).parent.absolute()
        print(path)
        self.aircraft_con = game_aircraft_control(DCS_W, DCS_H, path)
        super(MainWindow, self).__init__(parent)
        self.setWindowFlags(
            QtCore.Qt.WindowStaysOnTopHint |
            QtCore.Qt.Window |
            QtCore.Qt.WindowTitleHint |
            QtCore.Qt.FramelessWindowHint
        )
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.setStyleSheet("background:transparent;")
        
        self.setWindowTitle(WIN_NAME)
        
        self.status = QLabel('Wait for DCS Connection', self)
        self.status.move(30, 80)
        self.status.setFont(QFont('Consolas', 15))
        self.status.setFixedSize(DCS_W, 500)
        self.status.setStyleSheet('color: yellow')


        self.setGeometry(DCS_X, DCS_Y, DCS_W, DCS_H)

        self.aim_tgt = QLabel('+', self)
        self.aim_tgt.move(0, 0)
        self.aim_tgt.setFont(QFont('Arial', 30))
        self.aim_tgt.setFixedSize(30, 30)
        self.aim_tgt.setStyleSheet('color: red')

        self.load_image_label()

        self.vmouse_x = 0
        self.vmouse_y = 0

        self.vmouse_rate = 1
        
        self.windows_center_x0 = DCS_CX
        self.windows_center_y0 = DCS_CY

        self.free_look_x0 = None
        self.free_look_y0 = None

        self.is_free_look = False

        self.timer = QBasicTimer()
        self.timer.start(MAIN_WIN_DURATION, self)

        self.count = 0

        self.border_x_min = 0
        self.border_x_max = DCS_W - VMOUSE_SIZE
    
        self.border_y_min = 0 
        self.border_y_max = DCS_H - VMOUSE_SIZE

        print("Mouse Border, x", self.border_x_min, self.border_x_max, "Y", self.border_y_min, self.border_y_max)

    def load_image_label(self):
        # input_image = cv2.imread("./assets/circle_small.png", cv2.IMREAD_UNCHANGED)
        # height, width, channels = input_image.shape
        # bytesPerLine = channels * width
        # qImg = QtGui.QImage(input_image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGBA8888)
        # pixmap01 = QtGui.QPixmap.fromImage(qImg)

        pixmap_image = QtGui.QPixmap("./assets/circle_small.png")

        self.virtual_mouse = QLabel("virtual_mouse", self)
        self.virtual_mouse.setStyleSheet("background:transparent;")
        self.virtual_mouse.setPixmap(pixmap_image)
        self.virtual_mouse.setAlignment(QtCore.Qt.AlignCenter)
        self.virtual_mouse.setScaledContents(True)
        self.virtual_mouse.setMinimumSize(1,1)
        self.virtual_mouse.setFixedSize(VMOUSE_SIZE, VMOUSE_SIZE)
        self.virtual_mouse.move(DCS_CX, DCS_CY)

    def set_mouse_cur_pos_new(self, m, dt):
        x = m[0]
        y = m[1]
        self.mouse = m
        
        dmouse_x = (x - self.windows_center_x0)*self.vmouse_rate
        dmouse_y = (y - self.windows_center_y0)*self.vmouse_rate
        
        self.status.setStyleSheet('color: red')

        if self.is_free_look:
            self.aircraft_con.cam.set_mouse_free_look(dmouse_x, dmouse_y)
            self.status.setText(f"FreeLook\n{self.aircraft_con.cam.view_yaw*57.3:3.1f} {self.aircraft_con.cam.view_pitch*57.3:3.1f}")
        else:
            self.aircraft_con.cam.set_mouse_free_look_off()
            if self.virtual_mouse_inborder(self.vmouse_x, self.vmouse_y):
                self.aircraft_con.set_mouse_aircraft_control(dmouse_x, dmouse_y)
            if control_style == "warthunder":
                self.status.setText(f"MouseAim\n{self.aircraft_con.status()}")
            else:
                self.status.setText(f"{mouse_joystick_elemode}&RolRate\n{self.aircraft_con.status()}")

        mouse.move(self.windows_center_x0, self.windows_center_y0)

        self.move_virtual_mouse(self.vmouse_x, self.vmouse_y)
        self.move_aim_tgt(self.aim_tgt_x, self.aim_tgt_y)

    def vmouse_to_screen_coor(self, x, y):
        _X = int(x - VMOUSE_SIZE/2 + DCS_W/2)
        _Y = int(y + DCS_H/2 - VMOUSE_SIZE/2)
        return _X, _Y

    def virtual_mouse_inborder(self, x , y):
        _X, _Y = self.vmouse_to_screen_coor(x, y)
        return self.border_x_max > _X > self.border_x_min and \
            self.border_y_max > _Y > self.border_y_min

    def move_virtual_mouse(self, x, y):
        #For window mode
        _X, _Y = self.vmouse_to_screen_coor(x, y)
        self.virtual_mouse.move(_X, _Y)

    def move_aim_tgt(self, x, y):
        #For window mode
        #Full screen
        self.aim_tgt.move(int(x - 15 + DCS_W/2),int(y + DCS_H/2 - 15))

    def timerEvent(self, e):
        self.keyboard_watcher_sys()

        _m = mouse.get_position()
        self.mouse = _m
        self.vmouse_x, self.vmouse_y, self.aim_tgt_x, self.aim_tgt_y = self.aircraft_con.pre_update()

        top_win = win32gui.GetWindowText(win32gui.GetForegroundWindow())
        
        # if not self.aircraft_con.OK:
        #     self.status.setText(f"Wait for DCS Connection")
        #     self.status.setStyleSheet('color: yellow')

        if HIDE_WIHTOUT_DCS:
            if top_win != DCS_WIN_NAME and top_win != WIN_NAME:
                self.setVisible(False)
            else:
                self.setVisible(True)
            return

        if not self.aircraft_con.OK or not self.aircraft_con.updated:
            return

        if self.count % 3 == 0:
            self.set_mouse_cur_pos_new(_m, MAIN_WIN_DURATION*3/1000.0)

        self.keyboard_watcher()
        self.aircraft_con.update()
        self.count += 1

    def keyboard_watcher_sys(self):
        if keyboard.is_pressed(keyboard_exit):
            print("User require exit")
            sys.exit(0)

    def keyboard_watcher(self):
        if keyboard.is_pressed(keyboard_freelook):
            self.is_free_look = True
        else:
            self.is_free_look = False

        if keyboard.is_pressed(keyboard_inc_thr):
            self.aircraft_con.inc_thr(0.01)

        if keyboard.is_pressed(keyboard_dec_thr):
            self.aircraft_con.dec_thr(0.01)
        
        if keyboard.is_pressed(keyboard_ele_min):
            self.aircraft_con.set_user_ele(-1)

        if keyboard.is_pressed(keyboard_ele_max):
            self.aircraft_con.set_user_ele(1)

        if keyboard.is_pressed(keyboard_ail_min):
            self.aircraft_con.set_user_ail(-1)

        if keyboard.is_pressed(keyboard_ail_max):
            self.aircraft_con.set_user_ail(1)

        if keyboard.is_pressed(keyboard_rud_min):
            self.aircraft_con.set_user_rud(-1)

        if keyboard.is_pressed(keyboard_rud_max):
            self.aircraft_con.set_user_rud(1)
        


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win32gui.EnumWindows(callback, None)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
