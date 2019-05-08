#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import resources_rc
from PyQt4 import QtCore, QtGui
import serial
import rospy
import time
from std_msgs.msg import String

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class MyWidget(QtGui.QWidget): #Added extra whole function
    keyPressed = QtCore.pyqtSignal(QtCore.QEvent)
    keyReleased = QtCore.pyqtSignal(QtCore.QEvent)

    def __init__(self):
        super(MyWidget, self).__init__()

    def keyPressEvent(self, event):
        super(MyWidget, self).keyPressEvent(event)
        self.keyPressed.emit(event)

    def keyReleaseEvent(self, event):
        super(MyWidget, self).keyReleaseEvent(event)
        self.keyReleased.emit(event)

class Ui_main_window(object):

	#####################################################################################Setup Functions#########################################################################################
    def setupUi(self, main_window):
        main_window.setObjectName(_fromUtf8("main_window"))
        main_window.resize(800, 1000)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(main_window.sizePolicy().hasHeightForWidth())
        main_window.setSizePolicy(sizePolicy)
        main_window.setMaximumSize(QtCore.QSize(800, 1000))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/resources/moon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        main_window.setWindowIcon(icon)
        self.verticalLayout = QtGui.QVBoxLayout(main_window)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.main_grid = QtGui.QHBoxLayout()
        self.main_grid.setObjectName(_fromUtf8("main_grid"))
        self.vertical_grid = QtGui.QVBoxLayout()
        self.vertical_grid.setObjectName(_fromUtf8("vertical_grid"))
        spacerItem = QtGui.QSpacerItem(20, 300, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem)
        self.rover_speed_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setPointSize(26)
        font.setBold(True)
        font.setWeight(75)
        self.rover_speed_label.setFont(font)
        self.rover_speed_label.setTextFormat(QtCore.Qt.AutoText)
        self.rover_speed_label.setScaledContents(False)
        self.rover_speed_label.setAlignment(QtCore.Qt.AlignCenter)
        self.rover_speed_label.setObjectName(_fromUtf8("rover_speed_label"))
        self.vertical_grid.addWidget(self.rover_speed_label)
        self.speed_slider = QtGui.QSlider(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.speed_slider.sizePolicy().hasHeightForWidth())
        self.speed_slider.setSizePolicy(sizePolicy)
        self.speed_slider.setAutoFillBackground(False)
        self.speed_slider.setStyleSheet(_fromUtf8("#speed_slider:focus\n"
"{\n"
"    outline: none!important;\n"
"}"))
        self.speed_slider.setProperty("value", 0)
        self.speed_slider.setOrientation(QtCore.Qt.Horizontal)
        self.speed_slider.setTickPosition(QtGui.QSlider.TicksAbove)
        self.speed_slider.setTickInterval(5)
        self.speed_slider.setObjectName(_fromUtf8("speed_slider"))
        self.speed_slider.valueChanged.connect(self.speed_slider_event) #Extra line for value change event for text label
        self.vertical_grid.addWidget(self.speed_slider)
        spacerItem1 = QtGui.QSpacerItem(20, 300, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem1)
        self.battery_level_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setPointSize(26)
        font.setBold(True)
        font.setWeight(75)
        self.battery_level_label.setFont(font)
        self.battery_level_label.setAlignment(QtCore.Qt.AlignCenter)
        self.battery_level_label.setObjectName(_fromUtf8("battery_level_label"))
        self.vertical_grid.addWidget(self.battery_level_label)
        self.battery_bar = QtGui.QProgressBar(main_window)
        self.battery_bar.setProperty("value", 0)
        self.battery_bar.setObjectName(_fromUtf8("battery_bar"))
        self.vertical_grid.addWidget(self.battery_bar)
        spacerItem2 = QtGui.QSpacerItem(20, 300, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem2)
        self.main_grid.addLayout(self.vertical_grid)
        self.verticalLayout.addLayout(self.main_grid)
        main_window.keyPressed.connect(self.onKeyPress) #Added extra
        main_window.keyReleased.connect(self.offKeyPress) #Added extra


        self.retranslateUi(main_window)
        QtCore.QMetaObject.connectSlotsByName(main_window)

        self.initros(main_window) #Initialize ROS

    def retranslateUi(self, main_window):
        main_window.setWindowTitle(_translate("main_window", "Armstrong Controller", None))
        self.rover_speed_label.setText(_translate("main_window", "Rover Speed: " + str(self.speed_slider.value()) + "%", None))
        self.battery_level_label.setText(_translate("main_window", "Battery Level: 0.0V", None))

    #####################################################################################Additional Functions#########################################################################################
    def battery_progress(self, battery):
        float_battery = float(battery)
        actual_battery = float("{0:.1f}".format(float_battery)) - 0.2 #0.2 higher than recorded voltmeter
        percent = ((actual_battery - 10.0) / 2) * 100 #Viable range of only 10-12V
        self.battery_bar.setProperty("value", int(percent))
        self.battery_level_label.setText(_translate("main_window", "Battery Level: " + str(actual_battery) + "V", None))


    def initros(self, window):
    	self.pub = rospy.Publisher('arlo_wheels', String, queue_size=100)
    	rospy.init_node('Motor_Control_Talker', anonymous=True)
        self.battery_thread = ThreadClass() #Start thread for constantly getting the battery level
        self.battery_thread.start()
        window.connect(self.battery_thread, QtCore.SIGNAL("Battery_Value"), self.battery_progress) #Now connect main window thread for receiving that value
        self.pub.publish("rst\r")


    def arlo_speed_convert(self, speed):
    	'''0 to 200 encoder counts per second'''
    	if (speed > 0):
    		#Might have to convert to int 
    		arlo_speed = (200 * (speed * 0.01))
    	else:
    		arlo_speed = 0

    	return arlo_speed


    def onKeyPress(self, event):
        self.key_event = event
        if event.key() == QtCore.Qt.Key_I:
            arlo_speed = int(self.arlo_speed_convert(self.speed_slider.value()))
            self.pub.publish("gospd " + str(arlo_speed) + " " + str(arlo_speed) + "\r")

        elif event.key() == QtCore.Qt.Key_K:
            arlo_speed = int(-1 * self.arlo_speed_convert(self.speed_slider.value()))
            self.pub.publish("gospd " + str(arlo_speed) + " " + str(arlo_speed) + "\r")

        elif event.key() == QtCore.Qt.Key_J:
            arlo_speed = int(self.arlo_speed_convert(self.speed_slider.value()))
            self.pub.publish("gospd " + str(-1*arlo_speed) + " " + str(arlo_speed) + "\r")

        elif event.key() == QtCore.Qt.Key_L:
            arlo_speed = int(self.arlo_speed_convert(self.speed_slider.value()))
            self.pub.publish("gospd " + str(arlo_speed) + " " + str(-1*arlo_speed) + "\r")

        else:
             event.ignore()


    def offKeyPress(self, event):
        if (not self.key_event.isAutoRepeat()):
            self.pub.publish("go 0 0\r")

    def speed_slider_event(self):
    	self.rover_speed_label.setText(_translate("main_window", "Rover Speed: " + str(self.speed_slider.value()) +"%", None))


class ThreadClass(QtCore.QThread):
    def __init__(self, parent = None):
        super(ThreadClass, self).__init__(parent)

    def run(self):
        while 1:
            time.sleep(10.0)
            voltage = rospy.get_param("/battery_level")
            self.emit(QtCore.SIGNAL('Battery_Value'),voltage)
            
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    app.setStyle("GTK+") #Added extra
    main_window = MyWidget() #Added extra
    ui = Ui_main_window()
    ui.setupUi(main_window)
    main_window.show()
    sys.exit(app.exec_())

