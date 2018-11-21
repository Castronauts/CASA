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

class Ui_main_window(object):

	#####################################################################################Setup Functions#########################################################################################
    def setupUi(self, main_window):
        main_window.setObjectName(_fromUtf8("main_window"))
        main_window.resize(800, 900)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(main_window.sizePolicy().hasHeightForWidth())
        main_window.setSizePolicy(sizePolicy)
        main_window.setMaximumSize(QtCore.QSize(800, 900))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/resources/moon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        main_window.setWindowIcon(icon)
        self.verticalLayout = QtGui.QVBoxLayout(main_window)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.main_grid = QtGui.QHBoxLayout()
        self.main_grid.setObjectName(_fromUtf8("main_grid"))
        self.vertical_grid = QtGui.QVBoxLayout()
        self.vertical_grid.setObjectName(_fromUtf8("vertical_grid"))
        spacerItem = QtGui.QSpacerItem(20, 50, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem)
        self.gripper_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setPointSize(26)
        font.setBold(True)
        font.setWeight(75)
        self.gripper_label.setFont(font)
        self.gripper_label.setAlignment(QtCore.Qt.AlignCenter)
        self.gripper_label.setObjectName(_fromUtf8("gripper_label"))
        self.vertical_grid.addWidget(self.gripper_label)
        self.box_sliders = QtGui.QHBoxLayout()
        self.box_sliders.setObjectName(_fromUtf8("box_sliders"))
        self.Z_Axis_Layout = QtGui.QVBoxLayout()
        self.Z_Axis_Layout.setContentsMargins(50, -1, 50, -1)
        self.Z_Axis_Layout.setObjectName(_fromUtf8("Z_Axis_Layout"))
        self.z_top_label = QtGui.QLabel(main_window)
        self.z_top_label.setAlignment(QtCore.Qt.AlignCenter)
        self.z_top_label.setObjectName(_fromUtf8("z_top_label"))
        self.Z_Axis_Layout.addWidget(self.z_top_label)
        self.z_axis_bar = QtGui.QScrollBar(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.z_axis_bar.sizePolicy().hasHeightForWidth())
        self.z_axis_bar.setSizePolicy(sizePolicy)
        self.z_axis_bar.setMaximumSize(QtCore.QSize(15, 250))
        self.z_axis_bar.setAccessibleName(_fromUtf8(""))
        self.z_axis_bar.setTracking(True)
        self.z_axis_bar.setOrientation(QtCore.Qt.Vertical)
        self.z_axis_bar.setInvertedAppearance(False)
        self.z_axis_bar.setObjectName(_fromUtf8("z_axis_bar"))
        self.Z_Axis_Layout.addWidget(self.z_axis_bar, QtCore.Qt.AlignHCenter)
        self.z_bottom_label = QtGui.QLabel(main_window)
        self.z_bottom_label.setAlignment(QtCore.Qt.AlignCenter)
        self.z_bottom_label.setObjectName(_fromUtf8("z_bottom_label"))
        self.Z_Axis_Layout.addWidget(self.z_bottom_label)
        self.z_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.z_label.setFont(font)
        self.z_label.setAlignment(QtCore.Qt.AlignCenter)
        self.z_label.setObjectName(_fromUtf8("z_label"))
        self.Z_Axis_Layout.addWidget(self.z_label)
        self.box_sliders.addLayout(self.Z_Axis_Layout)
        self.X_Axis_Layout = QtGui.QHBoxLayout()
        self.X_Axis_Layout.setObjectName(_fromUtf8("X_Axis_Layout"))
        self.x_left_label = QtGui.QLabel(main_window)
        self.x_left_label.setAlignment(QtCore.Qt.AlignCenter)
        self.x_left_label.setIndent(-1)
        self.x_left_label.setObjectName(_fromUtf8("x_left_label"))
        self.X_Axis_Layout.addWidget(self.x_left_label)
        self.x_axis_extra_vertical = QtGui.QVBoxLayout()
        self.x_axis_extra_vertical.setContentsMargins(-1, 26, -1, 1)
        self.x_axis_extra_vertical.setObjectName(_fromUtf8("x_axis_extra_vertical"))
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.x_axis_extra_vertical.addItem(spacerItem1)
        self.x_axis_bar = QtGui.QScrollBar(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.x_axis_bar.sizePolicy().hasHeightForWidth())
        self.x_axis_bar.setSizePolicy(sizePolicy)
        self.x_axis_bar.setMaximumSize(QtCore.QSize(250, 15))
        self.x_axis_bar.setOrientation(QtCore.Qt.Horizontal)
        self.x_axis_bar.setObjectName(_fromUtf8("x_axis_bar"))
        self.x_axis_extra_vertical.addWidget(self.x_axis_bar, QtCore.Qt.AlignVCenter)
        spacerItem2 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.x_axis_extra_vertical.addItem(spacerItem2)
        self.x_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.x_label.setFont(font)
        self.x_label.setAlignment(QtCore.Qt.AlignCenter)
        self.x_label.setObjectName(_fromUtf8("x_label"))
        self.x_axis_extra_vertical.addWidget(self.x_label)
        self.X_Axis_Layout.addLayout(self.x_axis_extra_vertical)
        self.x_right_label = QtGui.QLabel(main_window)
        self.x_right_label.setAlignment(QtCore.Qt.AlignCenter)
        self.x_right_label.setObjectName(_fromUtf8("x_right_label"))
        self.X_Axis_Layout.addWidget(self.x_right_label)
        self.box_sliders.addLayout(self.X_Axis_Layout)
        self.Y_Axis_Layout = QtGui.QVBoxLayout()
        self.Y_Axis_Layout.setContentsMargins(39, -1, 39, -1)
        self.Y_Axis_Layout.setObjectName(_fromUtf8("Y_Axis_Layout"))
        self.y_forward_label = QtGui.QLabel(main_window)
        self.y_forward_label.setAlignment(QtCore.Qt.AlignCenter)
        self.y_forward_label.setObjectName(_fromUtf8("y_forward_label"))
        self.Y_Axis_Layout.addWidget(self.y_forward_label)
        self.y_axis_bar = QtGui.QScrollBar(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.y_axis_bar.sizePolicy().hasHeightForWidth())
        self.y_axis_bar.setSizePolicy(sizePolicy)
        self.y_axis_bar.setMaximumSize(QtCore.QSize(15, 250))
        self.y_axis_bar.setOrientation(QtCore.Qt.Vertical)
        self.y_axis_bar.setObjectName(_fromUtf8("y_axis_bar"))
        self.Y_Axis_Layout.addWidget(self.y_axis_bar, QtCore.Qt.AlignHCenter)
        self.y_backward_label = QtGui.QLabel(main_window)
        self.y_backward_label.setAlignment(QtCore.Qt.AlignCenter)
        self.y_backward_label.setObjectName(_fromUtf8("y_backward_label"))
        self.Y_Axis_Layout.addWidget(self.y_backward_label)
        self.y_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.y_label.setFont(font)
        self.y_label.setAlignment(QtCore.Qt.AlignCenter)
        self.y_label.setObjectName(_fromUtf8("y_label"))
        self.Y_Axis_Layout.addWidget(self.y_label)
        self.box_sliders.addLayout(self.Y_Axis_Layout)
        self.vertical_grid.addLayout(self.box_sliders)
        spacerItem3 = QtGui.QSpacerItem(20, 100, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem3)
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
        spacerItem4 = QtGui.QSpacerItem(20, 100, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem4)
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
        self.battery_bar.setInvertedAppearance(False)
        self.battery_bar.setObjectName(_fromUtf8("battery_bar"))
        self.vertical_grid.addWidget(self.battery_bar)
        spacerItem5 = QtGui.QSpacerItem(20, 100, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem5)
        self.main_grid.addLayout(self.vertical_grid)
        self.verticalLayout.addLayout(self.main_grid)

        self.retranslateUi(main_window)
        QtCore.QMetaObject.connectSlotsByName(main_window)

        self.initros(main_window) #Initialize ROS

    def retranslateUi(self, main_window):
        main_window.setWindowTitle(_translate("main_window", "ARM Controller", None))
        self.gripper_label.setText(_translate("main_window", "Gripper Movement Limits", None))
        self.z_top_label.setText(_translate("main_window", "Top Limit", None))
        self.z_bottom_label.setText(_translate("main_window", "Bottom Limit", None))
        self.z_label.setText(_translate("main_window", "Z-Axis", None))
        self.x_left_label.setText(_translate("main_window", "Left Limit", None))
        self.x_label.setText(_translate("main_window", "X-Axis", None))
        self.x_right_label.setText(_translate("main_window", "Right Limit", None))
        self.y_forward_label.setText(_translate("main_window", "Forward Limit", None))
        self.y_backward_label.setText(_translate("main_window", "Backward Limit", None))
        self.y_label.setText(_translate("main_window", "Y-Axis", None))
        self.rover_speed_label.setText(_translate("main_window", "Rover Speed: " + str(self.speed_slider.value()) + "%", None))
        self.battery_level_label.setText(_translate("main_window", "Battery Level: 0.0V", None))

    #####################################################################################Additional Functions#########################################################################################
    def battery_progress(self, battery):
        float_battery = float(battery)
        actual_battery = float("{0:.1f}".format(float_battery)) #higher than recorded voltmeter
        percent = ((actual_battery - 10.0) / 2) * 100 #Viable range of only 10-12V
        self.battery_bar.setProperty("value", int(percent))
        self.battery_level_label.setText(_translate("main_window", "Battery Level: " + str(actual_battery) + "V", None))


    def initros(self, window):
    	self.pub = rospy.Publisher('speed_arlo', String, queue_size=100)
    	rospy.init_node('Motor_Control_Talker', anonymous=True)
        self.battery_thread = ThreadClass() #Start thread for constantly getting the battery level
        self.battery_thread.start()
        window.connect(self.battery_thread, QtCore.SIGNAL("Battery_Value"), self.battery_progress) #Now connect main window thread for receiving that value


    def arlo_speed_convert(self, speed):
    	'''0 to 200 encoder counts per second'''
    	if (speed > 0):
    		#Might have to convert to int 
    		arlo_speed = (200 * (speed * 0.01))
    	else:
    		arlo_speed = 0

    	return arlo_speed

    def speed_slider_event(self):
    	self.rover_speed_label.setText(_translate("main_window", "Rover Speed: " + str(self.speed_slider.value()) +"%", None))

        self.pub.publish(str(int(self.arlo_speed_convert(self.speed_slider.value()))))


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
    main_window = QtGui.QWidget()
    ui = Ui_main_window()
    ui.setupUi(main_window)
    main_window.show()
    sys.exit(app.exec_())

