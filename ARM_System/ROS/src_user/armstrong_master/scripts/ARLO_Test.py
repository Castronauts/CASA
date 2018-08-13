#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import resources_rc
from PyQt4 import QtCore, QtGui
import serial
import rospy
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
        self.armstrong_controller_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setPointSize(26)
        font.setBold(True)
        font.setWeight(75)
        self.armstrong_controller_label.setFont(font)
        self.armstrong_controller_label.setAlignment(QtCore.Qt.AlignCenter)
        self.armstrong_controller_label.setObjectName(_fromUtf8("armstrong_controller_label"))
        self.vertical_grid.addWidget(self.armstrong_controller_label)
        self.joint_slider_grid = QtGui.QHBoxLayout()
        self.joint_slider_grid.setObjectName(_fromUtf8("joint_slider_grid"))
        self.gripper_grid = QtGui.QVBoxLayout()
        self.gripper_grid.setObjectName(_fromUtf8("gripper_grid"))
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.gripper_grid.addItem(spacerItem)
        self.gripper_slider = QtGui.QSlider(main_window)
        self.gripper_slider.setOrientation(QtCore.Qt.Horizontal)
        self.gripper_slider.setObjectName(_fromUtf8("gripper_slider"))
        self.gripper_slider.valueChanged.connect(self.gripper_slider_event) #Extra line for value change event for text label
        self.gripper_grid.addWidget(self.gripper_slider)
        self.gripper_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.gripper_label.setFont(font)
        self.gripper_label.setAlignment(QtCore.Qt.AlignCenter)
        self.gripper_label.setObjectName(_fromUtf8("gripper_label"))
        self.gripper_grid.addWidget(self.gripper_label)
        self.gripper_value_label = QtGui.QLabel(main_window)
        self.gripper_value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.gripper_value_label.setObjectName(_fromUtf8("gripper_value_label"))
        self.gripper_grid.addWidget(self.gripper_value_label)
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.gripper_grid.addItem(spacerItem1)
        self.joint_slider_grid.addLayout(self.gripper_grid)
        spacerItem2 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.joint_slider_grid.addItem(spacerItem2)
        self.joint1_grid = QtGui.QVBoxLayout()
        self.joint1_grid.setObjectName(_fromUtf8("joint1_grid"))
        self.joint1_slider = QtGui.QSlider(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joint1_slider.sizePolicy().hasHeightForWidth())
        self.joint1_slider.setSizePolicy(sizePolicy)
        self.joint1_slider.setProperty("value", 50)
        self.joint1_slider.setOrientation(QtCore.Qt.Vertical)
        self.joint1_slider.setObjectName(_fromUtf8("joint1_slider"))
        self.joint1_slider.valueChanged.connect(self.joint1_slider_event) #Extra line for value change event for text label
        self.joint1_grid.addWidget(self.joint1_slider)
        self.joint1_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.joint1_label.setFont(font)
        self.joint1_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint1_label.setObjectName(_fromUtf8("joint1_label"))
        self.joint1_grid.addWidget(self.joint1_label)
        self.joint1_value_label = QtGui.QLabel(main_window)
        self.joint1_value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint1_value_label.setObjectName(_fromUtf8("joint1_value_label"))
        self.joint1_grid.addWidget(self.joint1_value_label)
        self.joint_slider_grid.addLayout(self.joint1_grid)
        spacerItem3 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.joint_slider_grid.addItem(spacerItem3)
        self.joint2_grid = QtGui.QVBoxLayout()
        self.joint2_grid.setObjectName(_fromUtf8("joint2_grid"))
        self.joint2_slider = QtGui.QSlider(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joint2_slider.sizePolicy().hasHeightForWidth())
        self.joint2_slider.setSizePolicy(sizePolicy)
        self.joint2_slider.setProperty("value", 50)
        self.joint2_slider.setOrientation(QtCore.Qt.Vertical)
        self.joint2_slider.setObjectName(_fromUtf8("joint2_slider"))
        self.joint2_slider.valueChanged.connect(self.joint2_slider_event) #Extra line for value change event for text label
        self.joint2_grid.addWidget(self.joint2_slider)
        self.joint2_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.joint2_label.setFont(font)
        self.joint2_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint2_label.setObjectName(_fromUtf8("joint2_label"))
        self.joint2_grid.addWidget(self.joint2_label)
        self.joint2_value_label = QtGui.QLabel(main_window)
        self.joint2_value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint2_value_label.setObjectName(_fromUtf8("joint2_value_label"))
        self.joint2_grid.addWidget(self.joint2_value_label)
        self.joint_slider_grid.addLayout(self.joint2_grid)
        spacerItem4 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.joint_slider_grid.addItem(spacerItem4)
        self.joint3_grid = QtGui.QVBoxLayout()
        self.joint3_grid.setObjectName(_fromUtf8("joint3_grid"))
        self.joint3_slider = QtGui.QSlider(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joint3_slider.sizePolicy().hasHeightForWidth())
        self.joint3_slider.setSizePolicy(sizePolicy)
        self.joint3_slider.setProperty("value", 50)
        self.joint3_slider.setOrientation(QtCore.Qt.Vertical)
        self.joint3_slider.setObjectName(_fromUtf8("joint3_slider"))
        self.joint3_slider.valueChanged.connect(self.joint3_slider_event) #Extra line for value change event for text label
        self.joint3_grid.addWidget(self.joint3_slider)
        self.joint3_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.joint3_label.setFont(font)
        self.joint3_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint3_label.setObjectName(_fromUtf8("joint3_label"))
        self.joint3_grid.addWidget(self.joint3_label)
        self.joint3_value_label = QtGui.QLabel(main_window)
        self.joint3_value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint3_value_label.setObjectName(_fromUtf8("joint3_value_label"))
        self.joint3_grid.addWidget(self.joint3_value_label)
        self.joint_slider_grid.addLayout(self.joint3_grid)
        spacerItem5 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.joint_slider_grid.addItem(spacerItem5)
        self.joint4_grid = QtGui.QVBoxLayout()
        self.joint4_grid.setObjectName(_fromUtf8("joint4_grid"))
        self.joint4_slider = QtGui.QSlider(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joint4_slider.sizePolicy().hasHeightForWidth())
        self.joint4_slider.setSizePolicy(sizePolicy)
        self.joint4_slider.setProperty("value", 50)
        self.joint4_slider.setOrientation(QtCore.Qt.Vertical)
        self.joint4_slider.setObjectName(_fromUtf8("joint4_slider"))
        self.joint4_slider.valueChanged.connect(self.joint4_slider_event) #Extra line for value change event for text label
        self.joint4_grid.addWidget(self.joint4_slider)
        self.joint4_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.joint4_label.setFont(font)
        self.joint4_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint4_label.setObjectName(_fromUtf8("joint4_label"))
        self.joint4_grid.addWidget(self.joint4_label)
        self.joint4_value_label = QtGui.QLabel(main_window)
        self.joint4_value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint4_value_label.setObjectName(_fromUtf8("joint4_value_label"))
        self.joint4_grid.addWidget(self.joint4_value_label)
        self.joint_slider_grid.addLayout(self.joint4_grid)
        spacerItem6 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.joint_slider_grid.addItem(spacerItem6)
        self.joint5_grid = QtGui.QVBoxLayout()
        self.joint5_grid.setObjectName(_fromUtf8("joint5_grid"))
        self.joint5_slider = QtGui.QSlider(main_window)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joint5_slider.sizePolicy().hasHeightForWidth())
        self.joint5_slider.setSizePolicy(sizePolicy)
        self.joint5_slider.setProperty("value", 50)
        self.joint5_slider.setSliderPosition(50)
        self.joint5_slider.setOrientation(QtCore.Qt.Vertical)
        self.joint5_slider.setObjectName(_fromUtf8("joint5_slider"))
        self.joint5_slider.valueChanged.connect(self.joint5_slider_event) #Extra line for value change event for text label
        self.joint5_grid.addWidget(self.joint5_slider)
        self.joint5_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.joint5_label.setFont(font)
        self.joint5_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint5_label.setObjectName(_fromUtf8("joint5_label"))
        self.joint5_grid.addWidget(self.joint5_label)
        self.joint5_value_label = QtGui.QLabel(main_window)
        self.joint5_value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joint5_value_label.setObjectName(_fromUtf8("joint5_value_label"))
        self.joint5_grid.addWidget(self.joint5_value_label)
        self.joint_slider_grid.addLayout(self.joint5_grid)
        spacerItem7 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.joint_slider_grid.addItem(spacerItem7)
        self.state_grid = QtGui.QVBoxLayout()
        self.state_grid.setObjectName(_fromUtf8("state_grid"))
        self.states_combo_box = QtGui.QComboBox(main_window)
        self.states_combo_box.setEditable(False)
        self.states_combo_box.setObjectName(_fromUtf8("states_combo_box"))
        self.state_grid.addWidget(self.states_combo_box)
        self.play_saved_state_button = QtGui.QPushButton(main_window)
        self.play_saved_state_button.setObjectName(_fromUtf8("play_saved_state_button"))
        self.state_grid.addWidget(self.play_saved_state_button)
        self.save_joint_state_button = QtGui.QPushButton(main_window)
        self.save_joint_state_button.setObjectName(_fromUtf8("save_joint_state_button"))
        self.state_grid.addWidget(self.save_joint_state_button)
        self.joint_slider_grid.addLayout(self.state_grid)
        self.vertical_grid.addLayout(self.joint_slider_grid)
        self.arlo_controller_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setPointSize(26)
        font.setBold(True)
        font.setWeight(75)
        self.arlo_controller_label.setFont(font)
        self.arlo_controller_label.setTextFormat(QtCore.Qt.AutoText)
        self.arlo_controller_label.setScaledContents(False)
        self.arlo_controller_label.setAlignment(QtCore.Qt.AlignCenter)
        self.arlo_controller_label.setObjectName(_fromUtf8("arlo_controller_label"))
        self.vertical_grid.addWidget(self.arlo_controller_label)
        self.forward_button_grid = QtGui.QHBoxLayout()
        self.forward_button_grid.setSpacing(0)
        self.forward_button_grid.setObjectName(_fromUtf8("forward_button_grid"))
        self.forward_button = QtGui.QPushButton(main_window)
        self.forward_button.setMinimumSize(QtCore.QSize(200, 100))
        self.forward_button.setMaximumSize(QtCore.QSize(200, 100))
        self.forward_button.setStyleSheet(_fromUtf8("#forward_button\n"
"{\n"
"    background-color: transparent;\n"
"    border-image: url(:/resources/toparrow_normal.png);\n"
"    background: none;\n"
"    border: none;\n"
"    background-repeat: none;\n"
"}\n"
"#forward_button:pressed\n"
"{\n"
"    border-image: url(:/resources/toparrow_pressed.png);\n"
"}\n"
"#forward_button:focus\n"
"{\n"
"    outline: none;\n"
"}"))
        self.forward_button.setText(_fromUtf8(""))
        self.forward_button.setObjectName(_fromUtf8("forward_button"))
        self.forward_button.pressed.connect(self.forward_button_event) #Extra defined click event line
        #self.forward_button.setAutoRepeat(True) #Extra defined to keep going in the event if held down
        #self.forward_button.setAutoRepeatInterval(10) #Extra defined to repeat 10ms
        self.forward_button.released.connect(self.arlo_button_stop)
        self.forward_button_grid.addWidget(self.forward_button)
        self.vertical_grid.addLayout(self.forward_button_grid)
        self.left_right_button_grid = QtGui.QHBoxLayout()
        self.left_right_button_grid.setContentsMargins(-1, -1, 0, 0)
        self.left_right_button_grid.setSpacing(0)
        self.left_right_button_grid.setObjectName(_fromUtf8("left_right_button_grid"))
        spacerItem8 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.left_right_button_grid.addItem(spacerItem8)
        self.left_button = QtGui.QPushButton(main_window)
        self.left_button.setMinimumSize(QtCore.QSize(100, 200))
        self.left_button.setMaximumSize(QtCore.QSize(100, 200))
        self.left_button.setStyleSheet(_fromUtf8("#left_button\n"
"{\n"
"    background-color: transparent;\n"
"    border-image: url(:/resources/leftarrow_normal.png);\n"
"    background: none;\n"
"    border: none;\n"
"    background-repeat: none;\n"
"}\n"
"#left_button:pressed\n"
"{\n"
"    border-image: url(:/resources/leftarrow_pressed.png);\n"
"}\n"
"#left_button:focus\n"
"{\n"
"    outline: none;\n"
"}"))
        self.left_button.setText(_fromUtf8(""))
        self.left_button.setObjectName(_fromUtf8("left_button"))
        self.left_button.pressed.connect(self.left_button_event) #Extra defined click event
        #self.left_button.setAutoRepeat(True) #Extra defined to keep going in the event if held down
        #self.left_button.setAutoRepeatInterval(10) #Extra defined to repeat 10ms
        self.left_button.released.connect(self.arlo_button_stop)
        self.left_right_button_grid.addWidget(self.left_button)
        self.moon_label = QtGui.QLabel(main_window)
        self.moon_label.setMinimumSize(QtCore.QSize(200, 200))
        self.moon_label.setMaximumSize(QtCore.QSize(200, 200))
        self.moon_label.setText(_fromUtf8(""))
        self.moon_label.setPixmap(QtGui.QPixmap(_fromUtf8(":/resources/moon.png")))
        self.moon_label.setScaledContents(True)
        self.moon_label.setObjectName(_fromUtf8("moon_label"))
        self.left_right_button_grid.addWidget(self.moon_label)
        self.right_button = QtGui.QPushButton(main_window)
        self.right_button.setMinimumSize(QtCore.QSize(100, 200))
        self.right_button.setMaximumSize(QtCore.QSize(100, 200))
        self.right_button.setStyleSheet(_fromUtf8("#right_button\n"
"{\n"
"    background-color: transparent;\n"
"    border-image: url(:/resources/rightarrow_normal.png);\n"
"    background: none;\n"
"    border: none;\n"
"    background-repeat: none;\n"
"}\n"
"#right_button:pressed\n"
"{\n"
"    border-image: url(:/resources/rightarrow_pressed.png);\n"
"}\n"
"#right_button:focus\n"
"{\n"
"    outline: none;\n"
"}"))
        self.right_button.setText(_fromUtf8(""))
        self.right_button.setFlat(False)
        self.right_button.setObjectName(_fromUtf8("right_button"))
        self.right_button.pressed.connect(self.right_button_event) #Extra defined click event
        #self.right_button.setAutoRepeat(True) #Extra defined to keep going in the event if held down
        #self.right_button.setAutoRepeatInterval(10) #Extra defined to repeat 10ms
        self.right_button.released.connect(self.arlo_button_stop)
        self.left_right_button_grid.addWidget(self.right_button)
        spacerItem9 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.left_right_button_grid.addItem(spacerItem9)
        self.vertical_grid.addLayout(self.left_right_button_grid)
        self.backward_button_grid = QtGui.QHBoxLayout()
        self.backward_button_grid.setObjectName(_fromUtf8("backward_button_grid"))
        self.backward_button = QtGui.QPushButton(main_window)
        self.backward_button.setMinimumSize(QtCore.QSize(200, 100))
        self.backward_button.setMaximumSize(QtCore.QSize(200, 100))
        self.backward_button.setStyleSheet(_fromUtf8("#backward_button\n"
"{\n"
"    background-color: transparent;\n"
"    border-image: url(:/resources/bottomarrow_normal.png);\n"
"    background: none;\n"
"    border: none;\n"
"    background-repeat: none;\n"
"}\n"
"#backward_button:pressed\n"
"{\n"
"    border-image: url(:/resources/bottomarrow_pressed.png);\n"
"}\n"
"#backward_button:focus\n"
"{\n"
"    outline: none;\n"
"}"))
        self.backward_button.setText(_fromUtf8(""))
        self.backward_button.setObjectName(_fromUtf8("backward_button"))
        self.backward_button.pressed.connect(self.backward_button_event) #Extra defined click event
        #self.backward_button.setAutoRepeat(True) #Extra defined to keep going in the event if held down
        #self.backward_button.setAutoRepeatInterval(10) #Extra defined to repeat 10ms
        self.backward_button.released.connect(self.arlo_button_stop) #Extra defined function to send stop signal
        self.backward_button_grid.addWidget(self.backward_button)
        self.vertical_grid.addLayout(self.backward_button_grid)
        spacerItem10 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem10)
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
        self.speed_slider.setMaximum(100)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setObjectName(_fromUtf8("speed_slider"))
        self.speed_slider.valueChanged.connect(self.speed_slider_event) #Extra line for value change event for text label
        self.vertical_grid.addWidget(self.speed_slider)
        self.speed_label = QtGui.QLabel(main_window)
        self.speed_label.setAlignment(QtCore.Qt.AlignCenter)
        self.speed_label.setObjectName(_fromUtf8("speed_label"))
        self.vertical_grid.addWidget(self.speed_label)
        spacerItem11 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem11)
        self.battery_bar = QtGui.QProgressBar(main_window)
        self.battery_bar.setProperty("value", 0)
        self.battery_bar.setObjectName(_fromUtf8("battery_bar"))
        self.vertical_grid.addWidget(self.battery_bar)
        self.battery_label = QtGui.QLabel(main_window)
        self.battery_label.setAlignment(QtCore.Qt.AlignCenter)
        self.battery_label.setObjectName(_fromUtf8("battery_label"))
        self.vertical_grid.addWidget(self.battery_label)
        self.main_grid.addLayout(self.vertical_grid)
        self.verticalLayout.addLayout(self.main_grid)

        self.retranslateUi(main_window)
        QtCore.QMetaObject.connectSlotsByName(main_window)

        self.initros(main_window) #Initialize ROS 

        '''Initialize Xbee device
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        self.ser.write("rst")'''



    def retranslateUi(self, main_window):
        main_window.setWindowTitle(_translate("main_window", "ARM Controller", None))
        self.armstrong_controller_label.setText(_translate("main_window", "\"Arm\"strong Controller", None))
        self.gripper_label.setText(_translate("main_window", "Gripper", None))
        self.gripper_value_label.setText(_translate("main_window", "Close: " + str(self.gripper_slider.value()) + "%", None))
        self.joint1_label.setText(_translate("main_window", "Joint1", None))
        self.joint1_value_label.setText(_translate("main_window", "0.0", None))
        self.joint2_label.setText(_translate("main_window", "Joint2", None))
        self.joint2_value_label.setText(_translate("main_window", "0.0", None))
        self.joint3_label.setText(_translate("main_window", "Joint3", None))
        self.joint3_value_label.setText(_translate("main_window", "0.0", None))
        self.joint4_label.setText(_translate("main_window", "Joint4", None))
        self.joint4_value_label.setText(_translate("main_window", "0.0", None))
        self.joint5_label.setText(_translate("main_window", "Joint5", None))
        self.joint5_value_label.setText(_translate("main_window", "0.0", None))
        self.play_saved_state_button.setText(_translate("main_window", "Play State", None))
        self.save_joint_state_button.setText(_translate("main_window", "Save Joint States", None))
        self.arlo_controller_label.setText(_translate("main_window", "ARLO Controller", None))
        self.speed_label.setText(_translate("main_window", "Speed: " + str(self.speed_slider.value()) + "%", None))
        self.battery_label.setText(_translate("main_window", "Battery Level: 0V", None))

    #####################################################################################Additional Functions#########################################################################################
    def battery_progress(self,battery):
        float_battery = float(battery)
        actual_battery = float("{0:.1f}".format(float_battery)) - 0.2 #0.2 higher than recorded voltmeter
        percent = ((actual_battery - 10.0) / 2) * 100 #Viable range of only 10-12V
        self.battery_bar.setProperty("value", int(percent))
        self.battery_label.setText(_translate("main_window", "Battery Level: " + str(actual_battery) + "V", None))


    def initros(self, window):
    	self.pub = rospy.Publisher('arlo_wheels', String, queue_size=10)
    	rospy.init_node('Motor_Control_Talker', anonymous=True)
        self.battery_thread = ThreadClass() #Start thread for constantly getting the battery level
        self.battery_thread.start()
        window.connect(self.battery_thread, QtCore.SIGNAL("Battery_Value"), self.battery_progress) #Now connect main window thread for receiving that value
        self.pub.publish("rst")


    def arlo_speed_convert(self, speed):
    	'''0 to 200 encoder counts per second'''
    	if (speed > 0):
    		#Might have to convert to int 
    		arlo_speed = (200 * (speed * 0.01))
    	else:
    		arlo_speed = 0

    	return arlo_speed

    def forward_button_event(self):
    	arlo_speed = int(self.arlo_speed_convert(self.speed_slider.value()))
    	self.pub.publish("gospd " + str(arlo_speed) + " " + str(arlo_speed) + "\r")
 
    	#self.ser.write("go " + str(arlo_speed) + " " + str(arlo_speed))


    def backward_button_event(self):
    	arlo_speed = int(-1 * self.arlo_speed_convert(self.speed_slider.value()))
    	self.pub.publish("gospd " + str(arlo_speed) + " " + str(arlo_speed) + "\r")

    	#self.ser.write("go " + str(arlo_speed) + " " + str(arlo_speed))


    def left_button_event(self):
    	arlo_speed = int(self.arlo_speed_convert(self.speed_slider.value()))
    	self.pub.publish("gospd " + str(-1*arlo_speed) + " " + str(arlo_speed) + "\r")

    	#self.ser.write("go " + str(-1*arlo_speed) + " " + str(arlo_speed))


    def right_button_event(self):
    	arlo_speed = int(self.arlo_speed_convert(self.speed_slider.value()))
    	self.pub.publish("gospd " + str(arlo_speed) + " " + str(-1*arlo_speed) + "\r")

    	#self.ser.write("go " + str(arlo_speed) + " " + str(-1*arlo_speed))

    def arlo_button_stop(self):
    	self.pub.publish("go 0 0\r")
        #self.pub.publish("rst")
    	#self.ser.write("go 0 0")

    def speed_slider_event(self):
    	self.speed_label.setText(_translate("main_window", "Speed: " + str(self.speed_slider.value()) +"%", None))

    def gripper_slider_event(self):
    	pass

    def joint1_slider_event(self):
    	pass

    def joint2_slider_event(self):
    	pass

    def joint3_slider_event(self):
    	pass

    def joint4_slider_event(self):
    	pass

    def joint5_slider_event(self):
    	pass

class ThreadClass(QtCore.QThread):
    def __init__(self, parent = None):
        super(ThreadClass, self).__init__(parent)

    def run(self):
        while 1:
            voltage = rospy.get_param("/battery_level")
            self.emit(QtCore.SIGNAL('Battery_Value'),voltage)
            
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    app.setStyle("GTK+")
    main_window = QtGui.QWidget()
    ui = Ui_main_window()
    ui.setupUi(main_window)
    main_window.show()
    sys.exit(app.exec_())

