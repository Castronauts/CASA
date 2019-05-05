#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import resources_rc
from PyQt4 import QtCore, QtGui
import serial
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Int64

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
        self.Z_Axis_Layout.setContentsMargins(0, -1, 0, -1)
        self.Z_Axis_Layout.setObjectName(_fromUtf8("Z_Axis_Layout"))
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.Z_Axis_Layout.addItem(spacerItem1)
        self.z_top_label = QtGui.QLabel(main_window)
        self.z_top_label.setAlignment(QtCore.Qt.AlignCenter)
        self.z_top_label.setObjectName(_fromUtf8("z_top_label"))
        self.Z_Axis_Layout.addWidget(self.z_top_label)
        self.z_max_up_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.z_max_up_label.setFont(font)
        self.z_max_up_label.setAlignment(QtCore.Qt.AlignCenter)
        self.z_max_up_label.setObjectName(_fromUtf8("z_max_up_label"))
        self.Z_Axis_Layout.addWidget(self.z_max_up_label)
        self.z_axis_layout_helper = QtGui.QHBoxLayout()
        self.z_axis_layout_helper.setObjectName(_fromUtf8("z_axis_layout_helper"))
        self.z_axis_bar = QtGui.QScrollBar(main_window)
        self.z_axis_bar.setEnabled(False) #----------------------------------------------------------------
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.z_axis_bar.sizePolicy().hasHeightForWidth())
        self.z_axis_bar.setSizePolicy(sizePolicy)
        self.z_axis_bar.setMinimumSize(QtCore.QSize(15, 250))
        self.z_axis_bar.setMaximumSize(QtCore.QSize(15, 250))
        self.z_axis_bar.setAccessibleName(_fromUtf8(""))
        self.z_axis_bar.setTracking(True)
        self.z_axis_bar.setOrientation(QtCore.Qt.Vertical)
        self.z_axis_bar.setInvertedAppearance(True)
        self.z_axis_bar.setObjectName(_fromUtf8("z_axis_bar"))
        self.z_axis_layout_helper.addWidget(self.z_axis_bar)
        self.Z_Axis_Layout.addLayout(self.z_axis_layout_helper)
        self.z_max_down_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.z_max_down_label.setFont(font)
        self.z_max_down_label.setAlignment(QtCore.Qt.AlignCenter)
        self.z_max_down_label.setObjectName(_fromUtf8("z_max_down_label"))
        self.Z_Axis_Layout.addWidget(self.z_max_down_label)
        self.z_bottom_label = QtGui.QLabel(main_window)
        self.z_bottom_label.setAlignment(QtCore.Qt.AlignCenter)
        self.z_bottom_label.setObjectName(_fromUtf8("z_bottom_label"))
        self.Z_Axis_Layout.addWidget(self.z_bottom_label)
        spacerItem2 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.Z_Axis_Layout.addItem(spacerItem2)
        self.z_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.z_label.setFont(font)
        self.z_label.setAlignment(QtCore.Qt.AlignCenter)
        self.z_label.setObjectName(_fromUtf8("z_label"))
        self.Z_Axis_Layout.addWidget(self.z_label)
        spacerItem3 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.Z_Axis_Layout.addItem(spacerItem3)
        self.box_sliders.addLayout(self.Z_Axis_Layout)
        self.X_Axis_Layout = QtGui.QVBoxLayout()
        self.X_Axis_Layout.setContentsMargins(0, -1, 0, -1)
        self.X_Axis_Layout.setObjectName(_fromUtf8("X_Axis_Layout"))
        spacerItem4 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.X_Axis_Layout.addItem(spacerItem4)
        self.x_left_label = QtGui.QLabel(main_window)
        self.x_left_label.setAlignment(QtCore.Qt.AlignCenter)
        self.x_left_label.setObjectName(_fromUtf8("x_left_label"))
        self.X_Axis_Layout.addWidget(self.x_left_label)
        self.x_max_left_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.x_max_left_label.setFont(font)
        self.x_max_left_label.setAlignment(QtCore.Qt.AlignCenter)
        self.x_max_left_label.setObjectName(_fromUtf8("x_max_left_label"))
        self.X_Axis_Layout.addWidget(self.x_max_left_label)
        self.x_axis_layout_helper = QtGui.QHBoxLayout()
        self.x_axis_layout_helper.setObjectName(_fromUtf8("x_axis_layout_helper"))
        self.x_axis_bar = QtGui.QScrollBar(main_window)
        self.x_axis_bar.setEnabled(False) #---------------------------------------------------------
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.x_axis_bar.sizePolicy().hasHeightForWidth())
        self.x_axis_bar.setSizePolicy(sizePolicy)
        self.x_axis_bar.setMinimumSize(QtCore.QSize(15, 250))
        self.x_axis_bar.setMaximumSize(QtCore.QSize(15, 250))
        self.x_axis_bar.setOrientation(QtCore.Qt.Vertical)
        self.x_axis_bar.setObjectName(_fromUtf8("x_axis_bar"))
        self.x_axis_layout_helper.addWidget(self.x_axis_bar)
        self.X_Axis_Layout.addLayout(self.x_axis_layout_helper)
        self.x_max_right_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.x_max_right_label.setFont(font)
        self.x_max_right_label.setAlignment(QtCore.Qt.AlignCenter)
        self.x_max_right_label.setObjectName(_fromUtf8("x_max_right_label"))
        self.X_Axis_Layout.addWidget(self.x_max_right_label)
        self.x_right_label = QtGui.QLabel(main_window)
        self.x_right_label.setAlignment(QtCore.Qt.AlignCenter)
        self.x_right_label.setObjectName(_fromUtf8("x_right_label"))
        self.X_Axis_Layout.addWidget(self.x_right_label)
        spacerItem5 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.X_Axis_Layout.addItem(spacerItem5)
        self.x_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.x_label.setFont(font)
        self.x_label.setAlignment(QtCore.Qt.AlignCenter)
        self.x_label.setObjectName(_fromUtf8("x_label"))
        self.X_Axis_Layout.addWidget(self.x_label)
        spacerItem6 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.X_Axis_Layout.addItem(spacerItem6)
        self.box_sliders.addLayout(self.X_Axis_Layout)
        self.Y_Axis_Layout = QtGui.QVBoxLayout()
        self.Y_Axis_Layout.setContentsMargins(0, -1, 0, -1)
        self.Y_Axis_Layout.setObjectName(_fromUtf8("Y_Axis_Layout"))
        spacerItem7 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.Y_Axis_Layout.addItem(spacerItem7)
        self.y_forward_label = QtGui.QLabel(main_window)
        self.y_forward_label.setAlignment(QtCore.Qt.AlignCenter)
        self.y_forward_label.setObjectName(_fromUtf8("y_forward_label"))
        self.Y_Axis_Layout.addWidget(self.y_forward_label)
        self.y_max_fwd_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.y_max_fwd_label.setFont(font)
        self.y_max_fwd_label.setTextFormat(QtCore.Qt.AutoText)
        self.y_max_fwd_label.setAlignment(QtCore.Qt.AlignCenter)
        self.y_max_fwd_label.setObjectName(_fromUtf8("y_max_fwd_label"))
        self.Y_Axis_Layout.addWidget(self.y_max_fwd_label)
        self.y_axis_layout_helper = QtGui.QHBoxLayout()
        self.y_axis_layout_helper.setObjectName(_fromUtf8("y_axis_layout_helper"))
        self.y_axis_bar = QtGui.QScrollBar(main_window)
        self.y_axis_bar.setEnabled(False) #---------------------------------------------------------
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.y_axis_bar.sizePolicy().hasHeightForWidth())
        self.y_axis_bar.setSizePolicy(sizePolicy)
        self.y_axis_bar.setMinimumSize(QtCore.QSize(15, 250))
        self.y_axis_bar.setMaximumSize(QtCore.QSize(15, 250))
        self.y_axis_bar.setTracking(True)
        self.y_axis_bar.setOrientation(QtCore.Qt.Vertical)
        self.y_axis_bar.setInvertedAppearance(True)
        self.y_axis_bar.setObjectName(_fromUtf8("y_axis_bar"))
        self.y_axis_layout_helper.addWidget(self.y_axis_bar)
        self.Y_Axis_Layout.addLayout(self.y_axis_layout_helper)
        self.y_max_back_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.y_max_back_label.setFont(font)
        self.y_max_back_label.setAlignment(QtCore.Qt.AlignCenter)
        self.y_max_back_label.setObjectName(_fromUtf8("y_max_back_label"))
        self.Y_Axis_Layout.addWidget(self.y_max_back_label)
        self.y_backward_label = QtGui.QLabel(main_window)
        self.y_backward_label.setAlignment(QtCore.Qt.AlignCenter)
        self.y_backward_label.setObjectName(_fromUtf8("y_backward_label"))
        self.Y_Axis_Layout.addWidget(self.y_backward_label)
        spacerItem8 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.Y_Axis_Layout.addItem(spacerItem8)
        self.y_label = QtGui.QLabel(main_window)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.y_label.setFont(font)
        self.y_label.setAlignment(QtCore.Qt.AlignCenter)
        self.y_label.setObjectName(_fromUtf8("y_label"))
        self.Y_Axis_Layout.addWidget(self.y_label)
        spacerItem9 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.Y_Axis_Layout.addItem(spacerItem9)
        self.box_sliders.addLayout(self.Y_Axis_Layout)
        self.vertical_grid.addLayout(self.box_sliders)
        spacerItem10 = QtGui.QSpacerItem(20, 100, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem10)
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
        spacerItem11 = QtGui.QSpacerItem(20, 100, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vertical_grid.addItem(spacerItem11)
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
        self.z_max_up_label.setText(_translate("main_window", "Max Height", None))
        self.z_max_down_label.setText(_translate("main_window", "Minimum Height", None))
        self.z_label.setText(_translate("main_window", "Z-Axis", None))
        self.x_left_label.setText(_translate("main_window", "Left Limit", None))
        self.x_right_label.setText(_translate("main_window", "Right Limit", None))
        self.x_max_left_label.setText(_translate("main_window", "Max Left", None))
        self.x_max_right_label.setText(_translate("main_window", "Max Right", None))
        self.x_label.setText(_translate("main_window", "X_Axis", None))
        self.y_forward_label.setText(_translate("main_window", "Forward Limit", None))
        self.y_backward_label.setText(_translate("main_window", "Backward Limit", None))
        self.y_max_fwd_label.setText(_translate("main_window", "Max Forward", None))
        self.y_max_back_label.setText(_translate("main_window", "Max Backward", None))
        self.y_label.setText(_translate("main_window", "Y-Axis", None))
        self.battery_level_label.setText(_translate("main_window", "Battery Level: 0.0V", None))

        #Set color of max labels to red
        self.z_max_up_label.setStyleSheet('color: red')
        self.z_max_down_label.setStyleSheet('color: red')
        self.x_max_left_label.setStyleSheet('color: red')
        self.x_max_right_label.setStyleSheet('color: red')
        self.y_max_fwd_label.setStyleSheet('color: red')
        self.y_max_back_label.setStyleSheet('color: red')

        #Hide max labels initially
        self.z_max_up_label.hide()
        self.z_max_down_label.hide()
        self.x_max_left_label.hide()
        self.x_max_right_label.hide()
        self.y_max_fwd_label.hide()
        self.y_max_back_label.hide()

    #####################################################################################Additional Functions#########################################################################################
    def battery_progress(self, battery):
        float_battery = float(battery)
        actual_battery = float("{0:.1f}".format(float_battery)) #higher than recorded voltmeter
        percent = ((actual_battery - 10.0) / 2) * 100 #Viable range of only 10-12V
        self.battery_bar.setProperty("value", int(percent))
        self.battery_level_label.setText(_translate("main_window", "Battery Level: " + str(actual_battery) + "V", None))

    def updateXBar(self, data, window):
        self.x_axis_bar.setSliderPosition(data.data)

        if(data.data >= 98): #Max right achieved
            window.emit(QtCore.SIGNAL('X_Show_Hide'), 1)

        elif (data.data <= 2): #Max left achieved
            window.emit(QtCore.SIGNAL('X_Show_Hide'), -1)

        elif (data.data < 98 and data.data > 2): #Hide objects
            window.emit(QtCore.SIGNAL('X_Show_Hide'), 0)


    def updateYBar(self, data, window):
        self.y_axis_bar.setSliderPosition(data.data)

        if(data.data >= 98): #Max forward achieved
            window.emit(QtCore.SIGNAL('Y_Show_Hide'), 1)

        elif (data.data <= 2): #Max backward achieved
            window.emit(QtCore.SIGNAL('Y_Show_Hide'), -1)

        elif (data.data < 98 and data.data > 2): #Hide objects
            window.emit(QtCore.SIGNAL('Y_Show_Hide'), 0)

    def updateZBar(self, data, window):
        self.z_axis_bar.setSliderPosition(data.data)

        if(data.data >= 98): #Max height achieved
            window.emit(QtCore.SIGNAL('Z_Show_Hide'), 1)

        elif (data.data <= 2): #Max down achieved
            window.emit(QtCore.SIGNAL('Z_Show_Hide'), -1)

        elif (data.data < 98 and data.data > 2): #Hide objects
            window.emit(QtCore.SIGNAL('Z_Show_Hide'), 0)

    def xShowHide(self, value):
        if (value == 1):
            self.x_max_left_label.hide()
            self.x_max_right_label.show()

        elif (value == -1):
            self.x_max_right_label.hide()
            self.x_max_left_label.show()

        else:
            self.x_max_left_label.hide()
            self.x_max_right_label.hide()

    def yShowHide(self, value):
        if (value == 1):
            self.y_max_fwd_label.show()
            self.y_max_back_label.hide()

        elif (value == -1):
            self.y_max_fwd_label.hide()
            self.y_max_back_label.show()

        else:
            self.y_max_fwd_label.hide()
            self.y_max_back_label.hide()

    def zShowHide(self, value):
        if (value == 1):
            self.z_max_up_label.show()
            self.z_max_down_label.hide()

        elif (value == -1):
            self.z_max_up_label.hide()
            self.z_max_down_label.show()

        else:
            self.z_max_up_label.hide()
            self.z_max_down_label.hide()

    def initros(self, window):
    	rospy.init_node('Motor_Control_Talker', anonymous=True)
        self.pub = rospy.Publisher('speed_arlo', String, queue_size=100)
        rospy.Subscriber("x_gui", Int64, self.updateXBar, window)
        rospy.Subscriber("y_gui", Int64, self.updateYBar, window)
        rospy.Subscriber("z_gui", Int64, self.updateZBar, window)
        self.battery_thread = ThreadClass() #Start thread for constantly getting the battery level
        self.battery_thread.start()
        window.connect(self.battery_thread, QtCore.SIGNAL("Battery_Value"), self.battery_progress) #Now connect main window thread for receiving that value
        window.connect(window, QtCore.SIGNAL("X_Show_Hide"), self.xShowHide) #1 show right, -1 show left, 0 hide
        window.connect(window, QtCore.SIGNAL("Y_Show_Hide"), self.yShowHide) #1 show forward, -1 show back, 0 hide
        window.connect(window, QtCore.SIGNAL("Z_Show_Hide"), self.zShowHide) #1 show height, -1 show min height, 0 hide


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
    #main_window.setEnabled(False)
    sys.exit(app.exec_())
