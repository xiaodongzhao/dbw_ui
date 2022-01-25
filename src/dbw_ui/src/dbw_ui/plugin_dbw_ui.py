#!/usr/bin/env python
from __future__ import division, print_function

import os
import json
import rospkg
import rospy
from python_qt_binding import QtCore, QtWidgets, loadUi
from qt_gui.plugin import Plugin

from QtWidgets import QWidget
from QtGui import QPalette, QColor

from dbw_mkz_msgs.msg import TurnSignalCmd, GearCmd, GearReport, ThrottleCmd, ThrottleReport, ThrottleInfoReport, BrakeCmd, BrakeInfoReport, BrakeReport, SteeringCmd, SteeringReport
from dataspeed_ulc_msgs.msg import UlcReport, UlcCmd
from std_msgs.msg import Bool, Empty


class DbwUiPlugin(Plugin):
    def __init__(self, context):
        super(DbwUiPlugin, self).__init__(context)

        name = "dbw_ui"
        self.setObjectName(name)

        # setup UI
        self.resource_folder = os.path.join(
            rospkg.RosPack().get_path('dbw_ui'), 'resource')
        ui_file = os.path.join(self.resource_folder, 'dbw_ui.ui')
        self._window = QWidget()
        loadUi(ui_file, self._window)
        self._window.setObjectName(name + 'UI')
        self._window.setWindowTitle(name)
        context.add_widget(self._window)  # Add widget to the user interface

        #  multiple instances
        if context.serial_number() > 1:
            self._window.setWindowTitle(self._window.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # constant values used in DBW ROS messages
        self.msg_constants = {}
        with open(os.path.join(self.resource_folder,
                               "msg_constants.json")) as f:
            constants = json.load(f)
            for k1, v1 in constants.iteritems():  # reverse key, value
                self.msg_constants[k1] = {}
                for k2, v2 in v1.iteritems():
                    self.msg_constants[k1][v2] = k2

        # sub and pub ROS messages
        ns = "/vehicle"
        self.cmd_started_status = {}
        self.report_subs = {}
        self.cmd_pubs = {}
        for name in ["Ulc", "Gear", "Throttle", "Brake", "Steering"]:
            name_lower = name.lower()
            report = "%sReport" % name
            cmd = "%sCmd" % name
            self.report_subs[report] = rospy.Subscriber(
                ns + "/%s_report" % name_lower,
                globals()[report], self.report_cb)
            self.cmd_pubs[cmd] = rospy.Publisher(ns + "/%s_cmd" % name_lower,
                                                 globals()[cmd],
                                                 queue_size=1)
            getattr(self._window, "%s_start" % cmd).clicked.connect(
                lambda checked, source=cmd: self.cmd_start(source))
            if name in ["Throttle", "Brake"]:
                self.report_subs["%sInfo" % name] = rospy.Subscriber(
                    ns + "/%s_info_report" % name_lower,
                    globals()["%sInfoReport" % name], self.report_cb)

        # input range change
        self._window.BrakeCmd_pedal_cmd_type.currentIndexChanged[int].connect(
            self.brake_cmd_type_changed)
        self._window.ThrottleCmd_pedal_cmd_type.currentIndexChanged[
            int].connect(self.throttle_cmd_type_changed)

        # turn signal
        self.cmd_pubs["TurnSignalCmd"] = rospy.Publisher(ns +
                                                         "/turn_signal_cmd",
                                                         TurnSignalCmd,
                                                         queue_size=1)
        self._window.TurnSignalCmd_start.clicked.connect(
            lambda checked, source="TurnSignalCmd": self.cmd_start(source))

        # dbw
        self.dbw_enabled_status = False
        self.dbw_enable_pub = rospy.Publisher(ns + "/enable",
                                              Empty,
                                              queue_size=1,
                                              latch=True)
        self.dbw_disable_pub = rospy.Publisher(ns + "/disable",
                                               Empty,
                                               queue_size=1,
                                               latch=True)
        self.dbw_enabled_sub = rospy.Subscriber(ns + "/dbw_enabled", Bool,
                                                self.dbw_enabled_cb)
        self._window.dbw_enable.clicked.connect(self.dbw_enable)

        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_cb)

    def brake_cmd_type_changed(self, index):
        min_max_map = {
            1: (0.15, 0.5),
            2: (0, 1),
            3: (0, 3412),
            4: (0, 3412),
            6: (0, 10)
        }
        min_input, max_input = min_max_map.get(index, (0, 0))
        self._window.BrakeCmd_pedal_cmd.setMinimum(min_input)
        self._window.BrakeCmd_pedal_cmd.setMaximum(max_input)
        self._window.BrakeCmd_pedal_cmd.setValue(min_input)

    def throttle_cmd_type_changed(self, index):
        min_max_map = {1: (0.15, 0.8), 2: (0, 1)}
        min_input, max_input = min_max_map.get(index, (0, 0))
        self._window.ThrottleCmd_pedal_cmd.setMinimum(min_input)
        self._window.ThrottleCmd_pedal_cmd.setMaximum(max_input)
        self._window.ThrottleCmd_pedal_cmd.setValue(min_input)

    def shutdown_plugin(self):
        self.timer.shutdown()
        for topic, sub in self.report_subs.iteritems():
            sub.unregister()

    def timer_cb(self, event):
        for cmd, status in self.cmd_started_status.iteritems():
            if status:
                self.cmd_send(cmd)

    def report_cb(self, msg):
        msg_full_type = msg._type
        msg_type = msg_full_type.split('/')[1]
        for i, sig_name in enumerate(msg.__slots__):
            if sig_name == "header":
                continue
            sig_type = msg._slot_types[i]
            sig_full_name = "%s/%s" % (msg_full_type, sig_name)
            sig_value = getattr(msg, sig_name)
            try:
                widget = getattr(self._window, "%s_%s" % (msg_type, sig_name))
                if sig_type == "bool":
                    widget.setChecked(sig_value)
                elif sig_type == "float32":
                    widget.setText(str(sig_value))
                elif sig_type == "uint8" and sig_full_name in self.msg_constants:
                    widget.setText(
                        "%s(%s)" %
                        (sig_value,
                         self.msg_constants[sig_full_name].get(sig_value)))
                else:  # check for nested msg, only one nested layer
                    text = []
                    for sub_sig_name in sig_value.__slots__:
                        sub_sig_full_name = "%s/%s" % (sig_type, sub_sig_name)
                        sub_sig_value = getattr(sig_value, sub_sig_name)
                        text.append("%s:%s(%s)" % (sub_sig_name, sub_sig_value, self.msg_constants[sub_sig_full_name].get(sub_sig_value)))
                    widget.setText("\n".join(text))
            except (KeyError, AttributeError), e:
                rospy.logwarn(
                    "report_cb does not know how to handle signal %s(%s):%s"
                    % (sig_name, sig_type, e))
            except RuntimeError:  # error when PyQt is closed
                return

    def cmd_send(self, cmd):
        msg = globals()[cmd]()
        for i, sig_name in enumerate(msg.__slots__):
            if sig_name == "header":
                continue
            sig_type = msg._slot_types[i]
            try:
                widget = getattr(self._window, "%s_%s" % (cmd, sig_name))
                if sig_type == "float32" or sig_type == "float64":
                    setattr(msg, sig_name, widget.value())
                elif sig_type == "bool":
                    setattr(msg, sig_name, widget.isChecked())
                elif sig_type == "uint8":
                    if isinstance(widget, QtWidgets.QComboBox):
                        setattr(msg, sig_name, widget.currentIndex())
                    else:
                        setattr(msg, sig_name, widget.value())
                else:  # check for nested msg, only one nested layer
                    try:
                        sig_value = getattr(msg, sig_name)
                        sub_sig_name = sig_value.__slots__[0]
                        if isinstance(widget, QtWidgets.QComboBox):
                            setattr(sig_value, sub_sig_name,
                                    widget.currentIndex())
                        else:
                            setattr(sig_value, sub_sig_name, widget.value())
                    except (KeyError, AttributeError), e:
                        rospy.logwarn(
                            "cmd_send does not know handle %s(%s): %s" %
                            (sig_name, sig_type, e))
            except RuntimeError:  # error when PyQt is closed
                return
            except AttributeError:
                rospy.logwarn("not supported signal %s in %s" % (sig_name, cmd))
                continue
        self.cmd_pubs[cmd].publish(msg)

    def set_widget_color(self, widget, color):
        palette = widget.palette()
        palette.setColor(QPalette.Button, QColor(color))
        widget.setPalette(palette)
        widget.update()

    def cmd_start(self, cmd):
        widget = getattr(self._window, "%s_start" % cmd)
        if self.cmd_started_status.get(cmd):
            widget.setText("Start %s" % cmd)
            self.set_widget_color(widget, QtCore.Qt.white)
            self.cmd_started_status[cmd] = False
            rospy.loginfo("Stopped %s" % cmd)
        else:
            widget.setText("Stop %s" % cmd)
            self.set_widget_color(widget, QtCore.Qt.red)
            self.cmd_started_status[cmd] = True
            rospy.loginfo("Started %s" % cmd)

    def dbw_enable(self):
        e = Empty()
        if self.dbw_enabled_status:
            self.dbw_disable_pub.publish(e)
            rospy.loginfo("Requested to disable drive-by-wire")
        else:
            self.dbw_enable_pub.publish(e)
            rospy.loginfo("Requested to enable drive-by-wire")

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled_status = msg.data
        if self.dbw_enabled_status:
            self._window.dbw_enable.setText("Disable drive-by-wire")
            self.set_widget_color(self._window.dbw_enable, QtCore.Qt.red)
            rospy.loginfo("Drive-by-wire enabled")
        else:
            self._window.dbw_enable.setText("Enable drive-by-wire")
            self.set_widget_color(self._window.dbw_enable, QtCore.Qt.white)
            rospy.loginfo("Drive-by-wire disabled")
