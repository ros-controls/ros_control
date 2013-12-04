import python_qt_binding.QtCore as QtCore
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QHeaderView, QIcon, QMenu, QTreeWidgetItem, QWidget

from qt_gui.plugin import Plugin

import rospy
from rqt_py_common.plugin_container_widget import PluginContainerWidget
from controller_manager_msgs.srv import *

class ControllerManagerGUI(Plugin):

    def __init__(self, context):
        super(ControllerManagerGUI, self).__init__(context)
        self.setObjectName('controller_manager_gui')

        self.widget = ControllerManagerWidget(self)

        self.main_widget = PluginContainerWidget(self.widget, True, False)

        self.widget.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        self.widget.shutdown_plugin()

class ControllerManagerWidget(QWidget):

    _column_names = ['name', 'state', 'type', 'hw_iface', 'resources']
    _column_names_pretty = ['Controller Name', 'State', 'Type', 'HW Interface', 'Resources']

    sig_sysmsg = Signal(str)
    def __init__(self, plugin):
        super(ControllerManagerWidget, self).__init__()

        self._plugin = plugin
        self.setWindowTitle('Controller Manager')

        vlayout_outer = QtGui.QVBoxLayout(self)
        vlayout_outer.setObjectName('vert_layout_outer')

        # create tree/list widget
        ctrl_list_tree_widget = QtGui.QTreeWidget(self)
        ctrl_list_tree_widget.setObjectName('ctrl_list_tree_widget')
        self.ctrl_list_tree_widget = ctrl_list_tree_widget
        ctrl_list_tree_widget.setColumnCount(len(self._column_names))
        ctrl_list_tree_widget.setHeaderLabels(self._column_names_pretty)
        ctrl_list_tree_widget.sortByColumn(0, Qt.AscendingOrder)
        ctrl_list_tree_widget.setContextMenuPolicy(Qt.CustomContextMenu)
        ctrl_list_tree_widget.customContextMenuRequested.connect(
                            self.on_ctrl_list_tree_widget_customContextMenuRequested)
        vlayout_outer.addWidget(ctrl_list_tree_widget)

        header = self.ctrl_list_tree_widget.header()
        header.setResizeMode(QHeaderView.ResizeToContents)
        header.customContextMenuRequested.connect(
                            self.handle_header_view_customContextMenuRequested)
        header.setContextMenuPolicy(Qt.CustomContextMenu)

        self._ctrlers = {}
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        # controller manager services
        self.list_ctrlers = rospy.ServiceProxy('controller_manager/list_controllers', 
                                               ListControllers)
        self.unload_ctrler = rospy.ServiceProxy('controller_manager/unload_controller', 
                                               UnloadController)
        self.switch_ctrlers = rospy.ServiceProxy('controller_manager/switch_controller', 
                                               SwitchController)

        # init and start update timer
        self._timer_refresh_ctrlers = QTimer(self)
        self._timer_refresh_ctrlers.timeout.connect(self._refresh_ctrlers_cb)

    def _refresh_ctrlers_cb(self):
        try:
            self.refresh_ctrlers()
        except Exception as e:
            self.sig_sysmsg.emit(e.message)

    @Slot()
    def refresh_ctrlers(self):
        rospy.wait_for_service('controller_manager/list_controllers', 0.2)
        try:
            resp = self.list_ctrlers.call(ListControllersRequest())
        except rospy.ServiceException as e:
            # TODO: display warning somehow
            controller_list = []
            return 

        controller_list = resp.controller
        new_ctrlers = {}
        for c in controller_list:
            if c.name not in self._ctrlers:
                # new controller
                item = QTreeWidgetItem(self.ctrl_list_tree_widget)
                item.setData(0, Qt.UserRole, c.name)
                ctrler = {'item' : item,
                          'state' : c.state,
                          'type' : c.type,
                          'hw_iface' : c.hardware_interface,
                          'resources' : "[" + ", ".join(c.resources) + "]"}
                ctrler['item'].setText(self._column_index['name'], c.name)
                update_type = True
                update_state = True
            else:
                # controller already in list
                ctrler = self._ctrlers[c.name]
                update_type = False
                update_state = False
                if ctrler['type'] != c.type or ctrler['hw_iface'] != c.hardware_interface:
                    # total controller change
                    ctrler['state'] = c.state
                    ctrler['type'] = c.type
                    ctrler['hw_iface'] = c.hardware_interface
                    ctrler['resources'] = "[" + ", ".join(c.resources) + "]"
                    update_type = True
                if ctrler['state'] != c.state:
                    # state change
                    ctrler['state'] = c.state
                    update_state = True

            # update entries if needed
            if update_type:
                ctrler['item'].setText(self._column_index['type'], ctrler['type'])
                ctrler['item'].setText(self._column_index['hw_iface'], ctrler['hw_iface'])
                ctrler['item'].setText(self._column_index['resources'], ctrler['resources'])
            if update_state or update_type:
                ctrler['item'].setText(self._column_index['state'], ctrler['state'])
            new_ctrlers[c.name] = ctrler

        # remove missing controllers
        for old_ctrler_name in self._ctrlers.keys():
            if old_ctrler_name not in new_ctrlers:
                item = self._ctrlers[old_ctrler_name]['item']
                index = self.ctrl_list_tree_widget.indexOfTopLevelItem(item)
                self.ctrl_list_tree_widget.takeTopLevelItem(index)
                del self._ctrlers[old_ctrler_name]
                
        # update current controllers
        self._ctrlers = new_ctrlers

    def start(self):
        self._timer_refresh_ctrlers.start(1000)

    @Slot('QPoint')
    def handle_header_view_customContextMenuRequested(self, pos):
        header = self.ctrl_list_tree_widget.header()

        # show context menu
        menu = QMenu(self)
        action_toggle_auto_resize = menu.addAction('Toggle Auto-Resize')
        action = menu.exec_(header.mapToGlobal(pos))

        # evaluate user action
        if action is action_toggle_auto_resize:
            if header.resizeMode(0) == QHeaderView.ResizeToContents:
                header.setResizeMode(QHeaderView.Interactive)
            else:
                header.setResizeMode(QHeaderView.ResizeToContents)

    @Slot('QPoint')
    def on_ctrl_list_tree_widget_customContextMenuRequested(self, pos):
        item = self.ctrl_list_tree_widget.itemAt(pos)
        if item is None:
            return
        ctrl_name = item.data(0, Qt.UserRole)
        ctrl_state = self._ctrlers[ctrl_name]['state']

        # show context menu
        menu = QMenu(self)
        if ctrl_state == 'running':
            action_stop = menu.addAction(
                    QIcon.fromTheme('media-playback-stop'), 'Stop Controller')
            action_kill = menu.addAction(
                    QIcon.fromTheme('media-eject'), 'Stop & Unload Controller')
        elif ctrl_state == 'stopped':
            action_start = menu.addAction(
                    QIcon.fromTheme('media-playback-start'), 'Start Controller')
            action_unload = menu.addAction(
                    QIcon.fromTheme('media-eject'), 'Unload Controller')

        action = menu.exec_(self.ctrl_list_tree_widget.mapToGlobal(pos))

        # evaluate user action
        if ctrl_state == 'running':
            if action is action_stop:
                self.start_stop_controller(ctrl_name, False)
            elif action is action_kill:
                self.start_stop_controller(ctrl_name, False)
                self.unload_controller(ctrl_name)
        elif ctrl_state == 'stopped':
            if action is action_start:
                self.start_stop_controller(ctrl_name, True)
            elif action is action_unload:
                self.unload_controller(ctrl_name)

    def unload_controller(self, name):
        rospy.wait_for_service('controller_manager/unload_controller', 0.2)
        try:
            resp = self.unload_ctrler.call(UnloadControllerRequest(name))
            if resp.ok == 1:
                # successful
                return 0
            else:
                # failure
                return 1
        except rospy.ServiceException as e:
            # exception
            return 2

    def start_stop_controller(self, name, is_start):
        rospy.wait_for_service('controller_manager/switch_controller', 0.2)
        start = []
        stop = []
        strictness = SwitchControllerRequest.STRICT
        if is_start:
            start = [name]
        else:
            stop = [name]
        try:
            resp = self.switch_ctrlers.call(SwitchControllerRequest(start, stop, strictness))
            if resp.ok == 1:
                # successful
                return 0
            else:
                # failure
                return 1
        except rospy.ServiceException as e:
            # exception
            return 2

    def shutdown_plugin(self):
        self.list_ctrlers.close()
        self.unload_ctrler.close()
        self.switch_ctrlers.close()
        self._timer_refresh_ctrlers.stop()

