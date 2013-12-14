import python_qt_binding.QtCore as QtCore
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QHeaderView, QIcon, QMenu, QTreeWidgetItem, QWidget

from qt_gui.plugin import Plugin

import rospy
import rosservice
import rosparam
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
    _column_names_pretty = ['Controller Name', 'State', 'Type', 'HW Interface', 'Claimed Resources']

    sig_sysmsg = Signal(str)
    def __init__(self, plugin):
        super(ControllerManagerWidget, self).__init__()

        self._plugin = plugin
        self.setWindowTitle('Controller Manager')

        # create layouts
        vlayout_outer = QtGui.QVBoxLayout(self)
        vlayout_outer.setObjectName('vert_layout_outer')
        hlayout_top = QtGui.QHBoxLayout(self)
        hlayout_top.setObjectName('hori_layout_top')
        vlayout_outer.addLayout(hlayout_top)

        # create top bar
        # controller manager namespace combo box & label
        cm_ns_label = QtGui.QLabel(self)
        cm_ns_label.setObjectName('cm_ns_label')
        cm_ns_label.setText('CM Namespace:')
        fixed_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed)
        cm_ns_label.setSizePolicy(fixed_policy)
        hlayout_top.addWidget(cm_ns_label)
        cm_namespace_combo = QtGui.QComboBox(self)
        cm_namespace_combo.setObjectName('cm_namespace_combo')
        hlayout_top.addWidget(cm_namespace_combo)
        self.cm_namespace_combo = cm_namespace_combo

        # load controller combo box & label
        load_ctrl_label = QtGui.QLabel(self)
        load_ctrl_label.setObjectName('load_ctrl_label')
        load_ctrl_label.setText('Load Controller:')
        fixed_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed)
        load_ctrl_label.setSizePolicy(fixed_policy)
        hlayout_top.addWidget(load_ctrl_label)
        load_ctrl_combo = QtGui.QComboBox(self)
        load_ctrl_combo.setObjectName('load_ctrl_combo')
        load_ctrl_size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        load_ctrl_combo.setSizePolicy(load_ctrl_size_policy)
        hlayout_top.addWidget(load_ctrl_combo)
        self.load_ctrl_combo = load_ctrl_combo

        # load control button
        load_ctrl_button = QtGui.QPushButton(self)
        load_ctrl_button.setObjectName('load_ctrl_button')
        button_size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        button_size_policy.setHorizontalStretch(0)
        button_size_policy.setVerticalStretch(0)
        button_size_policy.setHeightForWidth(load_ctrl_button.sizePolicy().hasHeightForWidth())
        load_ctrl_button.setSizePolicy(button_size_policy)
        load_ctrl_button.setBaseSize(QtCore.QSize(30, 30))
        load_ctrl_button.setIcon(QIcon.fromTheme('list-add'))
        load_ctrl_button.setIconSize(QtCore.QSize(20,20))
        load_ctrl_button.clicked.connect(self.load_cur_ctrl)
        hlayout_top.addWidget(load_ctrl_button)

        # start control button
        start_ctrl_button = QtGui.QPushButton(self)
        start_ctrl_button.setObjectName('start_ctrl_button')
        button_size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        button_size_policy.setHorizontalStretch(0)
        button_size_policy.setVerticalStretch(0)
        button_size_policy.setHeightForWidth(start_ctrl_button.sizePolicy().hasHeightForWidth())
        start_ctrl_button.setSizePolicy(button_size_policy)
        start_ctrl_button.setBaseSize(QtCore.QSize(30, 30))
        start_ctrl_button.setIcon(QIcon.fromTheme('media-playback-start'))
        start_ctrl_button.setIconSize(QtCore.QSize(20,20))
        start_ctrl_button.clicked.connect(self.start_cur_ctrl)
        hlayout_top.addWidget(start_ctrl_button)

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
        self.list_types = {}
        self.list_ctrlers = {}
        self.load_ctrler = {}
        self.unload_ctrler = {}
        self.switch_ctrlers = {}
        self.ctrlman_ns_cur = ''
        self.loadable_params = {}

        # init and start update timer
        self._timer_refresh_ctrlers = QTimer(self)
        self._timer_refresh_ctrlers.timeout.connect(self._refresh_ctrlers_cb)

    def controller_manager_connect(self, ctrlman_ns):
        self.list_types[ctrlman_ns] = rospy.ServiceProxy(
                ctrlman_ns + '/controller_manager/list_controller_types', ListControllerTypes)
        self.list_ctrlers[ctrlman_ns] = rospy.ServiceProxy(
                ctrlman_ns + '/controller_manager/list_controllers', ListControllers)
        self.load_ctrler[ctrlman_ns] = rospy.ServiceProxy(
                ctrlman_ns + '/controller_manager/load_controller', LoadController)
        self.unload_ctrler[ctrlman_ns] = rospy.ServiceProxy(
                ctrlman_ns + '/controller_manager/unload_controller', UnloadController)
        self.switch_ctrlers[ctrlman_ns] = rospy.ServiceProxy(
                ctrlman_ns + '/controller_manager/switch_controller', SwitchController)
        self.cm_namespace_combo.addItem(ctrlman_ns)

    def controller_manager_disconnect(self, ctrlman_ns):
        self.list_types[ctrlman_ns].close()
        del self.list_types[ctrlman_ns]
        self.list_ctrlers[ctrlman_ns].close()
        del self.list_ctrlers[ctrlman_ns]
        self.load_ctrler[ctrlman_ns].close()
        del self.load_ctrler[ctrlman_ns]
        self.unload_ctrler[ctrlman_ns].close()
        del self.unload_ctrler[ctrlman_ns]
        self.switch_ctrlers[ctrlman_ns].close()
        del self.switch_ctrlers[ctrlman_ns]
        combo_ind = self.cm_namespace_combo.findText(ctrlman_ns)
        self.cm_namespace_combo.removeItem(combo_ind)

    def _refresh_ctrlers_cb(self):
        try:
            # refresh the list of controller managers we can find
            srv_list = rosservice.get_service_list()
            ctrlman_ns_list_cur = []
            for srv_name in srv_list:
                if 'controller_manager/list_controllers' in srv_name:
                    srv_type = rosservice.get_service_type(srv_name)
                    if srv_type == 'controller_manager_msgs/ListControllers':
                        ctrlman_ns = srv_name.split('/controller_manager/list_controllers')[0]
                        if ctrlman_ns == '':
                            ctrlman_ns = '/'
                        # ctrlman_ns is a Controller Manager namespace
                        if ctrlman_ns not in self.list_ctrlers:
                            # we haven't connected to it yet, create the service proxies
                            self.controller_manager_connect(ctrlman_ns)
                        ctrlman_ns_list_cur.append(ctrlman_ns)

            # remove every controller manager which isn't up anymore
            for ctrlman_ns_old in self.list_ctrlers.keys():
                if ctrlman_ns_old not in ctrlman_ns_list_cur:
                    self.controller_manager_disconnect(ctrlman_ns_old)

            # refresh the controller list for the current controller manager
            self.refresh_loadable_ctrlers()
            self.refresh_ctrlers()
        except Exception as e:
            self.sig_sysmsg.emit(e.message)

    def remove_ctrler_from_list(self, ctrler_name):
        item = self._ctrlers[ctrler_name]['item']
        index = self.ctrl_list_tree_widget.indexOfTopLevelItem(item)
        self.ctrl_list_tree_widget.takeTopLevelItem(index)
        del self._ctrlers[ctrler_name]

    def remove_loadable_from_list(self, load_text):
        load_ctrl_ind = self.load_ctrl_combo.findText(load_text)
        self.load_ctrl_combo.removeItem(load_ctrl_ind)
        del self.loadable_params[load_text]

    def refresh_loadable_ctrlers(self):
        if self.cm_namespace_combo.count() == 0:
            # no controller managers found so there are no loadable controllers
            # remove old loadables
            for old_loadable_text in self.loadable_params.keys():
                self.remove_loadable_from_list(old_loadable_text)
            return

        ctrlman_ns = self.cm_namespace_combo.currentText()

        if self.ctrlman_ns_cur != ctrlman_ns:
            # new controller manager selected
            # remove old loadables from list from last CM
            for old_loadable_text in self.loadable_params.keys():
                self.remove_loadable_from_list(old_loadable_text)

        rospy.wait_for_service(ctrlman_ns + '/controller_manager/list_controller_types', 0.2)
        try:
            resp = self.list_types[ctrlman_ns].call(ListControllerTypesRequest())
        except rospy.ServiceException as e:
            # TODO: display warning somehow
            return 
        ctrler_types = resp.types
        loadable_params_cur = []
        all_params = rosparam.list_params('/')
        # for every parameter
        for pname in all_params:
            # remove the controller manager namespace
            if ctrlman_ns == '/':
                pname_sub = pname
            else:
                pname_sub = pname[len(ctrlman_ns):]
            psplit = pname_sub.split('/')
            if len(psplit) > 2 and psplit[2] == 'type':
                loadable_type = rosparam.get_param(pname)
                if loadable_type in ctrler_types:
                    load_text = pname[:-5] + '  -  ' + loadable_type
                    loadable_params_cur.append(load_text)
                    if load_text not in self.loadable_params:
                        self.loadable_params[load_text] = psplit[1]
                        self.load_ctrl_combo.addItem(load_text)

        # remove loadable parameters no longer in the parameter server
        for load_text_old in self.loadable_params.keys():
            if load_text_old not in loadable_params_cur:
                self.remove_loadable_from_list(load_text_old)

    @Slot()
    def refresh_ctrlers(self):
        if self.cm_namespace_combo.count() == 0:
            # no controller managers found so there are no controllers to update
            # remove old controllers
            for old_ctrler_name in self._ctrlers.keys():
                self.remove_ctrler_from_list(old_ctrler_name)
            return

        ctrlman_ns = self.cm_namespace_combo.currentText()

        if self.ctrlman_ns_cur != ctrlman_ns:
            # new controller manager selected

            # remove old controllers from list from last CM
            for old_ctrler_name in self._ctrlers.keys():
                self.remove_ctrler_from_list(old_ctrler_name)
            self.ctrlman_ns_cur = ctrlman_ns

        rospy.wait_for_service(ctrlman_ns + '/controller_manager/list_controllers', 0.2)
        try:
            resp = self.list_ctrlers[ctrlman_ns].call(ListControllersRequest())
        except rospy.ServiceException as e:
            # TODO: display warning somehow
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
                self.remove_ctrler_from_list(old_ctrler_name)
                
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

    def load_cur_ctrl(self):
        ctrlman_ns = self.cm_namespace_combo.currentText()
        load_text = self.load_ctrl_combo.currentText()
        load_param = self.loadable_params[load_text]
        self.load_controller(ctrlman_ns, load_param)

    def start_cur_ctrl(self):
        ctrlman_ns = self.cm_namespace_combo.currentText()
        load_text = self.load_ctrl_combo.currentText()
        load_param = self.loadable_params[load_text]
        self.load_controller(ctrlman_ns, load_param)
        self.start_stop_controller(ctrlman_ns, load_param, True)

    @Slot('QPoint')
    def on_ctrl_list_tree_widget_customContextMenuRequested(self, pos):
        ctrlman_ns = self.cm_namespace_combo.currentText()
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
                    QIcon.fromTheme('list-remove'), 'Stop and Unload Controller')
        elif ctrl_state == 'stopped':
            action_start = menu.addAction(
                    QIcon.fromTheme('media-playback-start'), 'Start Controller')
            action_unload = menu.addAction(
                    QIcon.fromTheme('list-remove'), 'Unload Controller')

        action = menu.exec_(self.ctrl_list_tree_widget.mapToGlobal(pos))

        # evaluate user action
        if ctrl_state == 'running':
            if action is action_stop:
                self.start_stop_controller(ctrlman_ns, ctrl_name, False)
            elif action is action_kill:
                self.start_stop_controller(ctrlman_ns, ctrl_name, False)
                self.unload_controller(ctrlman_ns, ctrl_name)
        elif ctrl_state == 'stopped':
            if action is action_start:
                self.start_stop_controller(ctrlman_ns, ctrl_name, True)
            elif action is action_unload:
                self.unload_controller(ctrlman_ns, ctrl_name)

    def load_controller(self, ctrlman_ns, name):
        rospy.wait_for_service(ctrlman_ns + '/controller_manager/load_controller', 0.2)
        try:
            resp = self.load_ctrler[ctrlman_ns].call(LoadControllerRequest(name))
            if resp.ok == 1:
                # successful
                return 0
            else:
                # failure
                return 1
        except rospy.ServiceException as e:
            # exception
            return 2

    def unload_controller(self, ctrlman_ns, name):
        rospy.wait_for_service(ctrlman_ns + '/controller_manager/unload_controller', 0.2)
        try:
            resp = self.unload_ctrler[ctrlman_ns].call(UnloadControllerRequest(name))
            if resp.ok == 1:
                # successful
                return 0
            else:
                # failure
                return 1
        except rospy.ServiceException as e:
            # exception
            return 2

    def start_stop_controller(self, ctrlman_ns, name, is_start):
        rospy.wait_for_service(ctrlman_ns + '/controller_manager/switch_controller', 0.2)
        start = []
        stop = []
        strictness = SwitchControllerRequest.STRICT
        if is_start:
            start = [name]
        else:
            stop = [name]
        try:
            resp = self.switch_ctrlers[ctrlman_ns].call(
                        SwitchControllerRequest(start, stop, strictness))
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
        for ctrlman_ns in self.list_ctrlers.keys():
            self.controller_manager_disconnect(ctrlman_ns)
        self._timer_refresh_ctrlers.stop()

