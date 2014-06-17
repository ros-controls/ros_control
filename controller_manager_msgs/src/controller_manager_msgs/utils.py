#!/usr/bin/env python

# Copyright (C) 2014, PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of PAL Robotics S.L. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# NOTE: The Python API contained in this file is considered UNSTABLE and
# subject to change.
# No backwards compatibility guarrantees are provided at this moment.


###############################################################################
#
# Convenience methods for finding controller managers
#
###############################################################################

import rosservice
from rospy import ServiceProxy
from controller_manager_msgs.srv import *

# Names of controller manager services, and their respective types
_LIST_CONTROLLERS_STR = 'list_controllers'
_LIST_CONTROLLERS_TYPE = 'controller_manager_msgs/ListControllers'
_LIST_CONTROLLER_TYPES_STR = 'list_controller_types'
_LIST_CONTROLLER_TYPES_TYPE = 'controller_manager_msgs/ListControllerTypes'
_LOAD_CONTROLLER_STR = 'load_controller'
_LOAD_CONTROLLER_TYPE = 'controller_manager_msgs/LoadController'
_UNLOAD_CONTROLLER_STR = 'unload_controller'
_UNLOAD_CONTROLLER_TYPE = 'controller_manager_msgs/UnloadController'
_SWITCH_CONTROLLER_STR = 'switch_controller'
_SWITCH_CONTROLLER_TYPE = 'controller_manager_msgs/SwitchController'
_RELOAD_CONTROLLER_LIBS_STR = 'reload_controller_libraries'
_RELOAD_CONTROLLER_LIBS_TYPE = ('controller_manager_msgs/'
                                'ReloadControllerLibraries')

# Map from service names to their respective type
cm_services = {
    _LIST_CONTROLLERS_STR:       _LIST_CONTROLLERS_TYPE,
    _LIST_CONTROLLER_TYPES_STR:  _LIST_CONTROLLER_TYPES_TYPE,
    _LOAD_CONTROLLER_STR:        _LOAD_CONTROLLER_TYPE,
    _UNLOAD_CONTROLLER_STR:      _UNLOAD_CONTROLLER_TYPE,
    _SWITCH_CONTROLLER_STR:      _SWITCH_CONTROLLER_TYPE,
    _RELOAD_CONTROLLER_LIBS_STR: _RELOAD_CONTROLLER_LIBS_TYPE
}


def get_controller_managers(namespace='/', initial_guess=None):
    """
    Get list of active controller manager namespaces.

    @param namespace: Namespace where to look for controller managers.
    @type namespace: str
    @param initial_guess: Initial guess of the active controller managers.
    Typically c{initial_guess} is the output of a previous call to this method,
    and is useful when periodically checking for changes in the list of
    active controller managers.
    Elements in this list will go through a lazy validity check (as opposed to
    a full name+type API verification), so providing a good estimate can
    significantly reduce the number of ROS master queries incurred by this
    method.
    @type initial_guess: [str]
    @return: Sorted list of active controller manager namespaces.
    @rtype: [str]
    """
    ns_list = []
    if initial_guess is not None:
        ns_list = initial_guess[:]  # force copy

    # Get list of (potential) currently running controller managers
    ns_list_curr = _sloppy_get_controller_managers(namespace)

    # Update initial guess:
    # 1. Remove entries not found in current list
    # 2. Add new untracked controller managers
    ns_list[:] = [ns for ns in ns_list if ns in ns_list_curr]
    ns_list += [ns for ns in ns_list_curr
                if ns not in ns_list and is_controller_manager(ns)]

    return sorted(ns_list)


def is_controller_manager(namespace):
    """
    Check if the input namespace exposes the controller_manager ROS interface.

    This method has the overhead of several ROS master queries
    (one per ROS API member).

    @param namespace: Namespace to check
    @type namespace: str
    @return: True if namespace exposes the controller_manager ROS interface
    @rtype: bool
    """
    cm_ns = namespace
    if not cm_ns or cm_ns[-1] != '/':
        cm_ns += '/'
    for srv_name in cm_services.keys():
        if not _srv_exists(cm_ns + srv_name, cm_services[srv_name]):
            return False
    return True


def _sloppy_get_controller_managers(namespace):
    """
    Get list of I{potential} active controller manager namespaces.

    The method name is prepended with I{sloppy}, and the returned list contains
    I{potential} active controller managers because it does not perform a
    thorough check of the expected ROS API.
    It rather tries to minimize the number of ROS master queries.

    This method has the overhead of one ROS master query.

    @param namespace: Namespace where to look for controller managers.
    @type namespace: str
    @return: List of I{potential} active controller manager namespaces.
    @rtype: [str]
    """
    try:
        # refresh the list of controller managers we can find
        srv_list = rosservice.get_service_list(namespace=namespace)
    except ROSServiceIOException:
        return []

    ns_list = []
    for srv_name in srv_list:
        match_str = '/' + _LIST_CONTROLLERS_STR
        if match_str in srv_name:
            ns = srv_name.split(match_str)[0]
            if ns == '':
                # controller manager services live in root namespace
                # unlikely, but still possible
                ns = '/'
            ns_list.append(ns)
    return ns_list


def _srv_exists(srv_name, srv_type):
    """
    Check if a ROS service of specific name and type exists.

    This method has the overhead of one ROS master query.

    @param srv_name: Fully qualified service name
    @type srv_name:  str
    @param srv_type: Service type
    @type srv_type: str
    """
    if not srv_name or not srv_type:
        return False
    return srv_type == rosservice.get_service_type(srv_name)


###############################################################################
#
# Convenience classes for querying controller managers and controllers
#
###############################################################################

class ControllerManagerLister:
    """
    Convenience functor for querying the list of active controller managers.

    Useful when frequently updating the list, as it internally performs
    some optimizations that reduce the number of interactions with the
    ROS master.

    Example usage:
        >>> list_cm = ControllerManagerLister()
        >>> print(list_cm())
    """
    def __init__(self, namespace='/'):
        self._ns = namespace
        self._cm_list = []

    def __call__(self):
        """
        Get list of running controller managers.
        """
        self._cm_list = get_controller_managers(self._ns, self._cm_list)
        return self._cm_list


class ControllerLister:
    """
    Convenience functor for querying loaded controller data.

    The output of calling this functor can be used as input to the different
    controller filtering functions available in this module.

    Example usage. Get I{running} controllers of type C{bar_base/bar}:
        >>> list_controllers = ControllerLister('foo_robot/controller_manager')
        >>> all_ctrl = list_controllers()
        >>> running_ctrl = filter_by_state(all_ctrl, 'running')
        >>> running_bar_ctrl = filter_by_type(running_ctrl, 'bar_base/bar')
    """
    def __init__(self, namespace='/controller_manager'):
        """
        @param namespace Namespace of controller manager to monitor.
        @type namespace str
        """
        self._srv_name = namespace + '/' + _LIST_CONTROLLERS_STR
        self._srv = self._make_srv()

    """
    @return: Controller list.
    @rtype: [controller_manager_msgs/ControllerState]
    """
    def __call__(self):
        try:
            controller_list = self._srv.call().controller
        except:
            # Attempt service proxy reconnection
            self._srv = self._make_srv()
            try:
                controller_list = self._srv.call().controller
            except:
                controller_list = []
        return controller_list

    def _make_srv(self):
        return ServiceProxy(self._srv_name,
                            ListControllers,
                            persistent=True)  # NOTE: For performance


###############################################################################
#
# Convenience methods for filtering controller state information
#
###############################################################################


def filter_by_name(ctrl_list, ctrl_name, match_substring=False):
    """
    Filter controller state list by controller name.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_name: Controller name
    @type ctrl_name: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified name
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, 'name', ctrl_name, match_substring)


def filter_by_type(ctrl_list, ctrl_type, match_substring=False):
    """
    Filter controller state list by controller type.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_type: Controller type
    @type ctrl_type: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified type
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, 'type', ctrl_type, match_substring)


def filter_by_state(ctrl_list, ctrl_state, match_substring=False):
    """
    Filter controller state list by controller state.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param ctrl_state: Controller state
    @type ctrl_state: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified state
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list, 'state', ctrl_state, match_substring)


def filter_by_hardware_interface(ctrl_list, hw_iface, match_substring=False):
    """
    Filter controller state list by controller hardware interface.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param hw_iface: Controller hardware interface
    @type hw_iface: str
    @param match_substring: Set to True to allow substring matching
    @type match_substring: bool
    @return: Controllers matching the specified hardware interface
    @rtype: [controller_manager_msgs/ControllerState]
    """
    return _filter_by_attr(ctrl_list,
                           'hardware_interface',
                           hw_iface,
                           match_substring)


def filter_by_resources(ctrl_list, resources, match_any=False):
    # NOTE: This method will most likely change when controllers are allowed
    # to have multiple hardware interfaces, as each hardware interface will
    # have a separate list of claimed resources
    """
    Filter controller state list by claimed resources.

    @param ctrl_list: Controller state list
    @type ctrl_list: [controller_manager_msgs/ControllerState]
    @param resources: Resources
    @type resources: str
    @param match_any: If set to False, all elements in C{resources} must
    be claimed by a controller for a positive match (note that the controller
    resource list can be contain additional entries than those in
    C{resources}).
    If set to True, at least one element in C{resources} must be claimed by a
    controller for a positive match.
    @type match_any: bool
    @return: Controllers matching the specified hardware interface
    @rtype: [controller_manager_msgs/ControllerState]
    """
    list_out = []
    for ctrl in ctrl_list:
        for res in resources:
            add_ctrl = not match_any  # Initial flag value
            if res in ctrl.resources:
                if match_any:  # One hit: enough to accept controller
                    add_ctrl = True
                    break
            else:
                if not match_any:  # One miss: enough to discard controller
                    add_ctrl = False
                    break
        if add_ctrl:
            list_out.append(ctrl)
    return list_out


def _filter_by_attr(list_in, attr_name, attr_val, match_substring=False):
    """
    Filter input list by the value of its elements' attributes.
    """
    list_out = []
    for val in list_in:
        if match_substring:
            if attr_val in getattr(val, attr_name):
                list_out.append(val)
        else:
            if getattr(val, attr_name) == attr_val:
                list_out.append(val)
    return list_out
