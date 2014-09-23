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

import rospy
from controller_manager_msgs.srv import *


class ControllerManagerDummy:
    """
    Dummy controller manager instance.

    Creates the expected controller manager ROS interface, with a simple
    default behavior that can be overridden by modifying its members.
    """
    def __init__(self, ns="/controller_manager"):
        # Controller manager namespace
        cm_ns = ns
        if not cm_ns or cm_ns[-1] != '/':
            cm_ns += '/'

        # Service return values (reset if a specific behavior is desired)
        self.list_ctrl_resp = ListControllersResponse()
        self.list_types_resp = ListControllerTypesResponse()
        self.load_ctrl_resp = LoadControllerResponse(ok=True)
        self.unload_ctrl_resp = UnloadControllerResponse(ok=True)
        self.switch_ctrl_resp = SwitchControllerResponse(ok=True)
        self.reload_libs_resp = ReloadControllerLibrariesResponse(ok=True)

        # Service proxies
        self.list_ctrl = rospy.Service(cm_ns + 'list_controllers',
                                       ListControllers,
                                       self._list_ctrl_cb)
        self.list_types = rospy.Service(cm_ns + 'list_controller_types',
                                        ListControllerTypes,
                                        self._list_types_cb)
        self.load_ctrl = rospy.Service(cm_ns + 'load_controller',
                                       LoadController,
                                       self._load_ctrl_cb)
        self.unload_ctrl = rospy.Service(cm_ns + 'unload_controller',
                                         UnloadController,
                                         self._unload_ctrl_cb)
        self.switch_ctrl = rospy.Service(cm_ns + 'switch_controller',
                                         SwitchController,
                                         self._switch_ctrl_cb)
        self.reload_libs = rospy.Service(cm_ns + 'reload_controller_libraries',
                                         ReloadControllerLibraries,
                                         self._reload_libs_cb)

    def _list_ctrl_cb(self, req):
        return self.list_ctrl_resp

    def _list_types_cb(self, req):
        return self.list_types_resp

    def _load_ctrl_cb(self, req):
        return self.load_ctrl_resp

    def _unload_ctrl_cb(self, req):
        return self.unload_ctrl_resp

    def _switch_ctrl_cb(self, req):
        return self.switch_ctrl_resp

    def _reload_libs_cb(self, req):
        return self.reload_libs_resp
