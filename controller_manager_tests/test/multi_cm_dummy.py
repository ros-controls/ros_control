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
from controller_manager_tests import ControllerManagerDummy
from controller_manager_msgs.msg import ControllerState as CtrlState
from controller_manager_msgs.msg import HardwareInterfaceResources
from controller_manager_msgs.srv import ListControllersResponse, LoadController

if __name__ == '__main__':
    rospy.init_node('multi_cm_dummy')

    # Valid controller managers in different namespaces
    cm_root = ControllerManagerDummy('/')
    cm_foo1 = ControllerManagerDummy('/foo/robot/controller_manager1')
    cm_foo2 = ControllerManagerDummy('/foo/robot/controller_manager2')
    cm_default = ControllerManagerDummy()

    ctrl_list = [
    CtrlState(name='foo_controller',
              state='running',
              type='foo_base/foo',
              claimed_resources=[
                  HardwareInterfaceResources(
                      hardware_interface='hardware_interface::FooInterface',
                      resources=['one', 'two', 'three'])
              ]),
    CtrlState(name='bar_controller',
              state='running',
              type='bar_base/bar',
              claimed_resources=[
                  HardwareInterfaceResources(
                      hardware_interface='hardware_interface::BarInterface',
                      resources=['four'])
              ])
    ]

    resp = ListControllersResponse()
    resp.controller = ctrl_list
    cm_default.list_ctrl_resp = resp

    # Partial controller manager ROS API: missing service
    cm_incomplete = ControllerManagerDummy('/incomplete')
    cm_incomplete.reload_libs.shutdown()

    # Partial controller manager ROS API: service with wrong type
    cm_bad_type = ControllerManagerDummy('/bad_type')
    cm_bad_type.unload_ctrl.shutdown()
    cm_bad_type.unload_ctrl = rospy.Service('/bad_type/unload_controller',
                                            LoadController,  # NOTE: Wrong type
                                            cm_bad_type._unload_ctrl_cb)

    rospy.spin()
