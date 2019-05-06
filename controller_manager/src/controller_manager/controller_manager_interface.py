#! /usr/bin/env python
from __future__ import print_function
import rospy
from controller_manager_msgs.srv import *


def list_controller_types():
    rospy.wait_for_service('controller_manager/list_controller_types')
    s = rospy.ServiceProxy('controller_manager/list_controller_types', ListControllerTypes)
    resp = s.call(ListControllerTypesRequest())
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore = False):
    rospy.wait_for_service('controller_manager/reload_controller_libraries')
    s = rospy.ServiceProxy('controller_manager/reload_controller_libraries', ReloadControllerLibraries)

    list_srv = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
    load_srv = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
    switch_srv = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

    print("Restore: " + str(restore))
    if restore:
        originally = list_srv.call(ListControllersRequest())

    resp = s.call(ReloadControllerLibrariesRequest(force_kill))
    if resp.ok:
        print("Successfully reloaded libraries")
        result = True
    else:
        print("Failed to reload libraries. Do you still have controllers loaded?")
        result = False

    if restore:
        for c in originally.controller:
            load_srv(c.name)
        to_start = [c.name for c in originally.controller if c.state == 'running']
        switch_srv(start_controllers=to_start,
                   stop_controllers=[],
                   strictness=SwitchControllerRequest.BEST_EFFORT)
        print("Controllers restored to original state")
    return result


def list_controllers():
    rospy.wait_for_service('controller_manager/list_controllers')
    s = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())
    if len(resp.controller) == 0:
        print("No controllers are loaded in mechanism control")
    else:
        for c in resp.controller:
            hwi = list(set(r.hardware_interface for r in  c.claimed_resources))
            print("'%s' - '%s' ( %s )" % (c.name, "+".join(hwi), c.state))


def load_controller(name):
    rospy.wait_for_service('controller_manager/load_controller')
    s = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
    resp = s.call(LoadControllerRequest(name))
    if resp.ok:
        print("Loaded \'" + name + "\'")
        return True
    else:
        print("Error when loading \'" + name + "\'")
        return False


def unload_controller(name):
    rospy.wait_for_service('controller_manager/unload_controller')
    s = rospy.ServiceProxy('controller_manager/unload_controller', UnloadController)
    resp = s.call(UnloadControllerRequest(name))
    if resp.ok == 1:
        print("Unloaded \'" + name + "\' successfully")
        return True
    else:
        print("Error when unloading \'" + name + "\'")
        return False


def start_controller(name):
    return start_stop_controllers(start_controllers=[name])


def start_controllers(names):
    return start_stop_controllers(start_controllers=names)


def stop_controller(name):
    return start_stop_controllers(stop_controllers=[name])


def stop_controllers(names):
    return start_stop_controllers(stop_controllers=names)


def start_stop_controllers(start_controllers=[], stop_controllers=[]):
    rospy.wait_for_service('controller_manager/switch_controller')
    s = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
    strictness = SwitchControllerRequest.STRICT
    resp = s.call(SwitchControllerRequest(start_controllers, stop_controllers, strictness, False, 0.0))
    if resp.ok == 1:
        if start_controllers:
            print("Started {} successfully".format(start_controllers))
        if stop_controllers:
            print("Stopped {} successfully".format(stop_controllers))
        return True
    else:
        print("Error when starting {} and stopping {}".format(start_controllers, stop_controllers))
        return False
