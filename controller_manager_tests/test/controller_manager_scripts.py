#!/usr/bin/env python
import unittest
import rospy
import subprocess

# output of controller_manager list, will by combined dynamically
myc1_running='my_controller1 - hardware_interface::EffortJointInterface ( running )\n'
myc1_stopped='my_controller1 - hardware_interface::EffortJointInterface ( stopped )\n'
myc2_running='my_controller2 - hardware_interface::EffortJointInterface ( running )\n'
myc2_stopped='my_controller2 - hardware_interface::EffortJointInterface ( stopped )\n'


# output of other commands
loaded_fmt = 'Loaded %s\n'
unloaded_fmt = 'Unloaded %s successfully\n'
stopped_fmt = "Stopped ['%s'] successfully\n"
started_fmt = "Started ['%s'] successfully\n"

no_controllers = 'No controllers are loaded in mechanism control\n'
reload_response = 'Restore: False\nSuccessfully reloaded libraries\n'

# run controller_manager in process
def run_cm(*args):
    return subprocess.check_output('rosrun controller_manager controller_manager ' + ' '.join(args), shell=True)

# helper function that return the actual and expected response
def load_c(name):
    return run_cm('load', name), loaded_fmt % name

def unload_c(name):
    return run_cm('unload', name), unloaded_fmt % name

def stop_c(name):
    return run_cm('stop', name), stopped_fmt % name

def start_c(name):
    return run_cm('start', name), started_fmt % name

def spawn_c(name):
    return run_cm('spawn', name), loaded_fmt % name + started_fmt % name

def kill_c(name):
    return run_cm('kill', name), stopped_fmt % name + unloaded_fmt % name

class TestUtils(unittest.TestCase):
    def test_scripts(self):
        rospy.wait_for_service("/controller_manager/list_controllers", 2.0)

        # ensure that test controller is available
        listed_types = run_cm('list-types').split()
        self.assertIn('controller_manager_tests/EffortTestController', listed_types)

        # ensure that no controllers are loaded
        self.assertEqual(run_cm('list'), no_controllers)

        # spawn (load+start) my_controller1 externally
        s1 = subprocess.Popen('rosrun controller_manager spawner my_controller1 __name:=myspawner', shell=True)
        rospy.sleep(1.0)
        self.assertEqual(run_cm('list'), myc1_running)

        # load my_controller2
        self.assertEqual(*load_c('my_controller2'))
        self.assertEqual(run_cm('list'), myc1_running + myc2_stopped)

        # stop my_controller1
        self.assertEqual(*stop_c('my_controller1'))
        self.assertEqual(run_cm('list'), myc1_stopped + myc2_stopped)

        # start my_controller2
        self.assertEqual(*start_c('my_controller2'))
        self.assertEqual(run_cm('list'), myc1_stopped + myc2_running)

        # stop my_controller2
        self.assertEqual(*stop_c('my_controller2'))
        self.assertEqual(run_cm('list'), myc1_stopped + myc2_stopped)

        # start my_controller1
        self.assertEqual(*start_c('my_controller1'))
        self.assertEqual(run_cm('list'), myc1_running + myc2_stopped)

        # stop my_controller1 externally
        u1 = subprocess.Popen('rosrun controller_manager unspawner my_controller1 __name:=myunspawner', shell=True)
        rospy.sleep(1.0)
        self.assertEqual(run_cm('list'), myc1_stopped + myc2_stopped)

        # kill unspawner -> restart my_controller1
        subprocess.check_call('rosnode kill /myunspawner', shell=True)
        self.assertEqual(u1.wait(), 0)
        self.assertEqual(run_cm('list'), myc1_running + myc2_stopped)

        # kill spawner -> stop+unload my_controller1
        subprocess.check_call('rosnode kill /myspawner', shell=True)
        self.assertEqual(0, s1.wait())
        self.assertEqual(run_cm('list'), myc2_stopped)

        # spawn (load+start) my_controller1
        self.assertEqual(*spawn_c('my_controller1'))
        self.assertEqual(run_cm('list'), myc2_stopped + myc1_running)

        # kill (stop+unload) my_controller1
        self.assertEqual(*kill_c('my_controller1'))
        self.assertEqual(run_cm('list'), myc2_stopped)

        # unload my_controller2
        self.assertEqual(*unload_c('my_controller2'))
        self.assertEqual(run_cm('list'), no_controllers)

        self.assertEqual(run_cm('reload-libraries'), reload_response)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('controller_manager_msgs',
                   'controller_manager_scripts_rostest',
                   TestUtils)
