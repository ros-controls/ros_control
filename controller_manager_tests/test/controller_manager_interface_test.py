#!/usr/bin/env python
import unittest
from controller_manager import controller_manager_interface


class TestUtils(unittest.TestCase):
    def test_scripts(self):

        # load my_controller1.
        self.assertTrue(controller_manager_interface.load_controller('my_controller1'))

        # load a non-existent controller.
        self.assertFalse(controller_manager_interface.load_controller('non_existent_controller'))

        # start my_controller1.
        self.assertTrue(controller_manager_interface.start_controller('my_controller1'))

        # start my_controller3 which hasn't been loaded.
        self.assertFalse(controller_manager_interface.start_controller('my_controller3'))

        # stop my_controller1
        self.assertTrue(controller_manager_interface.stop_controller('my_controller1'))

        # load my_controller3.
        self.assertTrue(controller_manager_interface.load_controller('my_controller3'))

        # start my_controller1 and my_controller3
        self.assertTrue(controller_manager_interface.start_controllers(('my_controller1', 'my_controller3')))

        # stop my_controller1 and my_controller3
        self.assertTrue(controller_manager_interface.stop_controllers(('my_controller1', 'my_controller3')))

        # reload libraries and restore controllers
        self.assertTrue(controller_manager_interface.reload_libraries(force_kill=True, restore=True))

        # unload my_controller1.
        self.assertTrue(controller_manager_interface.unload_controller('my_controller1'))
        # unload my_controller2.
        self.assertTrue(controller_manager_interface.unload_controller('my_controller3'))

        # load my_controller1.
        self.assertTrue(controller_manager_interface.load_controller('my_controller1'))

        # reload librareis when some controllers are loaded.
        self.assertFalse(controller_manager_interface.reload_libraries(force_kill=False, restore=True))

        # reload librareis when controllers are loaded with force_kill=True.
        self.assertTrue(controller_manager_interface.reload_libraries(force_kill=True, restore=False))

        # reload librareis when no controllers are loaded.
        self.assertTrue(controller_manager_interface.reload_libraries(force_kill=False, restore=False))


if __name__ == '__main__':
    import rostest
    rostest.rosrun('controller_manager_msgs',
                   'controller_manager_scripts_rostest',
                   TestUtils)
