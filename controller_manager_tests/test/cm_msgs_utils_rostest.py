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

import unittest
import rospy
from controller_manager_msgs.utils import *

valid_cm = [
    '/',
    '/controller_manager',
    '/foo/robot/controller_manager1',
    '/foo/robot/controller_manager2'
]

invalid_cm = [
    'non_existent',
    '/incomplete',
    '/bad_type'
]


class TestUtils(unittest.TestCase):
    def _ready(self):
        try:
            rospy.wait_for_service("/controller_manager/list_controllers", 2.0)
        except ROSException:
            return False
        return True

    def test_is_controller_manager(self):
        self.assertTrue(self._ready())

        for cm in valid_cm:
            self.assertTrue(is_controller_manager(cm))
        for cm in invalid_cm:
            self.assertFalse(is_controller_manager(cm))

    def test_get_controller_managers(self):
        self.assertTrue(self._ready())

        # Root namespace
        self.assertEqual(valid_cm, get_controller_managers())
        self.assertEqual(valid_cm, get_controller_managers('/'))

        # Nested namespace
        nested_cm = [valid_cm[2], valid_cm[3]]
        self.assertEqual(nested_cm, get_controller_managers('/foo'))
        self.assertEqual(nested_cm, get_controller_managers('/foo/robot'))
        self.assertEqual(['/controller_manager'],
                         get_controller_managers('/controller_manager'))

        # Initial guess: Recommended usage pattern
        prev_cm = get_controller_managers()
        self.assertEqual(valid_cm,
                         get_controller_managers(initial_guess=prev_cm))

        # Initial guess: Partial guess
        self.assertEqual(valid_cm,
                         get_controller_managers(initial_guess=nested_cm))

        # Misuse of initial guess. Specifying entries that have not gone
        # through a full API check can yield incorrect results.
        # You have been warned!
        wrong_cm = get_controller_managers(initial_guess=invalid_cm)
        self.assertNotEqual(valid_cm, wrong_cm)
        diff = list(set(wrong_cm) - set(valid_cm))
        self.assertEqual(2, len(diff))
        self.assertIn('/incomplete', diff)
        self.assertIn('/bad_type', diff)

    def test_controller_manager_lister(self):
        self.assertTrue(self._ready())

        # Root namespace
        list_cm = ControllerManagerLister()
        self.assertEqual(valid_cm, list_cm())

        # Nested namespace
        list_cm_foo = ControllerManagerLister('/foo')
        nested_cm = [valid_cm[2], valid_cm[3]]
        self.assertEqual(nested_cm, list_cm_foo())

    def test_controller_lister(self):
        self.assertTrue(self._ready())

        # Default namespace
        list_controllers = ControllerLister()
        controllers = list_controllers()
        self.assertEqual(2, len(controllers))
        self.assertEqual('foo_controller', controllers[0].name)
        self.assertEqual('bar_controller', controllers[1].name)

        # Custom namespace
        list_controllers_ns = ControllerLister('/foo')
        self.assertEqual(0, len(list_controllers_ns()))

    def test_rosparam_controller_names(self):
        # Default namespace
        names_def = get_rosparam_controller_names()
        self.assertEqual(2, len(names_def))
        self.assertIn('foo_controller', names_def)
        self.assertIn('bar_controller', names_def)

        # Root namespace
        names_root = get_rosparam_controller_names('/')
        self.assertEqual(2, len(names_root))
        self.assertIn('foo_controller', names_root)
        self.assertIn('bar_controller', names_root)

        # Empty namespace
        names_empty = get_rosparam_controller_names('')
        self.assertEqual(2, len(names_empty))
        self.assertIn('foo_controller', names_empty)
        self.assertIn('bar_controller', names_empty)

        # Custom namespace
        names_ns  = get_rosparam_controller_names('/ns')
        self.assertEqual(2, len(names_ns))
        self.assertIn('baz_controller', names_ns)
        self.assertIn('qux_controller', names_ns)

        # Custom namespace, trailing slash
        names_nss  = get_rosparam_controller_names('/ns/')
        self.assertEqual(2, len(names_nss))
        self.assertIn('baz_controller', names_nss)
        self.assertIn('qux_controller', names_nss)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('controller_manager_msgs',
                   'cm_msgs_utils_rostest',
                   TestUtils)
