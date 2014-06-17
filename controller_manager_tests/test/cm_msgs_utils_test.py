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

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.utils import *

ctrl_list = [
    ControllerState(name='foo_controller',
                    state='running',
                    type='foo_base/foo',
                    hardware_interface='hardware_interface::FooInterface',
                    resources=['one', 'two', 'three']),
    ControllerState(name='bar_controller',
                    state='running',
                    type='bar_base/bar',
                    hardware_interface='hardware_interface::BarInterface',
                    resources=['four']),
    ControllerState(name='foobar_controller',
                    state='stopped',
                    type='foobar_base/foobar',
                    hardware_interface='hardware_interface::FooBarInterface',
                    resources=['one'])
]


def filter_by_name_test():
    # Non-existing
    assert(not filter_by_name(ctrl_list, 'null', match_substring=False))
    assert(not filter_by_name(ctrl_list, 'null', match_substring=True))

    # Existing, full match
    out = filter_by_name(ctrl_list, 'foo_controller')
    assert(1 == len(out))
    assert(out[0].name == 'foo_controller')

    # Existing, substring match
    out = filter_by_name(ctrl_list, 'foo_controller', match_substring=True)
    assert(1 == len(out))
    assert(out[0].name == 'foo_controller')

    out = filter_by_name(ctrl_list, 'foo', match_substring=True)
    assert(2 == len(out))
    assert(out[0].name == 'foo_controller')
    assert(out[1].name == 'foobar_controller')


def filter_by_state_test():
    # Non-existing
    assert(not filter_by_state(ctrl_list, 'null', match_substring=False))
    assert(not filter_by_state(ctrl_list, 'null', match_substring=True))

    # Existing, full match
    out = filter_by_state(ctrl_list, 'stopped')
    assert(1 == len(out))
    assert(out[0].name == 'foobar_controller')

    # Existing, substring match
    out = filter_by_state(ctrl_list, 'stopped', match_substring=True)
    assert(1 == len(out))
    assert(out[0].name == 'foobar_controller')

    out = filter_by_state(ctrl_list, 'run', match_substring=True)
    assert(2 == len(out))
    assert(out[0].name == 'foo_controller')
    assert(out[1].name == 'bar_controller')


def filter_by_type_test():
    # Non-existing
    assert(not filter_by_type(ctrl_list, 'null', match_substring=False))
    assert(not filter_by_type(ctrl_list, 'null', match_substring=True))

    # Existing, full match
    out = filter_by_type(ctrl_list, 'foo_base/foo')
    assert(1 == len(out))
    assert(out[0].name == 'foo_controller')

    # Existing, substring match
    out = filter_by_type(ctrl_list, 'foo_base/foo', match_substring=True)
    assert(1 == len(out))
    assert(out[0].name == 'foo_controller')

    out = filter_by_type(ctrl_list, 'foo', match_substring=True)
    assert(2 == len(out))
    assert(out[0].name == 'foo_controller')
    assert(out[1].name == 'foobar_controller')


def filter_by_hardware_interface_test():
    # Non-existing
    assert(not filter_by_hardware_interface(ctrl_list,
                                            'null',
                                            match_substring=False))
    assert(not filter_by_hardware_interface(ctrl_list,
                                            'null',
                                            match_substring=True))

    # Existing, full match
    out = filter_by_hardware_interface(ctrl_list,
                                       'hardware_interface::FooInterface')
    assert(1 == len(out))
    assert(out[0].name == 'foo_controller')

    # Existing, substring match
    out = filter_by_hardware_interface(ctrl_list,
                                       'hardware_interface::FooInterface',
                                       match_substring=True)
    assert(1 == len(out))
    assert(out[0].name == 'foo_controller')

    out = filter_by_hardware_interface(ctrl_list,
                                       'FooInterface',
                                       match_substring=True)
    assert(1 == len(out))
    assert(out[0].name == 'foo_controller')


def filter_by_resources_test():
    # Non-existing
    assert(not filter_by_resources(ctrl_list, ['null'], match_any=False))
    assert(not filter_by_resources(ctrl_list, ['null'], match_any=True))

    # Existing, full match
    res = ctrl_list[0].resources
    out = filter_by_resources(ctrl_list, res)
    assert(1 == len(out))
    assert(out[0].name == 'foo_controller')

    # Existing, partial match
    out = filter_by_resources(ctrl_list, res, match_any=True)
    assert(2 == len(out))
    assert(out[0].name == 'foo_controller')
    assert(out[1].name == 'foobar_controller')

    out = filter_by_resources(ctrl_list, ['one'], match_any=True)
    assert(2 == len(out))
    assert(out[0].name == 'foo_controller')
    assert(out[1].name == 'foobar_controller')
