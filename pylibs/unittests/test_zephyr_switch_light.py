#!/usr/bin/env python3
# Copyright (c) 2026, The OTNS Authors.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Unit tests of the Zephyr 'switch_input' and 'light_output' example apps, running as simulated native_sim
nodes in OTNS. These tests only run with the Zephyr-based nodes: build them with zephyr/script/build_examples
and point OTNS_NODES_DIR to the directory with the installed launchers, e.g.:

    OTNS_NODES_DIR=./zephyr/ot-versions python3 pylibs/unittests/test_zephyr_switch_light.py
"""

import os
import unittest
from pathlib import Path

from OTNSTestCase import OTNSTestCase
from otns.cli import errors

NODES_DIR = Path(os.environ.get('OTNS_NODES_DIR', ''))
SWITCH_EXE = NODES_DIR / 'switch'
LIGHT_EXE = NODES_DIR / 'light'

# The switch app drops a button press if the previous CoAP request is still in progress. Presses
# separated by at least this time (in simulated seconds) are reliably delivered.
PRESS_INTERVAL = 3
NETWORK_FORM_TIMEOUT = 300
NETWORK_SETTLE_TIME = 30


@unittest.skipUnless(
    os.environ.get('OTNS_NODES_DIR') and SWITCH_EXE.is_file() and LIGHT_EXE.is_file(),
    'Requires the Zephyr-based nodes: build with zephyr/script/build_examples and set '
    'OTNS_NODES_DIR=<repo>/zephyr/ot-versions')
class SwitchLightTestCase(OTNSTestCase):

    def addSwitch(self, x: float = 100, y: float = 100) -> int:
        return self.ns.add('router', x, y, executable=str(SWITCH_EXE))

    def addLight(self, x: float = 200, y: float = 100) -> int:
        return self.ns.add('router', x, y, executable=str(LIGHT_EXE))

    def formNetwork(self) -> None:
        """Run the simulation until all nodes joined a single partition as leader or router."""
        ns = self.ns
        for _ in range(NETWORK_FORM_TIMEOUT // 10):
            self.go(10)
            if len(ns.partitions()) == 1 and 0 not in ns.partitions() and all(
                    ns.get_state(nid) in ('leader', 'router') for nid in ns.nodes()):
                self.go(NETWORK_SETTLE_TIME)  # let the router links and multicast forwarding settle
                return
        self.fail('Network did not form in time: ' + str({nid: ns.get_state(nid) for nid in ns.nodes()}))

    def press(self, switch: int, button: str) -> None:
        self.ns.node_cmd(switch, f'switch press {button}')
        self.go(PRESS_INTERVAL)

    def pressUntilLights(self, switch: int, button: str, expected: str, *lights: int, attempts: int = 4) -> None:
        """
        Press an idempotent button ('on' or 'off') until all lights reach the expected state. The CoAP
        command is a non-confirmable multicast without retransmission by the app, so a single press
        may get lost on a multi-hop or multi-receiver path.
        """
        for _ in range(attempts):
            self.press(switch, button)
            if all(self.lightState(light) == expected for light in lights):
                return
        self.assertLights(expected, *lights)

    def lightState(self, light: int) -> str:
        output = self.ns.node_cmd(light, 'light state')
        self.assertEqual(1, len(output), output)
        return output[0]

    def assertLights(self, expected: str, *lights: int) -> None:
        for light in lights:
            self.assertEqual(expected, self.lightState(light), f'Light node {light} state mismatch')


class LightOutputTests(SwitchLightTestCase):
    """Tests of the light_output app alone."""

    def testLightInitiallyOff(self):
        light = self.addLight()
        self.go(1)
        self.assertLights('off', light)

    def testLightStateStaysOffWithoutCommands(self):
        light = self.addLight()
        self.go(60)
        self.assertLights('off', light)
        self.assertNodeState(light, 'leader')

    def testLightStateInvalidArgs(self):
        light = self.addLight()
        self.go(1)
        for cmd in ('light', 'light foo', 'light state extra'):
            with self.assertRaises(errors.OTNSCliError, msg=cmd):
                self.ns.node_cmd(light, cmd)
        self.assertLights('off', light)


class SwitchInputTests(SwitchLightTestCase):
    """Tests of the switch_input app alone."""

    def testPressInvalidArgs(self):
        switch = self.addSwitch()
        self.go(1)
        for cmd in ('switch', 'switch press', 'switch press foo', 'switch foo on', 'switch press on extra'):
            with self.assertRaises(errors.OTNSCliError, msg=cmd):
                self.ns.node_cmd(switch, cmd)

    def testPressWithoutNetwork(self):
        # Pressing a button while the node is not attached must not crash or hang the node.
        switch = self.addSwitch()
        self.go(1)
        for button in ('on', 'off', 'toggle'):
            self.ns.node_cmd(switch, f'switch press {button}')
            self.go(PRESS_INTERVAL)
        self.assertEqual(1, len(self.ns.nodes()))
        self.assertNodeState(switch, 'leader')


class SwitchLightTests(SwitchLightTestCase):
    """Tests of the switch_input and light_output apps interacting over a Thread network."""

    def testOnOff(self):
        switch, light = self.addSwitch(), self.addLight()
        self.formNetwork()
        self.assertLights('off', light)

        self.press(switch, 'on')
        self.assertLights('on', light)
        self.press(switch, 'off')
        self.assertLights('off', light)

    def testOnOffAreIdempotent(self):
        switch, light = self.addSwitch(), self.addLight()
        self.formNetwork()

        self.press(switch, 'off')
        self.assertLights('off', light)
        self.press(switch, 'on')
        self.press(switch, 'on')
        self.assertLights('on', light)
        self.press(switch, 'off')
        self.press(switch, 'off')
        self.assertLights('off', light)

    def testToggle(self):
        switch, light = self.addSwitch(), self.addLight()
        self.formNetwork()

        expected = 'off'
        for _ in range(4):
            expected = 'on' if expected == 'off' else 'off'
            self.press(switch, 'toggle')
            self.assertLights(expected, light)

    def testToggleAfterOnOff(self):
        switch, light = self.addSwitch(), self.addLight()
        self.formNetwork()

        self.press(switch, 'on')
        self.press(switch, 'toggle')
        self.assertLights('off', light)
        self.press(switch, 'toggle')
        self.assertLights('on', light)
        self.press(switch, 'off')
        self.press(switch, 'toggle')
        self.assertLights('on', light)

    def testMulticastControlsAllLights(self):
        switch = self.addSwitch(200, 100)
        lights = [self.addLight(100, 100), self.addLight(300, 100), self.addLight(200, 200)]
        self.formNetwork()

        self.pressUntilLights(switch, 'on', 'on', *lights)
        self.pressUntilLights(switch, 'off', 'off', *lights)
        self.pressUntilLights(switch, 'on', 'on', *lights)

    def testLightsHaveIndependentState(self):
        switch = self.addSwitch(100, 100)
        light1 = self.addLight(200, 100)
        self.formNetwork()
        self.press(switch, 'on')
        self.assertLights('on', light1)

        # A light added later starts off, independent of the state of the existing one.
        light2 = self.addLight(200, 200)
        self.formNetwork()
        self.assertLights('on', light1)
        self.assertLights('off', light2)
        self.pressUntilLights(switch, 'on', 'on', light1, light2)
        self.pressUntilLights(switch, 'off', 'off', light1, light2)

    def testMultipleSwitchesControlLight(self):
        switch1, switch2 = self.addSwitch(100, 100), self.addSwitch(100, 200)
        light = self.addLight(200, 150)
        self.formNetwork()

        self.press(switch1, 'on')
        self.assertLights('on', light)
        self.press(switch2, 'off')
        self.assertLights('off', light)
        self.press(switch2, 'toggle')
        self.assertLights('on', light)
        self.press(switch1, 'toggle')
        self.assertLights('off', light)

    def testMultiHopMulticast(self):
        # Switch and light are out of direct radio range (radio range is 220): the command reaches
        # the light through the multicast forwarding of the router in between.
        switch = self.addSwitch(0, 100)
        middle = self.addLight(150, 100)
        light = self.addLight(300, 100)
        self.formNetwork()

        self.press(switch, 'on')
        self.assertLights('on', middle, light)
        self.press(switch, 'off')
        self.assertLights('off', middle, light)
        self.press(switch, 'toggle')
        self.assertLights('on', middle, light)

    def testNoCommandWhenOutOfRange(self):
        switch = self.addSwitch(0, 0)
        light = self.addLight(100, 0)
        self.formNetwork()
        self.press(switch, 'on')
        self.assertLights('on', light)

        # Moving the light far away detaches it from the switch: no further command reaches it.
        self.ns.move(light, 5000, 0)
        self.go(PRESS_INTERVAL)
        self.press(switch, 'off')
        self.assertLights('on', light)

    def testButtonPressesFurtherApart(self):
        switch, light = self.addSwitch(), self.addLight()
        self.formNetwork()

        self.press(switch, 'on')
        self.go(60)
        self.assertLights('on', light)
        self.press(switch, 'toggle')
        self.go(60)
        self.assertLights('off', light)


if __name__ == '__main__':
    unittest.main()
