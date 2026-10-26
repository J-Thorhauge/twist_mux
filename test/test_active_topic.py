# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import threading
import time
import unittest

import launch_testing
import rclpy

from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

PUBLISH_PERIOD = 0.05
# Inputs time out after 0.3s; leave margin for the 50ms active check timer.
SETTLE_TIME = 0.6


def generate_test_description():
    parameters_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), 'active_config.yaml')

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[parameters_file])

    return LaunchDescription([
        twist_mux,
        launch_testing.actions.ReadyToTest(),
    ]), {'twist_mux': twist_mux}


class TestActiveTopic(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_active_topic')

        cls.active = []
        cls.cmd_vel_out = []
        cls.lock = threading.Lock()

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        cls.node.create_subscription(
            String, '/twist_mux/active', cls._on_active, latched)
        cls.node.create_subscription(
            Twist, 'cmd_vel_out', cls._on_cmd_vel_out, 10)

        cls.pub_high = cls.node.create_publisher(Twist, 'vel_high', 10)
        cls.pub_low = cls.node.create_publisher(Twist, 'vel_low', 10)
        cls.pub_lock = cls.node.create_publisher(Bool, 'lock_pause', 10)

        cls.publish_high = False
        cls.publish_low = False
        cls.node.create_timer(PUBLISH_PERIOD, cls._publish_inputs)

        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_active(cls, msg):
        with cls.lock:
            cls.active.append(msg.data)

    @classmethod
    def _on_cmd_vel_out(cls, msg):
        with cls.lock:
            cls.cmd_vel_out.append(msg.linear.x)

    @classmethod
    def _publish_inputs(cls):
        if cls.publish_high:
            msg = Twist()
            msg.linear.x = 2.0
            cls.pub_high.publish(msg)
        if cls.publish_low:
            msg = Twist()
            msg.linear.x = 1.0
            cls.pub_low.publish(msg)

    def _wait_for_active(self, expected, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.active and self.active[-1] == expected:
                    return
            time.sleep(0.01)
        with self.lock:
            history = list(self.active)
        self.fail(f"active never became '{expected}', history: {history}")

    def _reset(self):
        self._set_inputs(high=False, low=False)
        self.pub_lock.publish(Bool(data=False))
        self._wait_for_active('')
        with self.lock:
            self.active.clear()
            self.cmd_vel_out.clear()

    def _set_inputs(self, high, low):
        type(self).publish_high = high
        type(self).publish_low = low

    def test_active_topic(self):
        # Latched initial value: nothing has published yet.
        self._wait_for_active('')
        self._reset()

        # A single source becomes active, and is reported only once.
        self._set_inputs(high=False, low=True)
        self._wait_for_active('low')
        time.sleep(SETTLE_TIME)
        with self.lock:
            self.assertEqual(self.active, ['low'])
            self.assertTrue(self.cmd_vel_out)
            self.assertTrue(all(x == 1.0 for x in self.cmd_vel_out))

        # A higher priority source takes over.
        self._set_inputs(high=True, low=True)
        self._wait_for_active('high')
        time.sleep(SETTLE_TIME)
        with self.lock:
            self.assertEqual(self.active, ['low', 'high'])
            self.cmd_vel_out.clear()
        time.sleep(SETTLE_TIME)
        with self.lock:
            self.assertTrue(self.cmd_vel_out)
            self.assertTrue(all(x == 2.0 for x in self.cmd_vel_out))

        # The higher priority source times out; control falls back.
        self._set_inputs(high=False, low=True)
        self._wait_for_active('low')

        # Everything times out: the timer reports it with no input traffic.
        self._set_inputs(high=False, low=False)
        self._wait_for_active('')
        with self.lock:
            self.assertEqual(self.active, ['low', 'high', 'low', ''])

    def test_lock_masks_lower_priority(self):
        self._reset()

        self.pub_lock.publish(Bool(data=True))
        self._set_inputs(high=False, low=True)
        time.sleep(SETTLE_TIME)
        with self.lock:
            self.assertEqual(self.active, [])
            self.assertEqual(self.cmd_vel_out, [])

        self._set_inputs(high=True, low=True)
        self._wait_for_active('high')

        self._reset()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15], process='twist_mux')
