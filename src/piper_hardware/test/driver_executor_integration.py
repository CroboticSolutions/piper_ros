#!/usr/bin/env python3
"""Fake-SDK executor test: stop must not wait for enable polling and must only hold."""
import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from piper_msgs.srv import Enable
from std_msgs.msg import Empty
from piper.piper_ctrl_single_node import PiperRosNode

assert os.environ.get('ROS_DOMAIN_ID')=='87'
rclpy.init(args=['--ros-args','-p','require_hardware_feedback:=true','-p','gripper_exist:=false'])
sdk=Mock()
sdk.motors_enabled.return_value=[False]*6
with patch('piper.piper_ctrl_single_node.create_backend',return_value=sdk), patch.object(PiperRosNode,'publish_thread',lambda self: None):
    driver=PiperRosNode()
client_node=Node('driver_executor_test',use_global_arguments=False)
client=client_node.create_client(Enable,'/enable_srv')
pub=client_node.create_publisher(Empty,'/piper/stop',1)
executor=MultiThreadedExecutor(num_threads=3)
executor.add_node(driver);executor.add_node(client_node)
t=threading.Thread(target=executor.spin);t.start()
try:
    assert client.wait_for_service(timeout_sec=3)
    time.sleep(.2)
    future=client.call_async(Enable.Request(enable_request=True))
    deadline=time.monotonic()+2
    while not sdk.enable.called and time.monotonic()<deadline:time.sleep(.01)
    assert sdk.enable.called and not future.done(), 'Enable polling not active'
    start=time.monotonic();pub.publish(Empty())
    while not driver.guard.stopped and time.monotonic()-start<1:time.sleep(.005)
    elapsed=time.monotonic()-start
    assert elapsed<.5, elapsed
    deadline=time.monotonic()+1
    while not future.done() and time.monotonic()<deadline:time.sleep(.01)
    assert future.done() and not future.result().enable_response
    # Hold only: quick-stop is a damped e-stop (the arm sinks) and DisableArm releases it.
    assert not sdk.quick_stop.called and not sdk.disable.called
    print(f'PASS: hold latched during enable polling in {elapsed:.3f}s, no e-stop frame; fake SDK only')
finally:
    executor.shutdown();t.join();driver.destroy_node();client_node.destroy_node();rclpy.shutdown()
