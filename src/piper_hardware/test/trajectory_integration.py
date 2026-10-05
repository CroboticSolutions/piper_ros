#!/usr/bin/env python3
"""Isolated real-JTC test, synthetic hardware only. Run in ROS_DOMAIN_ID=87.

No CAN/SDK/robot driver is constructed. Unique /test topics are hardcoded.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import yaml

assert os.environ.get('ROS_DOMAIN_ID') == '87', 'Use isolated domain 87, never robot domain'

class Plant(Node):
    def __init__(self, xml):
        super().__init__('piper_test_plant')
        self.position = [0.4]*6
        self.command = list(self.position)
        self.follow = True
        self.publish_feedback = True
        self.ready = True
        self.stops = 0
        self.teaching = False
        self.teach_pub = self.create_publisher(Bool, '/piper/teaching', 10)
        self.names = [f'joint{i}' for i in range(1, 7)]
        self.state_pub = self.create_publisher(JointState, '/test/feedback', 10)
        self.ready_pub = self.create_publisher(Bool, '/test/ready', 10)
        self.create_subscription(JointState, '/test/commands', self.on_command, 10)
        self.create_subscription(Empty, '/test/stop', self.on_stop, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.description = self.create_publisher(String, '/robot_description', qos)
        self.description.publish(String(data=xml))
        self.create_timer(0.01, self.tick)
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def on_command(self, msg):
        m = dict(zip(msg.name, msg.position))
        self.command = [m[n] for n in self.names]

    def on_stop(self, _):
        self.stops += 1

    def tick(self):
        self.teach_pub.publish(Bool(data=self.teaching))
        if self.follow and not self.teaching:
            self.position = list(self.command)
        if self.publish_feedback:
            self.state_pub.publish(JointState(name=self.names, position=self.position, velocity=[0.0]*6,
                                             header=__import__('std_msgs.msg', fromlist=['Header']).Header(stamp=self.get_clock().now().to_msg())))
            self.ready_pub.publish(Bool(data=self.ready))

    def goal(self, delta, duration=1.0):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = self.names
        p = JointTrajectoryPoint()
        p.positions = [x+delta for x in self.position]
        p.velocities = [0.0]*6
        p.time_from_start = Duration(sec=int(duration), nanosec=int(duration % 1 * 1e9))
        g.trajectory.points = [p]
        f = self.client.send_goal_async(g)
        wait(f, 3)
        handle = f.result()
        assert handle.accepted
        return handle.get_result_async()


def wait(f, seconds):
    deadline = time.monotonic()+seconds
    while not f.done() and time.monotonic()<deadline:
        time.sleep(.01)
    assert f.done(), 'Action future did not reach a terminal state'


def main():
    root = Path(tempfile.mkdtemp(prefix='piper_jtc_test_'))
    links = '<link name="base"/>' + ''.join(f'<link name="l{i}"/>' for i in range(1,7))
    joints = ''.join(f'<joint name="joint{i}" type="revolute"><parent link="'+('base' if i==1 else f'l{i-1}')+f'"/><child link="l{i}"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="3"/></joint>' for i in range(1,7))
    hardware = '<hardware><plugin>piper_hardware/PiperSystem</plugin><param name="feedback_topic">/test/feedback</param><param name="command_topic">/test/commands</param><param name="ready_topic">/test/ready</param><param name="stop_topic">/test/stop</param></hardware>'
    control = ''.join(f'<joint name="joint{i}"><command_interface name="position"/><state_interface name="position"/><state_interface name="velocity"/></joint>' for i in range(1,7))
    xml = f'<robot name="test">{links}{joints}<ros2_control name="TestPiper" type="system">{hardware}{control}</ros2_control></robot>'
    params = {'controller_manager':{'ros__parameters':{'update_rate':100,'arm_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'}}},
      'arm_controller':{'ros__parameters':{'joints':[f'joint{i}' for i in range(1,7)],'command_interfaces':['position'],'state_interfaces':['position','velocity'],'interpolate_from_desired_state':False,'set_last_command_interface_value_as_state_on_activation':False,
      'constraints':{'goal_time':.5,'stopped_velocity_tolerance':.02, **{f'joint{i}':{'trajectory':.05,'goal':.01} for i in range(1,7)}}}}}
    config=root/'params.yaml';config.write_text(yaml.safe_dump(params))
    rclpy.init();plant=Plant(xml);executor=SingleThreadedExecutor();executor.add_node(plant)
    thread=threading.Thread(target=executor.spin);thread.start()
    logfile=(root/'manager.log').open('w')
    manager=subprocess.Popen(['ros2','run','controller_manager','ros2_control_node','--ros-args','--params-file',str(config)],stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
    results={}
    teach = None
    try:
        subprocess.run(['ros2','run','controller_manager','spawner','arm_controller','--controller-manager-timeout','15'],check=True,timeout=25,stdout=logfile,stderr=subprocess.STDOUT)
        assert plant.client.wait_for_server(timeout_sec=5)
        f=plant.goal(.15);wait(f,4);r=f.result()
        results['tracking_success']={'status':r.status,'code':r.result.error_code}
        assert r.status==4 and r.result.error_code==0, results
        plant.follow=False
        f=plant.goal(.15);wait(f,4);r=f.result()
        results['immobile_path_rejected']={'status':r.status,'code':r.result.error_code}
        assert r.status==6 and r.result.error_code==-4, results
        time.sleep(.2)
        f=plant.goal(.02);wait(f,4);r=f.result()
        results['immobile_goal_rejected']={'status':r.status,'code':r.result.error_code}
        assert r.status==6 and r.result.error_code==-5, results
        plant.follow=True;time.sleep(.2)
        teach = subprocess.Popen(['ros2','run','piper','piper_teach_sync','--ros-args','-p','gripper_exist:=false','-r','/joint_states:=/test/feedback'], stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(1.0)
        f=plant.goal(.15,2.0);time.sleep(.2);plant.teaching=True
        wait(f,3);r=f.result()
        results['teach_cancels_action']={'status':r.status,'code':r.result.error_code}
        assert r.status in (5,6), results
        plant.position=[0.7]*6;time.sleep(.4);plant.teaching=False;time.sleep(.4)
        assert max(abs(x-.7) for x in plant.command)<.01, plant.command
        results['teach_exit_hold']=list(plant.command)
        f=plant.goal(.15,2.0);time.sleep(.2);plant.publish_feedback=False
        wait(f,4);r=f.result()
        results['lost_feedback']={'status':r.status,'code':r.result.error_code,'stop_messages':plant.stops}
        assert r.status!=4 and plant.stops>0, results
        print(json.dumps({'results':results,'logs':str(root)},indent=2))
    finally:
        if teach is not None:
            os.killpg(teach.pid, signal.SIGINT)
            teach.wait(timeout=5)
        os.killpg(manager.pid, signal.SIGINT)
        try:manager.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(manager.pid,signal.SIGKILL);manager.wait()
        executor.shutdown();thread.join();plant.destroy_node();rclpy.shutdown();logfile.close()
        print('Test logs:',root)

if __name__=='__main__':main()
