"""Check the controller's missing, invalid, and stale pose handling."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from turtle_nav_interfaces.action import ControllerNavigate, NavigateTurtle
from turtlesim.msg import Pose

rclpy.init()
node = Node('pose_fault_client')
public = ActionClient(node, NavigateTurtle, '/navigate_turtle')
internal = ActionClient(node, ControllerNavigate, '/controller/navigate')
# The fixture starts the navigation nodes without Turtlesim, so this test
# controls exactly which pose messages the controller sees.
pose_publisher = node.create_publisher(Pose, '/turtle1/pose', 10)
assert public.wait_for_server(timeout_sec=8)
assert internal.wait_for_server(timeout_sec=8)

def wait(future, timeout=8):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done(), 'timeout'
    return future.result()

def send(action, x, y):
    goal = NavigateTurtle.Goal()
    goal.x, goal.y = float(x), float(y)
    handle = wait(action.send_goal_async(goal))
    assert handle.accepted
    return handle

def publish(x, y, theta=0.0):
    pose = Pose()
    pose.x, pose.y, pose.theta = float(x), float(y), float(theta)
    pose_publisher.publish(pose)
    # Give ROS discovery and delivery time before requesting the next goal.
    end = time.monotonic() + 0.1
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

try:
    # No publisher has sent a pose yet.
    first = send(public, 8, 5.5)
    first_result = wait(first.get_result_async())
    assert first_result.status == GoalStatus.STATUS_ABORTED
    assert 'No turtle pose' in first_result.result.message
    print('PASS missing pose abort', flush=True)

    # A received pose with a non-finite coordinate is invalid.
    publish(math.nan, 5.5)
    second = send(public, 8, 5.5)
    second_result = wait(second.get_result_async())
    assert second_result.status == GoalStatus.STATUS_ABORTED
    assert 'stale or invalid' in second_result.result.message
    print('PASS invalid pose abort', flush=True)

    # A valid pose expires while no new pose messages arrive.
    publish(5.5, 5.5)
    third = send(public, 8, 5.5)
    third_result = wait(third.get_result_async())
    assert third_result.status == GoalStatus.STATUS_ABORTED
    assert 'stale or invalid' in third_result.result.message
    print('PASS stale pose abort', flush=True)

    # Direct internal requests must still reject out-of-bounds targets.
    internal_goal = ControllerNavigate.Goal()
    internal_goal.x, internal_goal.y = 11.0, 5.5
    assert not wait(internal.send_goal_async(internal_goal)).accepted
    print('PASS internal invalid target rejection', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
