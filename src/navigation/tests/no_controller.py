"""Check cancellation and start timeout when no controller node is running."""

import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from turtle_nav_interfaces.action import NavigateTurtle

from test_support import installed_executable

rclpy.init()
# Start only the public action server. Its internal controller action is absent.
server = subprocess.Popen(
    [installed_executable('turtle_navigation', 'navigation_action_server')],
    stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
node = Node('no_controller_client')
client = ActionClient(node, NavigateTurtle, '/navigate_turtle')

def wait(future, timeout=6):
    # ROS futures need this node to spin before their responses become visible.
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    assert future.done(), 'timeout'
    return future.result()

def send():
    goal = NavigateTurtle.Goal()
    goal.x, goal.y = 8.0, 5.5
    h = wait(client.send_goal_async(goal))
    assert h.accepted
    return h

try:
    assert client.wait_for_server(timeout_sec=8)
    # Cancel while the public server is still waiting for a controller.
    first = send()
    assert wait(first.cancel_goal_async()).goals_canceling
    canceled = wait(first.get_result_async())
    assert canceled.status == GoalStatus.STATUS_CANCELED
    assert 'before dispatch' in canceled.result.message or 'before controller dispatch' in canceled.result.message
    print('PASS cancel before controller dispatch:', canceled.result.message, flush=True)
    # A fresh goal left alone must abort when the controller start timer expires.
    second = send()
    aborted = wait(second.get_result_async())
    assert aborted.status == GoalStatus.STATUS_ABORTED
    assert 'Controller unavailable before start timeout' in aborted.result.message
    print('PASS missing controller start timeout', flush=True)
finally:
    server.terminate()
    try:
        server.wait(timeout=2)
    except subprocess.TimeoutExpired:
        server.kill()
        server.wait(timeout=2)
    node.destroy_node()
    rclpy.shutdown()
