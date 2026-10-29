"""Kill the controller in motion and check the public server's failure policy."""

import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from turtle_nav_interfaces.action import NavigateTurtle
from turtlesim.msg import Pose

from test_support import installed_executable

# Launch the three processes separately so only the controller can be killed.
procs = [
    subprocess.Popen([installed_executable('turtlesim', 'turtlesim_node')], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT),
    subprocess.Popen([installed_executable('turtle_navigation', 'navigation_controller')], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT),
    subprocess.Popen([installed_executable('turtle_navigation', 'navigation_action_server')], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT),
]
rclpy.init()
node = Node('controller_crash_client')
client = ActionClient(node, NavigateTurtle, '/navigate_turtle')
feedback = []
poses = []
commands = []
# The last command and subsequent poses show what happens after the sole
# velocity publisher disappears; the server can report but cannot command a stop.
node.create_subscription(Pose, '/turtle1/pose', lambda msg: poses.append((msg.x, msg.y)), 10)
node.create_subscription(Twist, '/turtle1/cmd_vel', lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10)

def wait(future, timeout=10):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.03)
    assert future.done(), 'timeout'
    return future.result()

try:
    assert client.wait_for_server(timeout_sec=8)
    goal = NavigateTurtle.Goal()
    goal.x, goal.y = 9.0, 2.0
    handle = wait(client.send_goal_async(goal, feedback_callback=lambda msg: feedback.append(msg.feedback.current_distance)))
    assert handle.accepted
    # Wait for an actual forward command before killing the controller.
    end = time.monotonic() + 4
    while not any(linear > 0.1 for linear, _ in commands) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.03)
    assert any(linear > 0.1 for linear, _ in commands), 'controller did not command forward motion'
    pose_at_crash = poses[-1] if poses else None
    # SIGKILL prevents the controller from publishing its normal zero Twist.
    procs[1].kill()
    procs[1].wait(timeout=2)
    wrapped = wait(handle.get_result_async(), timeout=8)
    print('controller-crash result:', wrapped.status, wrapped.result.message, flush=True)
    if pose_at_crash and poses:
        pose_at_timeout = poses[-1]
        print('pose at crash / timeout:', pose_at_crash, pose_at_timeout, 'last command:', commands[-1], flush=True)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
        print('pose one second after timeout:', poses[-1], 'last command:', commands[-1], flush=True)
    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert 'did not confirm stop' in wrapped.result.message.lower()
    # An unknown stop state is latched: new public goals must be rejected.
    next_goal = NavigateTurtle.Goal()
    next_goal.x, next_goal.y = 2.0, 2.0
    assert not wait(client.send_goal_async(next_goal)).accepted
    print('PASS controller crash aborts active goal and latches new goals', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
    for proc in procs:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
