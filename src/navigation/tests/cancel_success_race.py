"""Exercise the ordering of cancel requests and goal completion."""

import time
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from turtle_nav_interfaces.action import NavigateTurtle

rclpy.init()
node = Node('navigation_cancel_success_race')
client = ActionClient(node, NavigateTurtle, '/navigate_turtle')
assert client.wait_for_server(timeout_sec=8)

def wait(future, timeout=5):
    # The test client uses one thread, so spin it while waiting for ROS replies.
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.005)
    assert future.done(), 'timeout'
    return future.result()

counts = {GoalStatus.STATUS_SUCCEEDED: 0, GoalStatus.STATUS_CANCELED: 0}
try:
    # This is the Turtlesim spawn point, so the controller can finish immediately.
    # Rotate the delay to make the cancel arrive before or after that finish.
    for i in range(30):
        goal = NavigateTurtle.Goal()
        goal.x, goal.y = 5.544445, 5.544445
        handle = wait(client.send_goal_async(goal))
        assert handle.accepted, i
        time.sleep((0.0, 0.03, 0.12)[i % 3])
        cancel_reply = wait(handle.cancel_goal_async())
        wrapped = wait(handle.get_result_async())
        assert wrapped.status in counts, (i, wrapped.status, wrapped.result.message)
        # ROS accepts a cancel only while the goal can still enter CANCELING.
        # Once accepted, the terminal result must not switch back to success.
        if cancel_reply.goals_canceling:
            assert wrapped.status == GoalStatus.STATUS_CANCELED, (i, wrapped.status)
        else:
            assert wrapped.status == GoalStatus.STATUS_SUCCEEDED, (i, wrapped.status)
        counts[wrapped.status] += 1
    print('PASS 30 cancel/success races', counts, flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
