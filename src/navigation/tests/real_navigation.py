"""Exercise normal navigation and goal lifecycle against real Turtlesim."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from turtle_nav_interfaces.action import ControllerNavigate, NavigateTurtle

rclpy.init()
node = Node('navigation_real_edge_test')
public = ActionClient(node, NavigateTurtle, '/navigate_turtle')
internal = ActionClient(node, ControllerNavigate, '/controller/navigate')
commands = []
# Observe the controller's actual output for final-zero and speed-limit checks.
node.create_subscription(Twist, '/turtle1/cmd_vel', lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10)
assert public.wait_for_server(timeout_sec=8.0)
assert internal.wait_for_server(timeout_sec=8.0)


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def wait(future, timeout=25.0):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert future.done(), 'future timed out'
    return future.result()


def request(x, y, action_type=NavigateTurtle):
    goal = action_type.Goal()
    goal.x, goal.y = float(x), float(y)
    return goal


def send(client, x, y, feedback_callback=None, action_type=NavigateTurtle):
    handle = wait(client.send_goal_async(request(x, y, action_type), feedback_callback=feedback_callback))
    return handle


def result(handle):
    return wait(handle.get_result_async())


def expect_zero(label):
    # Allow the final zero command to arrive through the ROS subscription.
    spin_for(0.15)
    assert commands, f'{label}: no velocity commands observed'
    assert abs(commands[-1][0]) < 1e-6 and abs(commands[-1][1]) < 1e-6, f'{label}: final command {commands[-1]}'

try:
    # First move checks the complete public path: goal, feedback, result, stop.
    feedback = []
    handle = send(public, 8.0, 5.5, lambda msg: feedback.append(msg.feedback.current_distance))
    assert handle.accepted
    done = result(handle)
    assert done.status == GoalStatus.STATUS_SUCCEEDED and done.result.final_distance <= 0.15
    assert feedback and all(math.isfinite(x) for x in feedback)
    expect_zero('success')
    print('PASS success, feedback, tolerance, zero velocity', len(feedback), done.result.final_distance, flush=True)

    # Repeating the reached target exercises immediate completion.
    handle = send(public, 8.0, 5.5)
    assert handle.accepted and result(handle).status == GoalStatus.STATUS_SUCCEEDED
    expect_zero('already at target')
    print('PASS already at target', flush=True)

    # Invalid values should be rejected before any controller goal is sent.
    for x, y in [(float('nan'), 5.5), (float('inf'), 5.5), (float('-inf'), 5.5),
                 (5.5, float('nan')), (0.49, 5.5), (10.51, 5.5),
                 (5.5, 0.49), (5.5, 10.51)]:
        assert not send(public, x, y).accepted, f'invalid goal accepted: {x}, {y}'
    print('PASS NaN, infinities and all coordinate bounds rejected', flush=True)

    # B and C arrive while A moves. Only the newest pending goal, C, may run.
    a = send(public, 2.0, 1.5)
    assert a.accepted
    spin_for(0.15)
    b_future = public.send_goal_async(request(9.0, 9.0))
    c_future = public.send_goal_async(request(3.0, 8.0))
    b, c = wait(b_future), wait(c_future)
    assert b.accepted and c.accepted
    results = [result(h) for h in (a, b, c)]
    assert [r.status for r in results] == [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_SUCCEEDED]
    assert 'Preempted' in results[0].result.message
    assert 'Superseded' in results[1].result.message
    expect_zero('A/B/C')
    print('PASS latest-goal-wins A/B/C and final zero velocity', flush=True)

    # Client cancellation must wait for a controller stop and end CANCELED.
    d = send(public, 9.0, 2.0)
    assert d.accepted
    spin_for(0.15)
    assert wait(d.cancel_goal_async()).goals_canceling
    assert result(d).status == GoalStatus.STATUS_CANCELED
    expect_zero('active cancellation')
    print('PASS active cancellation and final zero velocity', flush=True)

    # A direct internal client holds the controller slot, so the public dispatch is rejected.
    direct = send(internal, 9.0, 2.0, action_type=ControllerNavigate)
    assert direct.accepted
    public_handle = send(public, 8.0, 8.0)
    assert public_handle.accepted
    rejected = result(public_handle)
    assert rejected.status == GoalStatus.STATUS_ABORTED and 'rejected' in rejected.result.message.lower()
    wait(direct.cancel_goal_async())
    assert result(direct).status == GoalStatus.STATUS_CANCELED
    expect_zero('internal conflict')
    print('PASS controller rejection when directly occupied', flush=True)

    for _ in range(10):
        # Cancel immediately after acceptance; success may already have won.
        h = send(public, 3.0, 8.0)
        assert h.accepted
        cancel = wait(h.cancel_goal_async())
        r = result(h)
        if cancel.goals_canceling:
            assert r.status == GoalStatus.STATUS_CANCELED
        else:
            assert r.status == GoalStatus.STATUS_SUCCEEDED
    expect_zero('near-success cancel race')
    print('PASS near-success cancellation race (10 runs)', flush=True)

    # Compare every observed command against the configured output limits.
    assert all(abs(linear) <= 2.00001 and abs(angular) <= 2.50001 for linear, angular in commands)
    print('PASS velocity bounds', len(commands), 'commands', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
