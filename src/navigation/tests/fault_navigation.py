"""Use a controllable fake controller to exercise public action failure paths."""

import argparse
import subprocess
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from turtle_nav_interfaces.action import ControllerNavigate, NavigateTurtle

from test_support import installed_executable

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['reject', 'no_feedback', 'stop_timeout', 'preempt_stop_timeout', 'bad_ack', 'delayed_accept', 'delayed_stop', 'pending_cancel', 'controller_abort'])
mode = parser.parse_args().mode

rclpy.init()
# Keep the real public server, but replace its internal controller action with
# a fake whose responses can be rejected, delayed, or deliberately incomplete.
server_process = subprocess.Popen(
    [installed_executable('turtle_navigation', 'navigation_action_server')],
    stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

class FakeController(Node):
    def __init__(self):
        super().__init__('fake_controller')
        self.events = []
        self.live = True
        self.server = ActionServer(
            self, ControllerNavigate, '/controller/navigate',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup())

    def on_goal(self, request):
        self.events.append(('goal_request', time.monotonic(), request.x, request.y))
        if mode == 'reject':
            return GoalResponse.REJECT
        if mode == 'delayed_accept':
            # Longer than the public server's start timeout.
            time.sleep(3.0)
        self.events.append(('accepted', time.monotonic(), request.x, request.y))
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.events.append(('cancel_request', time.monotonic()))
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        started = time.monotonic()
        if mode in ('bad_ack', 'controller_abort'):
            # Finish the internal action promptly, varying either its status
            # or its explicit stop acknowledgment.
            time.sleep(0.1)
            result = ControllerNavigate.Result()
            result.final_distance = 1.0
            result.stop_confirmed = mode != 'bad_ack'
            result.message = 'synthetic result'
            if mode == 'bad_ack':
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        if mode == 'delayed_accept':
            # The public server times out first, then must cancel a late accept.
            deadline = time.monotonic() + 5.0
            while self.live and time.monotonic() < deadline and not goal_handle.is_cancel_requested:
                time.sleep(0.05)
            self.events.append(('late_cancel_seen', time.monotonic(), goal_handle.is_cancel_requested))
        elif mode in ('stop_timeout', 'preempt_stop_timeout'):
            # Feedback keeps the goal alive, but cancel never confirms a stop.
            while self.live:
                feedback = ControllerNavigate.Feedback()
                feedback.current_distance = 5.0
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)
            goal_handle.abort()
            return ControllerNavigate.Result()
        else:
            while self.live and not goal_handle.is_cancel_requested:
                # Suppressing feedback triggers the public feedback timeout.
                if mode != 'no_feedback':
                    feedback = ControllerNavigate.Feedback()
                    feedback.current_distance = 5.0
                    goal_handle.publish_feedback(feedback)
                time.sleep(0.05)
            if mode in ('delayed_stop', 'pending_cancel'):
                # Leave a window for a replacement or pending cancellation.
                time.sleep(0.6)
        self.events.append(('stopped', time.monotonic(), started))
        result = ControllerNavigate.Result()
        result.final_distance = 4.0
        result.stop_confirmed = True
        result.message = 'synthetic stop'
        goal_handle.canceled()
        return result

fake = FakeController()
client_node = Node('fault_test_client')
client = ActionClient(client_node, NavigateTurtle, '/navigate_turtle')
executor = MultiThreadedExecutor(num_threads=4)
# The fake action callback may sleep while the client and cancel callbacks run.
# Multiple executor threads let those callbacks overlap during the test.
executor.add_node(fake)
executor.add_node(client_node)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()


def wait(future, timeout=10.0):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        time.sleep(0.02)
    assert future.done(), f'{mode}: future timed out'
    return future.result()


def send(x, y):
    goal = NavigateTurtle.Goal()
    goal.x, goal.y = float(x), float(y)
    handle = wait(client.send_goal_async(goal))
    return handle


def result(handle):
    return wait(handle.get_result_async())

try:
    assert client.wait_for_server(timeout_sec=8.0), 'public action unavailable'
    if mode == 'delayed_stop':
        # A is active, B waits, then C replaces B. C may dispatch only after
        # the fake records A's stop confirmation.
        a = send(8.0, 5.5)
        assert a.accepted
        time.sleep(0.2)
        b = send(2.0, 2.0)
        assert b.accepted
        c = send(3.0, 3.0)
        assert c.accepted
        ra, rb = result(a), result(b)
        assert ra.status == GoalStatus.STATUS_ABORTED
        assert rb.status == GoalStatus.STATUS_ABORTED
        # C has been sent to the fake controller only after A acknowledged its stop.
        end = time.monotonic() + 3.0
        while len([e for e in fake.events if e[0] == 'accepted']) < 2 and time.monotonic() < end:
            time.sleep(0.02)
        accepted = [e for e in fake.events if e[0] == 'accepted']
        stopped = [e for e in fake.events if e[0] == 'stopped']
        assert len(accepted) == 2 and stopped and accepted[1][1] >= stopped[0][1], fake.events
        cancel_reply = wait(c.cancel_goal_async())
        assert cancel_reply.goals_canceling
        assert result(c).status == GoalStatus.STATUS_CANCELED
        print('PASS delayed stop acknowledgment gates next dispatch and A/B/C results', flush=True)
    elif mode == 'pending_cancel':
        # Cancel B before it reaches the controller; only A should be accepted.
        a = send(8.0, 5.5)
        assert a.accepted
        time.sleep(0.2)
        b = send(2.0, 2.0)
        assert b.accepted
        assert wait(b.cancel_goal_async()).goals_canceling
        assert result(b).status == GoalStatus.STATUS_CANCELED
        assert result(a).status == GoalStatus.STATUS_ABORTED
        accepted = [event for event in fake.events if event[0] == 'accepted']
        assert len(accepted) == 1, fake.events
        print('PASS pending goal cancellation without controller dispatch', flush=True)
    elif mode == 'preempt_stop_timeout':
        # If A never confirms its stop, neither B nor a later goal may run.
        a = send(8.0, 5.5)
        assert a.accepted
        time.sleep(0.2)
        b = send(2.0, 2.0)
        assert b.accepted
        ra, rb = result(a), result(b)
        assert ra.status == GoalStatus.STATUS_ABORTED and rb.status == GoalStatus.STATUS_ABORTED
        assert 'did not confirm stop' in ra.result.message.lower()
        assert 'did not confirm stop' in rb.result.message.lower()
        assert not send(3.0, 3.0).accepted
        assert len([e for e in fake.events if e[0] == 'accepted']) == 1
        print('PASS preemption stop timeout aborts pending goal and blocks new goals', flush=True)
    else:
        first = send(8.0, 5.5)
        assert first.accepted
        if mode == 'stop_timeout':
            time.sleep(0.2)
            assert wait(first.cancel_goal_async()).goals_canceling
        first_result = result(first)
        print(mode, first_result.status, first_result.result.message, flush=True)
        if mode == 'reject':
            assert first_result.status == GoalStatus.STATUS_ABORTED
            assert 'rejected' in first_result.result.message.lower()
            second = send(2.0, 2.0)
            assert second.accepted and result(second).status == GoalStatus.STATUS_ABORTED
            boundary = send(0.5, 10.5)
            assert boundary.accepted and result(boundary).status == GoalStatus.STATUS_ABORTED
        elif mode == 'no_feedback':
            assert first_result.status == GoalStatus.STATUS_ABORTED
            assert 'unresponsive' in first_result.result.message.lower()
            second = send(2.0, 2.0)
            assert second.accepted and result(second).status == GoalStatus.STATUS_ABORTED
        elif mode == 'controller_abort':
            assert first_result.status == GoalStatus.STATUS_ABORTED
            assert 'synthetic result' in first_result.result.message
            second = send(2.0, 2.0)
            assert second.accepted and result(second).status == GoalStatus.STATUS_ABORTED
        elif mode == 'stop_timeout':
            # ROS requires an accepted cancel to finish CANCELED, while the
            # result message still reports that stopping was not confirmed.
            assert first_result.status == GoalStatus.STATUS_CANCELED
            assert 'did not confirm stop' in first_result.result.message.lower()
            assert not send(2.0, 2.0).accepted
        elif mode == 'bad_ack':
            assert first_result.status == GoalStatus.STATUS_ABORTED
            assert 'did not confirm' in first_result.result.message.lower()
            assert not send(2.0, 2.0).accepted
        elif mode == 'delayed_accept':
            # The late internal goal must receive a compensating cancel.
            assert first_result.status == GoalStatus.STATUS_ABORTED
            assert 'acceptance timed out' in first_result.result.message.lower()
            assert not send(2.0, 2.0).accepted
            end = time.monotonic() + 4.0
            while not any(e[0] == 'late_cancel_seen' for e in fake.events) and time.monotonic() < end:
                time.sleep(0.02)
            late = [e for e in fake.events if e[0] == 'late_cancel_seen']
            assert late and late[0][2], fake.events
        print('PASS', mode, flush=True)
finally:
    fake.live = False
    server_process.terminate()
    try:
        server_process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        server_process.kill()
        server_process.wait(timeout=2.0)
    executor.shutdown(timeout_sec=2.0)
    thread.join(timeout=2.0)
    fake.server.destroy()
    fake.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()
