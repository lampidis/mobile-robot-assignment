#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtle_nav_interfaces/action/controller_navigate.hpp"
#include "turtle_nav_interfaces/action/navigate_turtle.hpp"

// This node has two ROS roles:
//   server on /navigate_turtle: receives goals from the user/client;
//   client on /controller/navigate: asks the motion controller to drive.
// The public goal and its controller goal are separate action goals. Their
// handles let us send each client the right feedback and final result.
// One executor thread owns the state below. Controller calls are asynchronous,
// so public goal/cancel callbacks never wait for the turtle to move or stop.
class NavigationActionServer : public rclcpp::Node
{
public:
  using NavigateTurtle = turtle_nav_interfaces::action::NavigateTurtle;
  using ControllerNavigate = turtle_nav_interfaces::action::ControllerNavigate;
  // We are the server for the public goal but a client of the controller goal.
  using PublicGoalHandle = rclcpp_action::ServerGoalHandle<NavigateTurtle>;
  using PublicGoalHandlePtr = std::shared_ptr<PublicGoalHandle>;
  using ControllerGoalHandle = rclcpp_action::ClientGoalHandle<ControllerNavigate>;
  using ControllerClient = rclcpp_action::Client<ControllerNavigate>;
  using SteadyClock = std::chrono::steady_clock;
  using TimePoint = SteadyClock::time_point;

  NavigationActionServer()
  : rclcpp::Node("navigation_action_server")
  {
    // The YAML file supplies these values at launch; the numbers here are defaults.
    minimum_coordinate_ = declare_parameter<double>("minimum_coordinate", 0.5);
    maximum_coordinate_ = declare_parameter<double>("maximum_coordinate", 10.5);
    start_timeout_ = declare_parameter<double>("controller_start_timeout", 2.0);
    stop_timeout_ = declare_parameter<double>("controller_stop_timeout", 2.0);
    feedback_timeout_ = declare_parameter<double>("controller_feedback_timeout", 2.0);
    if (!std::isfinite(minimum_coordinate_) || !std::isfinite(maximum_coordinate_) ||
      minimum_coordinate_ >= maximum_coordinate_ ||
      !std::isfinite(start_timeout_) || start_timeout_ <= 0.0 ||
      !std::isfinite(stop_timeout_) || stop_timeout_ <= 0.0 ||
      !std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Navigation action server parameters must be finite and in range");
    }

    // The client sends internal goals; the server receives public goals.
    controller_client_ = rclcpp_action::create_client<ControllerNavigate>(this, "/controller/navigate");
    public_server_ = rclcpp_action::create_server<NavigateTurtle>(
      this, "/navigate_turtle",
      std::bind(&NavigationActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigationActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigationActionServer::handle_accepted, this, std::placeholders::_1));
    // A short timer advances queued goals and checks deadlines without blocking callbacks.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&NavigationActionServer::tick, this));
    RCLCPP_INFO(get_logger(), "Public navigation action ready on /navigate_turtle");
  }

private:
  // Stage describes where the single active goal is in the controller protocol.
  enum class Stage {WaitingForServer, AwaitingAcceptance, Running, Stopping};
  // The first stop reason wins: a later request cannot change its final meaning.
  enum class StopReason {None, ClientCancel, Preempted, Unresponsive};
  // Outcome is the terminal state we send to the public action client.
  enum class Outcome {Succeeded, Aborted, Canceled};

  struct ActiveGoal
  {
    // The public handle exists as soon as we accept the user's request.
    PublicGoalHandlePtr public_handle;
    // This appears only after the controller accepts our internal request.
    ControllerGoalHandle::SharedPtr controller_handle;
    // Local dispatch number, used to ignore callbacks from an older goal.
    std::uint64_t token;
    Stage stage{Stage::WaitingForServer};
    StopReason stop_reason{StopReason::None};
    TimePoint start_deadline;
    TimePoint stop_deadline;
    TimePoint last_feedback_time;
  };

  static TimePoint after_seconds(TimePoint now, double seconds)
  {
    return now + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(seconds));
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateTurtle::Goal> goal)
  {
    // This callback only decides acceptance. It does not start motion, and ROS
    // has not given us a public goal handle yet. Invalid targets are rejected.
    if (motion_blocked_ || !std::isfinite(goal->x) || !std::isfinite(goal->y) ||
      goal->x < minimum_coordinate_ || goal->x > maximum_coordinate_ ||
      goal->y < minimum_coordinate_ || goal->y > maximum_coordinate_)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const PublicGoalHandlePtr goal_handle)
  {
    // ROS identifies exactly which public goal the client wants to cancel.
    // A pending goal has not reached the controller and can finish directly.
    if (pending_ == goal_handle && !pending_cancel_requested_) {
      pending_cancel_requested_ = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    if (active_ && active_->public_handle == goal_handle &&
      active_->stop_reason == StopReason::None)
    {
      // The active goal may already be moving. Remember the request now; the
      // timer sends a controller cancel after ROS marks this goal CANCELING.
      request_stop(StopReason::ClientCancel);
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted(const PublicGoalHandlePtr goal_handle)
  {
    // ROS calls this after accepting the request and creating its goal handle.
    // pending_ holds only the newest goal waiting for the controller slot.
    // A superseded pending goal never reaches the controller.
    if (pending_) {
      finish_public(
        pending_, pending_cancel_requested_ ? Outcome::Canceled : Outcome::Aborted,
        -1.0F, pending_cancel_requested_ ? "Goal canceled before dispatch" :
        "Superseded by a newer goal before dispatch");
    }
    pending_ = goal_handle;
    pending_cancel_requested_ = false;
    if (active_ && active_->stop_reason == StopReason::None) {
      // Do not dispatch the newcomer yet. First get a stop result for the
      // previous controller goal, then the timer can promote pending_ to active_.
      request_stop(StopReason::Preempted);
    }
  }

  void request_stop(StopReason reason)
  {
    // Record a stop decision once. A cancel and a newer goal can arrive close
    // together; the first callback handled by this executor decides the outcome.
    if (!active_ || active_->stop_reason != StopReason::None) {
      return;
    }
    active_->stop_reason = reason;
    active_->stop_deadline = after_seconds(SteadyClock::now(), stop_timeout_);
  }

  void tick()
  {
    const auto now = SteadyClock::now();
    // Canceling a queued goal needs no controller message: it was not sent yet.
    if (pending_ && pending_cancel_requested_) {
      finish_public(pending_, Outcome::Canceled, -1.0F, "Goal canceled before dispatch");
      pending_.reset();
      pending_cancel_requested_ = false;
    }

    // At most one public goal owns the controller slot. Each stage has one
    // permitted next action and a deadline for missing controller responses.
    if (active_) {
      switch (active_->stage) {
        case Stage::WaitingForServer:
          if (active_->stop_reason != StopReason::None) {
            // Nothing reached the controller, so there is no motion to stop.
            finish_stopped_active(-1.0F, "Goal stopped before controller dispatch");
          } else if (now >= active_->start_deadline) {
            finish_active(Outcome::Aborted, -1.0F, "Controller unavailable before start timeout");
          } else if (controller_client_->action_server_is_ready()) {
            send_controller_goal(now);
          }
          break;
        case Stage::AwaitingAcceptance:
          // We sent a goal request but do not yet know whether motion began.
          // If that answer never arrives, future goals are blocked for safety.
          if (active_->stop_reason != StopReason::None && now >= active_->stop_deadline) {
            block_motion("Controller goal acceptance unknown after stop timeout");
          } else if (now >= active_->start_deadline) {
            block_motion("Controller goal acceptance timed out; motion state unknown");
          }
          break;
        case Stage::Running:
          // The controller normally sends distance feedback each control tick.
          // Missing feedback requests a stop rather than leaving motion unobserved.
          if (active_->stop_reason == StopReason::None &&
            std::chrono::duration<double>(now - active_->last_feedback_time).count() >= feedback_timeout_)
          {
            request_stop(StopReason::Unresponsive);
          }
          if (active_->stop_reason != StopReason::None) {
            send_controller_cancel();
          }
          break;
        case Stage::Stopping:
          // A cancel request alone is not proof of a stop; wait for its result.
          if (now >= active_->stop_deadline) {
            block_motion("Controller did not confirm stop before timeout");
          }
          break;
      }
    }

    if (!active_ && pending_ && !motion_blocked_) {
      // Move the newest waiting public goal into the only active slot. The
      // controller request is sent on a later tick when its server is ready.
      active_ = std::make_unique<ActiveGoal>();
      active_->public_handle = pending_;
      active_->token = ++next_token_;
      active_->start_deadline = after_seconds(now, start_timeout_);
      pending_.reset();
      RCLCPP_INFO(get_logger(), "Public goal accepted: (%.2f, %.2f)",
        active_->public_handle->get_goal()->x, active_->public_handle->get_goal()->y);
    }
  }

  void send_controller_goal(TimePoint now)
  {
    // Copy the public target into a distinct internal action request. No ROS
    // goal handle exists for it until the controller replies with acceptance.
    const auto token = active_->token;
    const auto public_goal = active_->public_handle->get_goal();
    ControllerNavigate::Goal controller_goal;
    controller_goal.x = public_goal->x;
    controller_goal.y = public_goal->y;

    // ROS will call these three functions later: acceptance, progress, result.
    // token links those callbacks to this dispatch, even if a newer public
    // goal has replaced it before a delayed callback arrives.
    ControllerClient::SendGoalOptions options;
    options.goal_response_callback = [this, token](ControllerGoalHandle::SharedPtr handle) {
        on_controller_response(token, handle);
      };
    options.feedback_callback = [this, token](
      ControllerGoalHandle::SharedPtr handle,
      const std::shared_ptr<const ControllerNavigate::Feedback> feedback)
      {
        on_controller_feedback(token, handle, feedback);
      };
    options.result_callback = [this, token](const ControllerGoalHandle::WrappedResult & result) {
        on_controller_result(token, result);
      };

    active_->stage = Stage::AwaitingAcceptance;
    active_->start_deadline = after_seconds(now, start_timeout_);
    try {
      controller_client_->async_send_goal(controller_goal, options);
    } catch (const std::exception & error) {
      // The request may have left this process before the exception was raised.
      block_motion(std::string("Could not send controller goal: ") + error.what());
    }
  }

  void on_controller_response(std::uint64_t token, ControllerGoalHandle::SharedPtr handle)
  {
    // A null handle means the controller rejected our internal goal.
    // A mismatched token belongs to an earlier public goal or a timed-out send.
    if (!active_ || active_->token != token) {
      // A response after a timeout must not leave an orphaned goal moving.
      if (handle) {
        try {
          controller_client_->async_cancel_goal(handle);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Could not cancel late controller goal: %s", error.what());
        }
      }
      return;
    }
    if (!handle) {
      finish_active(Outcome::Aborted, -1.0F, "Controller rejected the goal");
      return;
    }
    active_->controller_handle = handle;
    active_->last_feedback_time = SteadyClock::now();
    active_->stage = Stage::Running;
    if (active_->stop_reason != StopReason::None) {
      send_controller_cancel();
    }
  }

  void on_controller_feedback(
    std::uint64_t token, ControllerGoalHandle::SharedPtr handle,
    const std::shared_ptr<const ControllerNavigate::Feedback> feedback)
  {
    // Forward only feedback from the current internal handle to its matching
    // public handle. Stale or already-stopping goals cannot report progress.
    if (!active_ || active_->token != token || !active_->controller_handle ||
      handle != active_->controller_handle || active_->stop_reason != StopReason::None)
    {
      return;
    }
    active_->last_feedback_time = SteadyClock::now();
    auto public_feedback = std::make_shared<NavigateTurtle::Feedback>();
    public_feedback->current_distance = feedback->current_distance;
    active_->public_handle->publish_feedback(public_feedback);
  }

  void send_controller_cancel()
  {
    // Asking to cancel is asynchronous. The public goal remains active until
    // the controller sends a terminal result with stop_confirmed=true.
    // Stopping also records that the cancel request was already sent.
    if (!active_ || !active_->controller_handle || active_->stage != Stage::Running) {
      return;
    }
    active_->stage = Stage::Stopping;
    try {
      controller_client_->async_cancel_goal(active_->controller_handle);
    } catch (const std::exception & error) {
      // A result may still arrive. The deadline handles a missing acknowledgment.
      RCLCPP_ERROR(get_logger(), "Could not request controller stop: %s", error.what());
    }
  }

  void on_controller_result(
    std::uint64_t token, const ControllerGoalHandle::WrappedResult & wrapped)
  {
    // The token and ROS goal ID prevent an old controller result from finishing
    // a newer public goal. A valid result must acknowledge zero velocity.
    if (!active_ || active_->token != token ||
      (active_->controller_handle && wrapped.goal_id != active_->controller_handle->get_goal_id()))
    {
      return;
    }
    if (!wrapped.result || !wrapped.result->stop_confirmed) {
      block_motion("Controller result did not confirm a zero velocity command");
      return;
    }

    const float distance = wrapped.result->final_distance;
    // If cancellation/preemption was recorded first, it decides the public
    // outcome even if the controller happened to finish successfully meanwhile.
    if (active_->stop_reason != StopReason::None) {
      finish_stopped_active(distance, wrapped.result->message);
    } else if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      finish_active(Outcome::Succeeded, distance, wrapped.result->message);
    } else {
      finish_active(Outcome::Aborted, distance, wrapped.result->message);
    }
  }

  void finish_stopped_active(float distance, const std::string & controller_message)
  {
    // A client cancellation is CANCELED; replacement by a newer goal is ABORTED.
    // Both paths reach here only after no internal goal was sent or a stop result arrived.
    if (active_->stop_reason == StopReason::ClientCancel) {
      finish_active(Outcome::Canceled, distance, "Goal canceled; " + controller_message);
    } else if (active_->stop_reason == StopReason::Preempted) {
      finish_active(Outcome::Aborted, distance, "Preempted by newer goal; " + controller_message);
    } else {
      finish_active(Outcome::Aborted, distance, "Controller became unresponsive; " + controller_message);
    }
  }

  void finish_active(Outcome outcome, float distance, const std::string & message)
  {
    // Release the active slot before reporting the public terminal result.
    const auto handle = active_->public_handle;
    active_.reset();
    finish_public(handle, outcome, distance, message);
  }

  void finish_public(
    const PublicGoalHandlePtr & handle, Outcome outcome, float distance,
    const std::string & message)
  {
    // Every accepted public goal gets exactly one terminal result. A distance
    // of -1 means we have no controller measurement for this public goal.
    auto result = std::make_shared<NavigateTurtle::Result>();
    result->final_distance = distance;
    result->message = message;
    // ROS goals in CANCELING must finish with canceled(), even on stop failure.
    // The error message distinguishes an unconfirmed stop from a normal cancel.
    if (handle->is_canceling() || outcome == Outcome::Canceled) {
      handle->canceled(result);
    } else if (outcome == Outcome::Succeeded) {
      handle->succeed(result);
    } else {
      handle->abort(result);
    }
    RCLCPP_INFO(get_logger(), "%s; final distance %.3f", message.c_str(), distance);
  }

  void block_motion(const std::string & reason)
  {
    // An unanswered request could have started motion. Reject new goals until
    // this server and the controller have been restarted.
    motion_blocked_ = true;
    RCLCPP_ERROR(get_logger(), "%s; rejecting new goals until restart", reason.c_str());
    if (active_) {
      finish_active(Outcome::Aborted, -1.0F, reason);
    }
    if (pending_) {
      finish_public(pending_, Outcome::Aborted, -1.0F, reason);
      pending_.reset();
      pending_cancel_requested_ = false;
    }
  }

  // These are node members, not global variables. Only this node's executor
  // accesses them. active_ owns the current dispatch; pending_ stores one
  // newer public goal; neither is a second moving controller goal.
  rclcpp_action::Server<NavigateTurtle>::SharedPtr public_server_;
  ControllerClient::SharedPtr controller_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ActiveGoal> active_;
  PublicGoalHandlePtr pending_;
  std::uint64_t next_token_{0};
  bool pending_cancel_requested_{false};
  bool motion_blocked_{false};
  double minimum_coordinate_;
  double maximum_coordinate_;
  double start_timeout_;
  double stop_timeout_;
  double feedback_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationActionServer>());
  rclcpp::shutdown();
  return 0;
}
