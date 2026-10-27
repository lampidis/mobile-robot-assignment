#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtle_nav_interfaces/action/controller_navigate.hpp"
#include "turtlesim/msg/pose.hpp"

// Owns turtle motion. A single-threaded executor runs the callbacks and timer,
// so goal state is changed in one place at a time without a mutex.
class NavigationController : public rclcpp::Node
{
public:
  using ControllerNavigate = turtle_nav_interfaces::action::ControllerNavigate;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ControllerNavigate>;
  using SteadyClock = std::chrono::steady_clock;

  NavigationController()
  : rclcpp::Node("navigation_controller")
  {
    // YAML or command-line values override these defaults at node startup.
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.15);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    linear_gain_ = declare_parameter<double>("linear_gain", 1.5);
    angular_gain_ = declare_parameter<double>("angular_gain", 4.0);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 2.0);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.5);
    heading_gate_ = declare_parameter<double>("heading_gate", 0.35);
    pose_timeout_ = declare_parameter<double>("pose_timeout", 1.0);
    minimum_coordinate_ = declare_parameter<double>("minimum_coordinate", 0.5);
    maximum_coordinate_ = declare_parameter<double>("maximum_coordinate", 10.5);

    // Fail during startup instead of running a controller with unsafe settings.
    if (!std::isfinite(position_tolerance_) || position_tolerance_ <= 0.0 ||
      !std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0 ||
      !std::isfinite(linear_gain_) || linear_gain_ <= 0.0 ||
      !std::isfinite(angular_gain_) || angular_gain_ <= 0.0 ||
      !std::isfinite(max_linear_speed_) || max_linear_speed_ <= 0.0 ||
      !std::isfinite(max_angular_speed_) || max_angular_speed_ <= 0.0 ||
      !std::isfinite(heading_gate_) || heading_gate_ <= 0.0 || heading_gate_ >= 1.57 ||
      !std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0 ||
      !std::isfinite(minimum_coordinate_) || !std::isfinite(maximum_coordinate_) ||
      minimum_coordinate_ >= maximum_coordinate_)
    {
      throw std::invalid_argument("Navigation controller parameters must be finite and in range");
    }

    velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    // Turtlesim's Pose has no header stamp; receipt time is used to detect stale data.
    pose_subscription_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      [this](const turtlesim::msg::Pose::SharedPtr pose) {
        pose_ = *pose;
        pose_received_ = true;
        last_pose_time_ = SteadyClock::now();
      });

    // These callbacks decide whether to accept, cancel, or start a controller goal.
    // They return quickly; the timer below performs the actual movement.
    action_server_ = rclcpp_action::create_server<ControllerNavigate>(
      this, "/controller/navigate",
      std::bind(&NavigationController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigationController::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigationController::handle_accepted, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&NavigationController::control_tick, this));

    RCLCPP_INFO(get_logger(), "Controller ready on /controller/navigate");
  }

  ~NavigationController() override
  {
    // Best effort on orderly shutdown; an abrupt process failure cannot send this.
    if (active_goal_ && velocity_publisher_) {
      publish_zero_velocity();
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ControllerNavigate::Goal> goal)
  {
    // Reserve the slot now, before handle_accepted receives the new goal handle.
    // This also rejects direct clients that try to command conflicting motion.
    if (active_goal_ || goal_reserved_) {
      RCLCPP_WARN(get_logger(), "Rejecting controller goal while another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(goal->x) || !std::isfinite(goal->y) ||
      goal->x < minimum_coordinate_ || goal->x > maximum_coordinate_ ||
      goal->y < minimum_coordinate_ || goal->y > maximum_coordinate_)
    {
      RCLCPP_WARN(get_logger(), "Rejecting invalid controller target");
      return rclcpp_action::GoalResponse::REJECT;
    }

    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (goal_handle != active_goal_ || !goal_handle->is_active()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    // Accepting the request is not the terminal result. The next timer tick
    // publishes zero velocity before reporting CANCELED.
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Keep one active goal and stop any previous command while starting it.
    active_goal_ = goal_handle;
    goal_reserved_ = false;
    cancel_requested_ = false;
    goal_start_time_ = SteadyClock::now();
    publish_zero_velocity();
    RCLCPP_INFO(
      get_logger(), "Controller target accepted: (%.2f, %.2f)",
      goal_handle->get_goal()->x, goal_handle->get_goal()->y);
  }

  void control_tick()
  {
    if (!active_goal_) {
      return;
    }

    const auto now = SteadyClock::now();
    // Handle cancellation before success so this tick cannot finish both ways.
    if (cancel_requested_ || active_goal_->is_canceling()) {
      finish_goal(Outcome::Canceled, "Controller goal canceled");
      return;
    }

    // No pose means no safe motion. Wait briefly for startup, then abort.
    if (!pose_received_) {
      publish_zero_velocity();
      if (seconds_since(goal_start_time_, now) >= pose_timeout_) {
        finish_goal(Outcome::Aborted, "No turtle pose received before timeout");
      }
      return;
    }
    // A once-valid pose can become stale if Turtlesim stops publishing.
    if (seconds_since(last_pose_time_, now) >= pose_timeout_ ||
      !std::isfinite(pose_.x) || !std::isfinite(pose_.y) || !std::isfinite(pose_.theta))
    {
      finish_goal(Outcome::Aborted, "Turtle pose is stale or invalid");
      return;
    }

    const auto goal = active_goal_->get_goal();
    const double dx = goal->x - pose_.x;
    const double dy = goal->y - pose_.y;
    const double distance = std::hypot(dx, dy);
    // Final heading is not part of success; position alone determines arrival.
    if (distance <= position_tolerance_) {
      finish_goal(Outcome::Succeeded, "Target reached");
      return;
    }

    const double desired_heading = std::atan2(dy, dx);
    const double heading_difference = desired_heading - pose_.theta;
    // atan2(sin, cos) wraps the error into [-pi, pi] for the shortest turn.
    const double heading_error = std::atan2(
      std::sin(heading_difference), std::cos(heading_difference));

    geometry_msgs::msg::Twist command;
    // Proportional steering turns more strongly for larger heading errors.
    command.angular.z = std::clamp(
      angular_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);
    // Turn in place first; drive only when pointed roughly at the target.
    if (std::abs(heading_error) < heading_gate_) {
      command.linear.x = std::clamp(linear_gain_ * distance, 0.0, max_linear_speed_) *
        std::cos(heading_error);
    }
    velocity_publisher_->publish(command);

    // The action client receives distance feedback on every control tick.
    auto feedback = std::make_shared<ControllerNavigate::Feedback>();
    feedback->current_distance = static_cast<float>(distance);
    active_goal_->publish_feedback(feedback);
  }

  enum class Outcome {Succeeded, Aborted, Canceled};

  void finish_goal(Outcome outcome, const std::string & message)
  {
    // Publish zero before any terminal result. stop_confirmed records that
    // command publication, not a measured proof that the turtle is motionless.
    publish_zero_velocity();
    const auto goal_handle = active_goal_;
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ControllerNavigate::Result>();
    // -1 means a finite position was unavailable for the distance estimate.
    result->final_distance = pose_received_ &&
      std::isfinite(pose_.x) && std::isfinite(pose_.y) ?
      static_cast<float>(std::hypot(goal->x - pose_.x, goal->y - pose_.y)) : -1.0F;
    result->stop_confirmed = true;
    result->message = message;

    if (outcome == Outcome::Succeeded) {
      goal_handle->succeed(result);
    } else if (outcome == Outcome::Canceled) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    RCLCPP_INFO(get_logger(), "%s; final distance %.3f", message.c_str(), result->final_distance);
    active_goal_.reset();
    cancel_requested_ = false;
  }

  void publish_zero_velocity()
  {
    velocity_publisher_->publish(geometry_msgs::msg::Twist{});
  }

  static double seconds_since(SteadyClock::time_point start, SteadyClock::time_point now)
  {
    return std::chrono::duration<double>(now - start).count();
  }

  rclcpp_action::Server<ControllerNavigate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::shared_ptr<GoalHandle> active_goal_;
  turtlesim::msg::Pose pose_;
  SteadyClock::time_point last_pose_time_;
  SteadyClock::time_point goal_start_time_;
  bool pose_received_{false};
  bool goal_reserved_{false};
  bool cancel_requested_{false};
  double position_tolerance_;
  double control_rate_hz_;
  double linear_gain_;
  double angular_gain_;
  double max_linear_speed_;
  double max_angular_speed_;
  double heading_gate_;
  double pose_timeout_;
  double minimum_coordinate_;
  double maximum_coordinate_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // spin() uses a single-threaded executor here, matching the state model above.
  rclcpp::spin(std::make_shared<NavigationController>());
  rclcpp::shutdown();
  return 0;
}
