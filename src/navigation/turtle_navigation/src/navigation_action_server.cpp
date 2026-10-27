#include <memory>

#include "rclcpp/rclcpp.hpp"

// Step 04 creates a launchable node. Public action handling is added in step 06.
class NavigationActionServer : public rclcpp::Node
{
public:
  NavigationActionServer()
  : rclcpp::Node("navigation_action_server")
  {
    RCLCPP_INFO(get_logger(), "Navigation action server skeleton started; goals are not handled yet");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationActionServer>());
  rclcpp::shutdown();
  return 0;
}
