#include <memory>

#include "rclcpp/rclcpp.hpp"

// Step 04 creates a launchable node. Pose input and motion control are added in step 05.
class NavigationController : public rclcpp::Node
{
public:
  NavigationController()
  : rclcpp::Node("navigation_controller")
  {
    RCLCPP_INFO(get_logger(), "Navigation controller skeleton started; no velocity is published yet");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationController>());
  rclcpp::shutdown();
  return 0;
}
