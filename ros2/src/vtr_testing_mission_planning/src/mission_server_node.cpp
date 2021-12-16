#include "rclcpp/rclcpp.hpp"

#include "vtr_logging/logging_init.hpp"
#include "vtr_mission_planning_v2/test_utils.hpp"
#include "vtr_navigation_v2/ros_mission_server.hpp"

using namespace vtr;
using namespace vtr::logging;
using namespace vtr::navigation;

int main(int argc, char **argv) {
  configureLogging("", true);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("testing_mission_server");

  auto mission_server = std::make_shared<ROSMissionServer>();
  auto state_machine = std::make_shared<TestStateMachine>(mission_server);
  mission_server->start(node, state_machine);

  rclcpp::spin(node);
  rclcpp::shutdown();
}