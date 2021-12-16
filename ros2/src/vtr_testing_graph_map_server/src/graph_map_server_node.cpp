#include "rclcpp/rclcpp.hpp"

#include "vtr_logging/logging_init.hpp"
#include "vtr_navigation_v2/graph_map_server.hpp"

using namespace vtr;
using namespace vtr::logging;
using namespace vtr::navigation;

int main(int argc, char **argv) {
  configureLogging("", true);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("testing_graph_map_server");

  auto graph_map_server = std::make_shared<GraphMapServer>();
  auto graph =
      tactic::Graph::MakeShared("/tmp/vtr_testing_graph_map_server", false);
  graph_map_server->start(node, graph);

  rclcpp::spin(node);
  rclcpp::shutdown();
}