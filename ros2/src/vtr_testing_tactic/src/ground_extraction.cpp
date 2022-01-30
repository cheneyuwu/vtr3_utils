#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/pipeline_v2.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/modules/factory.hpp"

using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Configure logging
  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled = node->declare_parameter<std::vector<std::string>>(
      "log_enabled", std::vector<std::string>{});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

  // module
  auto module_factory = std::make_shared<ROSModuleFactory>(node);
  auto module = module_factory->get("localization.ground_extraction");

  // Parameters
  const unsigned run_id = node->declare_parameter<int>("run_id", 0);

  size_t depth = 5;
  std::queue<tactic::VertexId> ids;

  /// Create a temporal evaluator
  auto evaluator = std::make_shared<tactic::TemporalEvaluator<tactic::Graph>>();
  evaluator->setGraph(graph.get());

  auto subgraph = graph->getSubgraph(tactic::VertexId(run_id, 0), evaluator);
  for (auto it = subgraph->begin(tactic::VertexId(run_id, 0));
       it != subgraph->end(); ++it) {
    lidar::LidarQueryCache qdata;
    lidar::LidarOutputCache output;
    qdata.node = node;

    // current vertex to localize against
    qdata.map_id.emplace(it->v()->id());
    // retrieve loc map
    auto vertex = graph->at(it->v()->id());
    const auto map_msg =
        vertex->retrieve<lidar::PointMap<lidar::PointWithInfo>>(
            "point_map", "vtr_lidar_msgs/msg/PointMap");
    auto locked_map_msg_ref = map_msg->locked();  // lock the msg
    auto &locked_map_msg = locked_map_msg_ref.get();
    qdata.curr_map_loc.emplace(locked_map_msg.getData());

    module->runAsync(qdata, output, graph, nullptr, {}, {});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // memory management
    ids.push(it->v()->id());
    if (ids.size() > depth) {
      graph->at(ids.front())->unload();
      ids.pop();
    }

    if (!rclcpp::ok()) break;  // for ctrl-c
  }

  graph->save();
  graph.reset();

  LOG(WARNING) << "Map Saving done!";

  rclcpp::shutdown();
}