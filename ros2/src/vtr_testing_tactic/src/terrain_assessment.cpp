#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/pipeline_v2.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_tactic/modules/factory.hpp"

namespace fs = std::filesystem;
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

  // Parameters
  const unsigned run_id = node->declare_parameter<int>("run_id", 0);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

  // Localization chain
  auto chain = std::make_shared<tactic::LocalizationChain>(graph);

  // module
  auto module_factory = std::make_shared<ROSModuleFactory>(node);
  auto module = module_factory->get("localization.terrain_assessment");

  // get the path we should repeat
  VertexId::Vector sequence;
  sequence.reserve(graph->numberOfVertices());
  LOG(WARNING) << "Total number of vertices: " << graph->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  auto evaluator =
      std::make_shared<tactic::TemporalEvaluator<tactic::GraphBase>>(*graph);
  auto subgraph = graph->getSubgraph(tactic::VertexId(run_id, 0), evaluator);
  std::stringstream ss;
  ss << "Repeat vertices: ";
  for (auto it = subgraph->begin(tactic::VertexId(run_id, 0));
       it != subgraph->end(); ++it) {
    ss << it->v()->id() << " ";
    sequence.push_back(it->v()->id());
  }
  LOG(WARNING) << ss.str();
  chain->setSequence(sequence);
  chain->expand();

  lidar::LidarOutputCache output;
  output.chain = chain;

  size_t depth = 5;
  std::queue<tactic::VertexId> ids;

  for (unsigned i = 0; i < sequence.size(); ++i) {
    lidar::LidarQueryCache qdata;
    qdata.node = node;

    // current vertex to localize against
    qdata.map_id.emplace(sequence[i]);
    qdata.map_sid.emplace(i);
    // retrieve loc map
    auto vertex = graph->at(sequence[i]);
    const auto map_msg =
        vertex->retrieve<lidar::PointMap<lidar::PointWithInfo>>(
            "point_map", "vtr_lidar_msgs/msg/PointMap");
    auto locked_map_msg_ref = map_msg->locked();  // lock the msg
    auto &locked_map_msg = locked_map_msg_ref.get();
    qdata.curr_map_loc.emplace(locked_map_msg.getData());

    module->runAsync(qdata, output, graph, nullptr, {}, {});
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // memory management
    ids.push(sequence[i]);
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