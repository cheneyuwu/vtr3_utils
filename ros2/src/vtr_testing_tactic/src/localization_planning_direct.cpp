#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"

#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/path_planning.hpp"
#include "vtr_lidar/pipeline_v2.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_path_planning/factory.hpp"
#include "vtr_path_planning/path_planning.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"

using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::path_planning;

namespace vtr {
namespace path_planning {

class TestCommandPublisher : public BasePathPlanner::Callback {
 public:
  PTR_TYPEDEFS(TestCommandPublisher);

  TestCommandPublisher(const rclcpp::Node::SharedPtr &node) : node_(node) {
    command_pub_ = node->create_publisher<Command>("command", 10);
  }

  tactic::Timestamp getCurrentTime() const override {
    return node_->now().nanoseconds();
  }

  void commandReceived(const Command &command) override {
    CLOG(DEBUG, "path_planning")
        << "Received control command: [" << command.linear.x << ", "
        << command.linear.y << ", " << command.linear.z << ", "
        << command.angular.x << ", " << command.angular.y << ", "
        << command.angular.z << "]";
    command_pub_->publish(command);
  }

 private:
  const rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;
};

}  // namespace path_planning
}  // namespace vtr

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  // odometry sequence directory
  const auto odo_dir_str =
      node->declare_parameter<std::string>("odo_dir", "/tmp");
  fs::path odo_dir{utils::expand_user(utils::expand_env(odo_dir_str))};

  // localization sequence directory
  const auto loc_dir_str =
      node->declare_parameter<std::string>("loc_dir", "/tmp");
  fs::path loc_dir{utils::expand_user(utils::expand_env(loc_dir_str))};

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

  CLOG(WARNING, "test") << "Odometry Directory: " << odo_dir.string();
  CLOG(WARNING, "test") << "Localization Directory: " << loc_dir.string();
  CLOG(WARNING, "test") << "Output Directory: " << data_dir.string();

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

  // Pipeline
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node);
  auto pipeline = pipeline_factory->get("pipeline");
  auto pipeline_output = pipeline->createOutputCache();
  // some modules require node for visualization
  pipeline_output->node = node;

  // Tactic Callback
  auto callback = std::make_shared<RvizTacticCallback>(node);

  // Tactic
  auto tactic =
      std::make_shared<Tactic>(Tactic::Config::fromROS(node), pipeline,
                               pipeline_output, graph, callback);
  tactic->setPipeline(PipelineMode::RepeatFollow);
  tactic->addRun();

  // Planner
  auto planner_factory = std::make_shared<ROSPathPlannerFactory>(node);
  auto planner =
      planner_factory->get("path_planning", pipeline_output,
                           std::make_shared<TestCommandPublisher>(node));
  planner->setRunning(true);

  // Get the path that we should repeat
  VertexId::Vector sequence;
  sequence.reserve(graph->numberOfVertices());
  CLOG(WARNING, "test") << "Total number of vertices: "
                        << graph->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  using LocEvaluator =
      pose_graph::eval::Mask::Privileged<tactic::Graph>::Caching;
  LocEvaluator::Ptr evaluator(new LocEvaluator());
  evaluator->setGraph(graph.get());
  auto privileged_path = graph->getSubgraph(0ul, evaluator);
  std::stringstream ss;
  ss << "Repeat vertices: ";
  for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
       ++it) {
    ss << it->v()->id() << " ";
    sequence.push_back(it->v()->id());
  }
  CLOG(WARNING, "test") << ss.str();

  tactic->setPath(sequence);

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "honeycomb";

  /// robot lidar transformation is hard-coded - check measurements.
  Eigen::Matrix4d T_lidar_robot_mat;
  T_lidar_robot_mat << 1, 0, 0, -0.06, 0, 1, 0, 0, 0, 0, 1, -1.45, 0, 0, 0, 1;
  EdgeTransform T_lidar_robot(T_lidar_robot_mat);
  T_lidar_robot.setZeroCovariance();
  CLOG(WARNING, "test") << "Transform from " << robot_frame << " to "
                        << lidar_frame << " has been set to" << T_lidar_robot;

  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg =
      tf2::eigenToTransform(Eigen::Affine3d(T_lidar_robot.inverse().matrix()));
  msg.header.frame_id = robot_frame;
  msg.child_frame_id = lidar_frame;
  tf_sbc->sendTransform(msg);

  const auto clock_publisher =
      node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // Load dataset
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = loc_dir.string();
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;  // default
  storage_options.max_cache_size = 0;    // default
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back("/points");

  rosbag2_cpp::Reader reader;
  reader.open(storage_options, converter_options);
  reader.set_filter(filter);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  using common::timing::Stopwatch;
  Stopwatch timer(false);

  int frame = 0;
  int terrain_type = 0;
  while (rclcpp::ok() && reader.has_next()) {
    // load rosbag2 message
    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage msg(*bag_message->serialized_data);
    auto points = std::make_shared<sensor_msgs::msg::PointCloud2>();
    serializer.deserialize_message(&msg, points.get());

    CLOG(WARNING, "test") << "Loading point cloud frame " << frame
                          << " with timestamp "
                          << (unsigned long)(points->header.stamp.sec * 1e9 +
                                             points->header.stamp.nanosec);
    timer.start();

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = points->header.stamp;
    clock_publisher->publish(time_msg);

    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<lidar::LidarQueryCache>();

    // some modules require node for visualization
    query_data->node = node;

    // set timestamp
    storage::Timestamp timestamp =
        points->header.stamp.sec * 1e9 + points->header.stamp.nanosec;
    query_data->stamp.emplace(timestamp);

    // make up some environment info
    tactic::EnvInfo env_info;
    env_info.terrain_type = terrain_type;
    query_data->env_info.emplace(env_info);

    // put in the pointcloud msg pointer into query data
    query_data->pointcloud_msg = points;

    // fill in the vehicle to sensor transform and frame names
    query_data->robot_frame.emplace(robot_frame);
    query_data->lidar_frame.emplace(lidar_frame);
    query_data->T_s_r.emplace(T_lidar_robot);

    // execute the pipeline
    tactic->input(query_data);

    ++frame;
    if ((frame % 100) == 0) {
      ++terrain_type;
      terrain_type %= 5;
    };

    CLOG(WARNING, "test") << "Point cloud frame " << frame << " with timestamp "
                          << (unsigned long)(points->header.stamp.sec * 1e9 +
                                             points->header.stamp.nanosec)
                          << " took " << timer;
    timer.reset();
  }

  planner->setRunning(false);
  planner.reset();

  tactic.reset();

  callback.reset();

  pipeline.reset();
  pipeline_factory.reset();

  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";

  rclcpp::shutdown();
}