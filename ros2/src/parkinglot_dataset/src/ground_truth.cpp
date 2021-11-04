#include <filesystem>

#include "proj.h"

#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.h"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"

#include "steam.hpp"

#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"

#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// clang-format off
namespace fs = std::filesystem;

using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;


class VelocityConstraintEval : public steam::ErrorEvaluator<6, 6>::type {
public:
  using Ptr = std::shared_ptr<VelocityConstraintEval>;
  using ConstPtr = std::shared_ptr<const VelocityConstraintEval>;

  VelocityConstraintEval(const steam::VectorSpaceStateVar::ConstPtr &stateVec,
                         const Eigen::Array<double, 6, 1> &coeff)
      : stateVec_(stateVec), coeff_(coeff) {
    if (stateVec_->getPerturbDim() != 6) {
      throw std::invalid_argument("Dimension was improper size");
    }
  }

  virtual bool isActive() const { return !stateVec_->isLocked(); }

  virtual Eigen::Matrix<double, 6, 1> evaluate() const {

    // Get value of state variable
    auto x = stateVec_->getValue().array();

    auto err = x * x * coeff_;

    return err.matrix();
  }

  virtual Eigen::Matrix<double, 6, 1>
  evaluate(const Eigen::Matrix<double, 6, 6> &lhs,
           std::vector<steam::Jacobian<6, 6>> *jacs) const {

    // Get value of state variable
    auto x = stateVec_->getValue().array();

    // Check for null ptr and clear jacobians
    if (jacs == NULL) {
      throw std::invalid_argument(
          "Null pointer provided to return-input 'jacs' in evaluate");
    }
    jacs->clear();

    // If state not locked, add Jacobian
    if (!stateVec_->isLocked()) {

      // Create Jacobian object
      jacs->push_back(steam::Jacobian<6, 6>());
      steam::Jacobian<6, 6> &jacref = jacs->back();
      jacref.key = stateVec_->getKey();

      auto diag = 2 * x * coeff_;

      // Fill out matrix
      Eigen::Matrix<double, 6, 6> jacobian =
          Eigen::Matrix<double, 6, 6>::Zero();
      jacobian.diagonal() = diag;
      jacref.jac = lhs * jacobian;
    }

    // Construct error and return
    auto err = x * x * coeff_;
    return err.matrix();
  }

private:
  steam::VectorSpaceStateVar::ConstPtr stateVec_;

  Eigen::Array<double, 6, 1> coeff_;
};

struct GPSPose {
  int64_t timestamp;
  double latitude;
  double longitude;
  double altitude;
  std::array<double, 9> cov;
  double x;
  double y;
  double x_interp;
  double y_interp;
};
using GPSPoses = std::vector<GPSPose>;

struct TrajStateVar {
  steam::Time time;
  steam::se3::TransformStateVar::Ptr pose;
  steam::VectorSpaceStateVar::Ptr velocity;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("parkinglot");

  configureLogging();

  // dataset directory
  const auto dataset_str = node->declare_parameter<std::string>("dataset", "");
  fs::path dataset_dir{utils::expand_user(utils::expand_env(dataset_str))};

  // Load dataset (gps)
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = dataset_dir.string();
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0; // default
  storage_options.max_cache_size = 0;   // default

  // load all gps messages
  std::vector<std::pair<int64_t, std::shared_ptr<sensor_msgs::msg::NavSatFix>>>
      gps_meas_vec;
  {
    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back("/fix");

    rosbag2_cpp::Reader reader;
    reader.open(storage_options, converter_options);
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;

    while (reader.has_next()) {
      auto bagmsg = reader.read_next();
      auto timestamp = bagmsg->time_stamp;
      rclcpp::SerializedMessage msg(*bagmsg->serialized_data);
      auto gps_meas = std::make_shared<sensor_msgs::msg::NavSatFix>();
      serializer.deserialize_message(&msg, gps_meas.get());

      gps_meas_vec.emplace_back(std::make_pair(timestamp, gps_meas));

      // LOG(INFO) << "Loaded GPS message index with timestamp: " << timestamp;
    }
  }

  // load all lidar time stamps
  std::vector<int64_t> timestamp_vec;
  {
    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back("/points");

    rosbag2_cpp::Reader reader;
    reader.open(storage_options, converter_options);
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

    while (reader.has_next()) {
      auto bagmsg = reader.read_next();
      rclcpp::SerializedMessage msg(*bagmsg->serialized_data);
      auto lidar_meas = std::make_shared<sensor_msgs::msg::PointCloud2>();
      serializer.deserialize_message(&msg, lidar_meas.get());
      auto timestamp = (int64_t)(lidar_meas->header.stamp.sec * 1e9) +
                       lidar_meas->header.stamp.nanosec;
      timestamp_vec.emplace_back(timestamp);
      // LOG(INFO) << "Loaded lidar timestamp: " << timestamp;
    }
  }

  // build a projection
  std::string pjstr{"+proj=etmerc +ellps=WGS84 +lat_0=43.78210381666667 "
                    "+lon_0=-79.46711405 +x_0=0 +y_0=0 +z_0=149.765 +k_0=1"};
  PJ *pj_utm = proj_create(PJ_DEFAULT_CTX, pjstr.c_str());
  if (pj_utm == nullptr) {
    LOG(ERROR) << "Failed to create projection";
    throw std::runtime_error{"Failed to create projection"};
  }

  std::array<double, 3> start_gps_coord{43.78210381666667, -79.46711405,
                                        149.765};
  std::array<double, 2> start_xy_coord{0, 0};

  // collect segments along a path that have good gps measurements
  GPSPoses gps_poses;
  int num_large_covariance = 0;
  for (const auto &gps_meas : gps_meas_vec) {
    const auto &meas = gps_meas.second;
    // check if covariance is good
    bool covariance_is_large = meas->position_covariance[0] > 0.1;
    if (covariance_is_large) {
      ++num_large_covariance;
      continue;
    }

    // convert to x y
    PJ_COORD src, res;
    src.uv.u = proj_torad(meas->longitude);
    src.uv.v = proj_torad(meas->latitude);
    res = proj_trans(pj_utm, PJ_FWD, src);

    GPSPose gps_pose;
    gps_pose.timestamp = gps_meas.first;
    gps_pose.latitude = meas->latitude - start_gps_coord[0];
    gps_pose.longitude = meas->longitude - start_gps_coord[1];
    gps_pose.altitude = meas->altitude - start_gps_coord[2];
    gps_pose.cov = meas->position_covariance;
    gps_pose.x = res.uv.u - start_xy_coord[0];
    gps_pose.y = res.uv.v - start_xy_coord[1];

    gps_poses.push_back(gps_pose);

    // LOG(INFO) << "GPS meas: " << gps_poses.timestamp.back() << " "
    //           << gps_poses.longitude.back() << " " <<
    //           gps_poses.latitude.back()
    //           << " " << gps_poses.altitude.back() << " " <<
    //           gps_poses.x.back()
    //           << " " << gps_poses.y.back();
    // LOG(INFO) << "GPS meas: " << gps_poses.timestamp.back() << " "
    //           << gps_poses.x.back() << " " << gps_poses.y.back();
  }

  LOG(INFO) << "Large cov " << num_large_covariance << ", total " << gps_meas_vec.size();

  // remove the first and last gps measurements
  GPSPoses gps_poses_temp(gps_poses.begin(), gps_poses.end());
  gps_poses = gps_poses_temp;

  size_t num_states = gps_poses.size();

  // create a steam trajectory that use gps measurements
  Eigen::MatrixXd gps_meas_mat = Eigen::MatrixXd::Zero(3, num_states);
  for (size_t i = 0; i < num_states; ++i) {
    gps_meas_mat(0, i) = gps_poses[i].x;
    gps_meas_mat(1, i) = gps_poses[i].y;
    gps_meas_mat(2, i) = 0.0;
  }

  std::vector<TrajStateVar> states;
  for (size_t i = 0; i < num_states; ++i) {
    TrajStateVar temp;
    temp.time = gps_poses[i].timestamp;

    // pose
    Eigen::Matrix3d C_0k;
    if (i < num_states - 1) {
      Eigen::Vector3d C_0k_xaxis = gps_meas_mat.col(i + 1) - gps_meas_mat.col(i);
      double theta_z = atan2(C_0k_xaxis(1), C_0k_xaxis(0));
      C_0k << cos(theta_z), -sin(theta_z), 0,
              sin(theta_z), cos(theta_z),  0,
              0, 0, 1;
    } else {
      C_0k << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    }

    lgmath::se3::Transformation T_k0(C_0k.transpose(), gps_meas_mat.block<3, 1>(0, i));
    temp.pose = std::make_shared<steam::se3::TransformStateVar>(T_k0);

    // velocity
    Eigen::Matrix<double,6,1> w_0k_ink; w_0k_ink.setZero();
    temp.velocity = std::make_shared<steam::VectorSpaceStateVar>(w_0k_ink);

    states.emplace_back(temp);
  }



  // add trajectory cost terms first
  auto cost_terms = std::make_shared<steam::ParallelizedCostTermCollection>();

  // Smoothing factor diagonal
  Eigen::Array<double,1,6> Qc_diag; Qc_diag << 1.0, 0.001, 0.001, 0.001, 0.001, 1.0;
  Eigen::Matrix<double,6,6> Qc_inv; Qc_inv.setZero(); Qc_inv.diagonal() = 1.0 / Qc_diag;

  // create trajectory
  steam::se3::SteamTrajInterface traj(Qc_inv, true);
  for (size_t i = 0; i < states.size(); i++) {
    const auto& state = states.at(i);
    auto temp = steam::se3::TransformStateEvaluator::MakeShared(state.pose);
    traj.add(state.time, temp, state.velocity);
  }

  {
    // publish initial path
    auto init_path_pub = node->create_publisher<nav_msgs::msg::Path>("init_path", 1);
    nav_msgs::msg::Path init_path;
    init_path.header.frame_id = "world";
    // path.header.stamp = 0;
    auto &poses = init_path.poses;
    for (auto& state: states) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose = tf2::toMsg(Eigen::Affine3d(state.pose->getValue().inverse().matrix()));
      poses.push_back(pose);
    }
    init_path_pub->publish(init_path);
  }

  // hard-code gps to robot transform
  Eigen::Matrix4d T_robot_gps_mat;
  T_robot_gps_mat << 1, 0, 0, 0.6,
                     0, 1, 0, 0,
                     0, 0, 1, 0.52,
                     0, 0, 0, 1;
  lgmath::se3::Transformation T_robot_gps(T_robot_gps_mat);
  auto T_gps_robot = T_robot_gps.inverse();
  auto T_gps_robot_eval = steam::se3::FixedTransformEvaluator::MakeShared(T_gps_robot);


  traj.appendPriorCostTerms(cost_terms);

  // shared loss functions
  auto loss_func = std::make_shared<steam::L2LossFunc>();

  // add gps measurement cost terms
  for (size_t i = 0; i < states.size(); i++) {
    const auto& state = states.at(i);

    // T_rq = T_k0
    auto T_rq = steam::se3::compose(T_gps_robot_eval, steam::se3::TransformStateEvaluator::MakeShared(state.pose));

    Eigen::Matrix3d cov;
    cov << gps_poses[i].cov[0], 0,                   0,
           0,                   gps_poses[i].cov[4], 0,
           0,                   0,                   0.0001;
    auto noise_model = std::make_shared<steam::StaticNoiseModel<3>>(cov, steam::COVARIANCE);

    // Construct error function
    auto error_func = std::make_shared<steam::PointToPointErrorEval2>(T_rq, Eigen::Vector3d::Zero(), gps_meas_mat.block<3, 1>(0, i));

    // Create cost term and add to problem
    auto cost =  std::make_shared<steam::WeightedLeastSqCostTerm<3, 6>>(error_func, noise_model, loss_func);

    cost_terms->add(cost);
  }

  // add side-slip constraint
  Eigen::Matrix<double, 6, 1> coeff;
  coeff << 0, 10, 10, 10, 10, 0;
  auto vel_noise_model = std::make_shared<steam::StaticNoiseModel<6>>(Eigen::Matrix<double, 6, 6>::Identity());
  for (size_t i = 0; i < states.size(); i++) {
    const auto& state = states.at(i);
    auto & velocity = state.velocity;

    auto error_func = std::make_shared<VelocityConstraintEval>(velocity, coeff);

    auto cost = std::make_shared<steam::WeightedLeastSqCostTerm<6, 6>>(error_func, vel_noise_model, loss_func);

    cost_terms->add(cost);
  }

  steam::OptimizationProblem problem;
  for (unsigned int i = 0; i < states.size(); i++) {
    const auto& state = states.at(i);
    problem.addStateVariable(state.pose);
    problem.addStateVariable(state.velocity);
  }
  problem.addCostTerm(cost_terms);

  typedef steam::DoglegGaussNewtonSolver SolverType;
  SolverType::Params params;
  params.verbose = true;
  SolverType solver(&problem, params);
  solver.optimize();

  {
    // publish final path
    auto final_path_pub = node->create_publisher<nav_msgs::msg::Path>("final_path", 1);
    nav_msgs::msg::Path final_path;
    final_path.header.frame_id = "world";
    // path.header.stamp = 0;
    auto &poses = final_path.poses;
    for (auto& state: states) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose = tf2::toMsg(Eigen::Affine3d(state.pose->getValue().inverse().matrix()));
      poses.push_back(pose);
    }
    final_path_pub->publish(final_path);
  }

  // interpolate
  std::vector<lgmath::se3::Transformation> T_0_robot_vec;
  for (const auto& timestamp: timestamp_vec)
    T_0_robot_vec.push_back(traj.getInterpPoseEval(steam::Time(timestamp))->evaluate().inverse());

  {
    // publish interpolated path
    auto inpert_path_pub = node->create_publisher<nav_msgs::msg::Path>("interp_path", 1);
    nav_msgs::msg::Path inpert_path;
    inpert_path.header.frame_id = "world";
    // path.header.stamp = 0;
    auto &poses = inpert_path.poses;
    for (const auto& T_0_robot: T_0_robot_vec) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose = tf2::toMsg(Eigen::Affine3d(T_0_robot.matrix()));
      poses.push_back(pose);
    }
    inpert_path_pub->publish(inpert_path);
  }

  std::ofstream outstream;
  outstream.open(dataset_dir / "robot_poses.txt");  // T_0_robot
  outstream << std::setprecision(6) << std::scientific;
  for (size_t i = 0; i < timestamp_vec.size(); i++) {
    outstream << timestamp_vec[i] << " ";
    const auto& tmp = T_0_robot_vec[i].matrix();
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        outstream << tmp(i, j);
        if (i != 3 || j != 3) outstream << " ";
      }
    outstream << "\n";
  }
  outstream.close();

  // cleanup
  proj_destroy(pj_utm);
}