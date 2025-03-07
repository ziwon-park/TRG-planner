#include "ros2_node.h"

#include <yaml-cpp/yaml.h>

ROS2Node::ROS2Node(const rclcpp::Node::SharedPtr &node) : n_(node) {
  getParams(n_);

  tf_cache.tfBuffer   = std::make_shared<tf2_ros::Buffer>(n_->get_clock());
  tf_cache.tfListener = std::make_shared<tf2_ros::TransformListener>(*tf_cache.tfBuffer);

  sub.ego_pose_ = n_->create_subscription<ROS2Types::Pose>(
      topics_["egoPose"], qos.for_reli, std::bind(&ROS2Node::cbPose, this, std::placeholders::_1));
  sub.ego_odom_ = n_->create_subscription<ROS2Types::Odom>(
      topics_["egoOdom"], qos.for_reli, std::bind(&ROS2Node::cbOdom, this, std::placeholders::_1));
  sub.obs_cloud_ = n_->create_subscription<ROS2Types::PointCloud>(
      topics_["obsCloud"],
      qos.for_reli,
      std::bind(&ROS2Node::cbCloud, this, std::placeholders::_1));
  sub.goal_ = n_->create_subscription<ROS2Types::Pose>(
      topics_["goal"], qos.for_reli, std::bind(&ROS2Node::cbGoal, this, std::placeholders::_1));

  pub.pre_map_ = n_->create_publisher<ROS2Types::PointCloud>(topics_["preMap"], 1);
  pub.goal_    = n_->create_publisher<ROS2Types::PointCloud>(topics_["outGoal"], 1);
  pub.path_    = n_->create_publisher<ROS2Types::Path>(topics_["path"], 1);

  debug.global_trg_ = n_->create_publisher<ROS2Types::MarkerArray>(topics_["globalTRG"], 1);
  debug.local_trg_  = n_->create_publisher<ROS2Types::MarkerArray>(topics_["localTRG"], 1);
  debug.obs_map_    = n_->create_publisher<ROS2Types::PointCloud>(topics_["obsMap"], 1);
  debug.path_info_  = n_->create_publisher<ROS2Types::FloatArray>(topics_["pathInfo"], 1);

  TRGPlanner::init();
  print("TRG Planner ROS2 initialized", param_.isVerbose);

  //// Threads
  thd.publish = std::thread(&ROS2Node::publishTimer, this);
  thd.debug   = std::thread(&ROS2Node::debugTimer, this);
}

ROS2Node::~ROS2Node() { thd.publish.join(); }

void ROS2Node::getParams(const rclcpp::Node::SharedPtr &n_) {
  n_->declare_parameter<bool>("ros2.isDebug", true);
  n_->declare_parameter<std::string>("ros2.frameId", "map");

  n_->declare_parameter<std::string>("ros2.topic.input.egoPose",
                                     "/trg_ros2_node/input/default_ego_pose");
  n_->declare_parameter<std::string>("ros2.topic.input.egoOdom",
                                     "/trg_ros2_node/input/default_ego_odom");
  n_->declare_parameter<std::string>("ros2.topic.input.obsCloud",
                                     "/trg_ros2_node/input/default_obs_cloud");
  n_->declare_parameter<std::string>("ros2.topic.input.goal", "/trg_ros2_node/input/default_goal");
  n_->declare_parameter<std::string>("ros2.topic.output.preMap",
                                     "/trg_ros2_node/output/default_preMap");
  n_->declare_parameter<std::string>("ros2.topic.output.goal",
                                     "/trg_ros2_node/output/default_goal");
  n_->declare_parameter<std::string>("ros2.topic.output.path",
                                     "/trg_ros2_node/output/default_path");
  n_->declare_parameter<std::string>("ros2.topic.debug.globalTRG",
                                     "/trg_ros2_node/debug/globalTRG");
  n_->declare_parameter<std::string>("ros2.topic.debug.localTRG", "/trg_ros2_node/debug/localTRG");
  n_->declare_parameter<std::string>("ros2.topic.debug.obsMap",
                                     "/trg_ros2_node/debug/default_obsMap");
  n_->declare_parameter<std::string>("ros2.topic.debug.pathInfo",
                                     "/trg_ros2_node/debug/default_pathInfo");

  n_->declare_parameter<std::string>("mapConfig", "default");

  //// Get parameters
  n_->get_parameter("ros2.isDebug", isDebug);
  n_->get_parameter("ros2.frameId", frame_id);

  n_->get_parameter("ros2.topic.input.egoPose", topics_["egoPose"]);
  n_->get_parameter("ros2.topic.input.egoOdom", topics_["egoOdom"]);
  n_->get_parameter("ros2.topic.input.obsCloud", topics_["obsCloud"]);
  n_->get_parameter("ros2.topic.input.goal", topics_["goal"]);
  n_->get_parameter("ros2.topic.output.preMap", topics_["preMap"]);
  n_->get_parameter("ros2.topic.output.goal", topics_["outGoal"]);
  n_->get_parameter("ros2.topic.output.path", topics_["path"]);
  n_->get_parameter("ros2.topic.debug.globalTRG", topics_["globalTRG"]);
  n_->get_parameter("ros2.topic.debug.localTRG", topics_["localTRG"]);
  n_->get_parameter("ros2.topic.debug.obsMap", topics_["obsMap"]);
  n_->get_parameter("ros2.topic.debug.pathInfo", topics_["pathInfo"]);

  std::string map_config_name;
  n_->get_parameter("mapConfig", map_config_name);
  std::string map_config_path = std::string(TRG_DIR) + "/configs/map/" + map_config_name + ".yaml";
  YAML::Node  map_config_yaml = YAML::LoadFile(map_config_path);

  param_.isVerbose = map_config_yaml["isVerbose"].as<bool>(true);

  param_.graph_rate    = map_config_yaml["timer"]["graphRate"].as<float>(1.0f);
  param_.planning_rate = map_config_yaml["timer"]["planningRate"].as<float>(1.0f);
  param_.publish_rate  = map_config_yaml["timer"]["publishRate"].as<float>(1.0f);
  param_.debug_rate    = map_config_yaml["timer"]["debugRate"].as<float>(1.0f);

  param_.isPreMap   = map_config_yaml["map"]["isPrebuiltMap"].as<bool>(false);
  param_.preMapPath = map_config_yaml["map"]["prebuiltMapPath"].as<std::string>("");
  param_.isVoxelize = map_config_yaml["map"]["isVoxelize"].as<bool>(false);
  param_.VoxelSize  = map_config_yaml["map"]["voxelSize"].as<float>(0.1f);

  param_.isPreGraph         = map_config_yaml["trg"]["isPrebuiltTRG"].as<bool>(false);
  param_.preGraphPath       = map_config_yaml["trg"]["prebuiltTRGPath"].as<std::string>("");
  param_.isUpdate           = map_config_yaml["trg"]["isUpdate"].as<bool>(false);
  param_.expandDist         = map_config_yaml["trg"]["expandDist"].as<float>(0.6f);
  param_.robotSize          = map_config_yaml["trg"]["robotSize"].as<float>(0.3f);
  param_.sampleNum          = map_config_yaml["trg"]["sampleNum"].as<int>(20);
  param_.heightThreshold    = map_config_yaml["trg"]["heightThreshold"].as<float>(0.15f);
  param_.collisionThreshold = map_config_yaml["trg"]["collisionThreshold"].as<float>(0.2f);
  param_.updateCollisionThreshold =
      map_config_yaml["trg"]["updateCollisionThreshold"].as<float>(0.2f);
  param_.safetyFactor   = map_config_yaml["trg"]["safetyFactor"].as<float>(1.0f);
  param_.goal_tolerance = map_config_yaml["trg"]["goalTolerance"].as<float>(0.8f);
}

void ROS2Node::cbPose(const std::shared_ptr<const ROS2Types::Pose> &msg) {
  {
    std::lock_guard<std::mutex> lock(mtx.odom);
    state_.frame_id = msg->header.frame_id;
    state_.pose3d =
        Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    state_.pose2d                  = Eigen::Vector2f(msg->pose.position.x, msg->pose.position.y);
    state_.quat                    = Eigen::Quaternionf(msg->pose.orientation.w,
                                     msg->pose.orientation.x,
                                     msg->pose.orientation.y,
                                     msg->pose.orientation.z);
    state_.T_B2M.block<3, 3>(0, 0) = state_.quat.toRotationMatrix();
    state_.T_B2M.block<3, 1>(0, 3) = state_.pose3d;
    flag_.poseIn                   = true;
  }
}

void ROS2Node::cbOdom(const std::shared_ptr<const ROS2Types::Odom> &msg) {
  {
    std::lock_guard<std::mutex> lock(mtx.odom);
    state_.frame_id = msg->header.frame_id;
    state_.pose3d   = Eigen::Vector3f(
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    state_.pose2d = Eigen::Vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
    state_.quat   = Eigen::Quaternionf(msg->pose.pose.orientation.w,
                                     msg->pose.pose.orientation.x,
                                     msg->pose.pose.orientation.y,
                                     msg->pose.pose.orientation.z);
    state_.T_B2M.block<3, 3>(0, 0) = state_.quat.toRotationMatrix();
    state_.T_B2M.block<3, 1>(0, 3) = state_.pose3d;
    flag_.poseIn                   = true;
  }
}

void ROS2Node::cbCloud(const std::shared_ptr<const ROS2Types::PointCloud> &msg) {
  if (!flag_.poseIn) {
    return;
  }
  {
    std::lock_guard<std::mutex>      lock(mtx.obs);
    pcl::PointCloud<PtsDefault>::Ptr cloud_in(new pcl::PointCloud<PtsDefault>());
    pcl::fromROSMsg(*msg, *cloud_in);
    if (msg->header.frame_id == state_.frame_id) {
      pcl::transformPointCloud(*cloud_in, *cs_.obsPtr, Eigen::Matrix4f::Identity());
    } else {
      try {
        tf_cache.tfStamped = tf_cache.tfBuffer->lookupTransform(
            state_.frame_id, msg->header.frame_id, rclcpp::Time(0));
        tf_cache.isTFCached = true;
      } catch (tf2::TransformException &ex) {
        print_error(ex.what());
        return;
      }

      // transform from sensor frame to map frame
      Eigen::Matrix4f    T_S2M = Eigen::Matrix4f::Identity();
      Eigen::Quaternionf q(tf_cache.tfStamped.transform.rotation.w,
                           tf_cache.tfStamped.transform.rotation.x,
                           tf_cache.tfStamped.transform.rotation.y,
                           tf_cache.tfStamped.transform.rotation.z);
      T_S2M.block<3, 3>(0, 0) = q.toRotationMatrix();
      T_S2M.block<3, 1>(0, 3) = Eigen::Vector3f(tf_cache.tfStamped.transform.translation.x,
                                                tf_cache.tfStamped.transform.translation.y,
                                                tf_cache.tfStamped.transform.translation.z);
      pcl::transformPointCloud(*cloud_in, *cs_.obsPtr, T_S2M);
    }
    flag_.obsIn = true;
  }
}

void ROS2Node::cbGoal(const std::shared_ptr<const ROS2Types::Pose> &msg) {
  if (!flag_.graphInit) {
    print_error("Graph is not initialized");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mtx.goal);
    goal_state_.pose =
        Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    goal_state_.quat = Eigen::Quaternionf(msg->pose.orientation.w,
                                          msg->pose.orientation.x,
                                          msg->pose.orientation.y,
                                          msg->pose.orientation.z);
    goal_state_.init = true;
    flag_.goalIn     = true;
    print("Goal received", param_.isVerbose);
  }
}

void ROS2Node::publishTimer() {
  while (is_running.load()) {
    auto start_loop = tic();
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));

    if (param_.isPreMap) {
      publishCloud(n_, frame_id, cs_.preMapPtr, pub.pre_map_);
    }
    if (flag_.pathFound) {
      flag_.pathFound = false;
      publishPath(n_, frame_id, path_.smooth, pub.path_);
      ROS2Types::FloatArray path_info_msg;
      path_info_msg.data.push_back(path_.direct_dist);
      path_info_msg.data.push_back(path_.raw_path_length);
      path_info_msg.data.push_back(path_.smooth_path_length);
      path_info_msg.data.push_back(path_.planning_time);
      path_info_msg.data.push_back(path_.avg_risk);
      debug.path_info_->publish(path_info_msg);
    }
    if (goal_state_.init) {
      PointCloudPtr goal_cloud(new pcl::PointCloud<PtsDefault>());
      PtsDefault    pt;
      pt.x = goal_state_.pose.x();
      pt.y = goal_state_.pose.y();
      pt.z = goal_state_.pose.z();
      goal_cloud->push_back(pt);
      publishCloud(n_, frame_id, goal_cloud, pub.goal_);
      goal_state_.init = false;
    }
    float loop_time   = toc(start_loop, "ms");
    int   remain_time = 1000 / param_.publish_rate - loop_time;
    if (remain_time > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(remain_time));
    }
    thd.hz["publish"] = std::round(1000 / toc(start_loop, "ms") * 100) / 100;
  }
}

void ROS2Node::debugTimer() {
  while (is_running.load()) {
    auto start_loop = tic();
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));
    if (flag_.graphInit) {
      vizGraph("global", debug.global_trg_);
    }
    if (flag_.graphInit && param_.isUpdate) {
      vizGraph("local", debug.local_trg_);
    }
    if (flag_.obsIn) {
      publishCloud(n_, frame_id, cs_.obsPtr, debug.obs_map_);
    }
    float loop_time   = toc(start_loop, "ms");
    int   remain_time = 1000 / param_.debug_rate - loop_time;
    if (remain_time > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(remain_time));
    }
    thd.hz["debug"] = std::round(1000 / toc(start_loop, "ms") * 100) / 100;
  }
}

void ROS2Node::vizGraph(std::string                                          type,
                        rclcpp::Publisher<ROS2Types::MarkerArray>::SharedPtr pub) {
  ROS2Types::MarkerArray graph_marker;
  ROS2Types::Marker      delete_marker;
  delete_marker.id     = -1;
  delete_marker.action = ROS2Types::Marker::DELETEALL;
  graph_marker.markers.push_back(delete_marker);

  std::unordered_map<int, TRG::Node *> nodes;
  trg_->lockGraph();
  trg_->getGraph(nodes, type);

  if (!nodes.empty()) {
    ROS2Types::Marker n_marker;
    n_marker.header.frame_id = frame_id;
    n_marker.header.stamp    = n_->now();
    n_marker.type            = ROS2Types::Marker::SPHERE_LIST;
    n_marker.id              = 0;
    n_marker.action          = ROS2Types::Marker::ADD;
    n_marker.scale.x = n_marker.scale.y = n_marker.scale.z = 0.1;
    n_marker.pose.orientation.w                            = 1.0;

    ROS2Types::Marker edge_marker;
    edge_marker.header.frame_id = frame_id;
    edge_marker.header.stamp    = n_->now();
    edge_marker.type            = ROS2Types::Marker::LINE_LIST;
    edge_marker.id              = 1;
    edge_marker.action          = ROS2Types::Marker::ADD;
    edge_marker.scale.x = edge_marker.scale.y = edge_marker.scale.z = 0.02;
    edge_marker.pose.orientation.w                                  = 1.0;

    float z_offset = 0.15;

    for (const auto &node : nodes) {
      if (node.second->state_ == TRG::NodeState::Invalid) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = node.second->pos_.x();
      p.y = node.second->pos_.y();
      p.z = node.second->pos_.z() + z_offset;
      n_marker.points.push_back(p);
      std_msgs::msg::ColorRGBA color;
      if (node.second->state_ == TRG::NodeState::Frontier) {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
      } else {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
      }
      n_marker.colors.push_back(color);

      for (const auto &edge : node.second->edges_) {
        if (nodes.find(edge->dst_id_) == nodes.end()) {
          continue;
        }
        if (nodes.at(edge->dst_id_)->state_ == TRG::NodeState::Invalid) {
          continue;
        }
        geometry_msgs::msg::Point p1, p2;
        p1.x = node.second->pos_.x();
        p1.y = node.second->pos_.y();
        p1.z = node.second->pos_.z() + z_offset;
        p2.x = nodes.at(edge->dst_id_)->pos_.x();
        p2.y = nodes.at(edge->dst_id_)->pos_.y();
        p2.z = nodes.at(edge->dst_id_)->pos_.z() + z_offset;
        edge_marker.points.push_back(p1);
        edge_marker.points.push_back(p2);
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0 * (1 - edge->weight_);
        color.b = 1.0 * (1 - edge->weight_);
        color.a = 1.0;
        edge_marker.colors.push_back(color);
        edge_marker.colors.push_back(color);
      }
    }
    graph_marker.markers.push_back(n_marker);
    graph_marker.markers.push_back(edge_marker);
  }
  pub->publish(graph_marker);
  trg_->unlockGraph();
}
