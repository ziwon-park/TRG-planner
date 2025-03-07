#include "ros1_node.h"

ROS1Node::ROS1Node(const ros::NodeHandle &nh) : nh_(nh) {
  getParams(nh_);

  sub.ego_pose_ = nh_.subscribe<ROS1Types::Pose>(topics_["egoPose"], 1, &ROS1Node::cbPose, this);
  sub.ego_odom_ = nh_.subscribe<ROS1Types::Odom>(topics_["egoOdom"], 1, &ROS1Node::cbOdom, this);
  sub.obs_cloud_ =
      nh_.subscribe<ROS1Types::PointCloud>(topics_["obsCloud"], 1, &ROS1Node::cbCloud, this);
  sub.obs_grid_ = nh_.subscribe<ROS1Types::GridMap>(topics_["obsGrid"], 1, &ROS1Node::cbGrid, this);
  sub.goal_     = nh_.subscribe<ROS1Types::Pose>(topics_["goal"], 1, &ROS1Node::cbGoal, this);

  pub.pre_map_ = nh_.advertise<ROS1Types::PointCloud>(topics_["preMap"], 1);
  pub.goal_    = nh_.advertise<ROS1Types::PointCloud>(topics_["outGoal"], 1);
  pub.path_    = nh_.advertise<ROS1Types::Path>(topics_["path"], 1);

  debug.global_trg_ = nh_.advertise<ROS1Types::MarkerArray>(topics_["globalTRG"], 1);
  debug.local_trg_  = nh_.advertise<ROS1Types::MarkerArray>(topics_["localTRG"], 1);
  debug.obs_map_    = nh_.advertise<ROS1Types::PointCloud>(topics_["obsMap"], 1);
  debug.path_info_  = nh_.advertise<ROS1Types::FloatArray>(topics_["pathInfo"], 1);

  TRGPlanner::init();
  print("TRG Planner ROS1 initialized", param_.isVerbose);

  //// Threads
  thd.publish = std::thread(&ROS1Node::publishTimer, this);
  if (isDebug) {
    thd.debug = std::thread(&ROS1Node::debugTimer, this);
  }
}

ROS1Node::~ROS1Node() { thd.publish.join(); }

void ROS1Node::getParams(const ros::NodeHandle &nh) {
  nh.param("/ros1/isDebug", isDebug, true);
  nh.param("/ros1/frameId", frame_id, std::string("map"));

  nh.param("/ros1/topic/input/egoPose",
           topics_["egoPose"],
           std::string("/trg_ros1_node/input/default_ego_pose"));
  nh.param("/ros1/topic/input/egoOdom",
           topics_["egoOdom"],
           std::string("/trg_ros1_node/input/default_ego_odom"));
  nh.param("/ros1/topic/input/obsCloud",
           topics_["obsCloud"],
           std::string("/trg_ros1_node/input/default_obs_cloud"));
  nh.param("/ros1/topic/input/obsGrid",
           topics_["obsGrid"],
           std::string("/trg_ros1_node/input/default_obs_grid"));
  nh.param(
      "/ros1/topic/input/goal", topics_["goal"], std::string("/trg_ros1_node/input/default_goal"));
  nh.param("/ros1/topic/output/preMap",
           topics_["preMap"],
           std::string("/trg_ros1_node/debug/default_preMap"));
  nh.param("/ros1/topic/output/goal",
           topics_["outGoal"],
           std::string("/trg_ros1_node/output/default_goal"));
  nh.param("/ros1/topic/output/path",
           topics_["path"],
           std::string("/trg_ros1_node/output/default_path"));
  nh.param("/ros1/topic/debug/globalTRG",
           topics_["globalTRG"],
           std::string("/trg_ros1_node/debug/globalTRG"));
  nh.param("/ros1/topic/debug/localTRG",
           topics_["localTRG"],
           std::string("/trg_ros1_node/debug/localTRG"));
  nh.param("/ros1/topic/debug/obsMap",
           topics_["obsMap"],
           std::string("/trg_ros1_node/debug/default_obsMap"));
  nh.param("/ros1/topic/debug/pathInfo",
           topics_["pathInfo"],
           std::string("/trg_ros1_node/debug/default_pathInfo"));

  std::string map_config_name;
  nh.param("mapConfig", map_config_name, std::string("default"));
  std::string map_config_path =
      std::string(TRG_ROS_DIR) + "/../config/" + map_config_name + ".yaml";
  if (!std::filesystem::exists(map_config_path)) {
    print_error("Map config file does not exist: " + map_config_path);
    exit(1);
  }
  TRGPlanner::setParams(map_config_path);
}

void ROS1Node::cbPose(const ROS1Types::Pose::ConstPtr &msg) {
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

void ROS1Node::cbOdom(const ROS1Types::Odom::ConstPtr &msg) {
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

void ROS1Node::cbCloud(const ROS1Types::PointCloud::ConstPtr &msg) {
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
        tf_cache.listener.waitForTransform(
            state_.frame_id, msg->header.frame_id, ros::Time(0), ros::Duration(0.01));
        tf_cache.listener.lookupTransform(
            state_.frame_id, msg->header.frame_id, ros::Time(0), tf_cache.tf);
        Eigen::Matrix4f T_S2M =
            Eigen::Matrix4f::Identity();  // transform from sensor frame to map frame
        T_S2M.block<3, 3>(0, 0) = Eigen::Quaternionf(tf_cache.tf.getRotation().w(),
                                                     tf_cache.tf.getRotation().x(),
                                                     tf_cache.tf.getRotation().y(),
                                                     tf_cache.tf.getRotation().z())
                                      .toRotationMatrix();
        T_S2M.block<3, 1>(0, 3) = Eigen::Vector3f(
            tf_cache.tf.getOrigin().x(), tf_cache.tf.getOrigin().y(), tf_cache.tf.getOrigin().z());
        pcl::transformPointCloud(*cloud_in, *cs_.obsPtr, T_S2M);
      } catch (tf::TransformException &ex) {
        pcl::transformPointCloud(*cloud_in, *cs_.obsPtr, state_.T_B2M);  // transform to map frame
      }
    }
    flag_.obsIn = true;
  }
}

void ROS1Node::cbGrid(const ROS1Types::GridMap::ConstPtr &msg) {
  if (!flag_.poseIn) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mtx.obs);
    grid_map::GridMap           gridMap_in;
    grid_map::GridMapRosConverter::fromMessage(*msg, gridMap_in);

    pcl::PointCloud<PtsDefault>::Ptr cloud_in(new pcl::PointCloud<PtsDefault>());
    for (grid_map::GridMapIterator it(gridMap_in); !it.isPastEnd(); ++it) {
      grid_map::Position pos;
      gridMap_in.getPosition(*it, pos);
      PtsDefault pt;
      pt.x = pos.x();
      pt.y = pos.y();
      pt.z = gridMap_in.at("elevation", *it);
      if (std::isnan(pt.z)) {
        continue;
      }
      cloud_in->push_back(pt);
    }
    if (msg->info.header.frame_id == state_.frame_id) {
      pcl::transformPointCloud(*cloud_in, *cs_.obsPtr, Eigen::Matrix4f::Identity());
    } else {
      try {
        tf_cache.listener.waitForTransform(
            state_.frame_id, msg->info.header.frame_id, ros::Time(0), ros::Duration(0.01));
        tf_cache.listener.lookupTransform(
            state_.frame_id, msg->info.header.frame_id, ros::Time(0), tf_cache.tf);
        Eigen::Matrix4f T_S2M =
            Eigen::Matrix4f::Identity();  // transform from sensor frame to map frame
        T_S2M.block<3, 3>(0, 0) = Eigen::Quaternionf(tf_cache.tf.getRotation().w(),
                                                     tf_cache.tf.getRotation().x(),
                                                     tf_cache.tf.getRotation().y(),
                                                     tf_cache.tf.getRotation().z())
                                      .toRotationMatrix();
        T_S2M.block<3, 1>(0, 3) = Eigen::Vector3f(
            tf_cache.tf.getOrigin().x(), tf_cache.tf.getOrigin().y(), tf_cache.tf.getOrigin().z());
        pcl::transformPointCloud(*cloud_in, *cs_.obsPtr, T_S2M);
      } catch (tf::TransformException &ex) {
        pcl::transformPointCloud(*cloud_in, *cs_.obsPtr, state_.T_B2M);  // transform to map frame
      }
    }
    flag_.obsIn = true;
  }
}

void ROS1Node::cbGoal(const ROS1Types::Pose::ConstPtr &msg) {
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

void ROS1Node::publishTimer() {
  while (is_running.load()) {
    auto start_loop = tic();
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));

    if (param_.isPreMap) {
      publishCloud(frame_id, cs_.preMapPtr, pub.pre_map_);
    }
    if (flag_.pathFound) {
      flag_.pathFound = false;
      publishPath(frame_id, path_.smooth, pub.path_);
      ROS1Types::FloatArray path_info_msg;
      path_info_msg.data.push_back(path_.direct_dist);
      path_info_msg.data.push_back(path_.raw_path_length);
      path_info_msg.data.push_back(path_.smooth_path_length);
      path_info_msg.data.push_back(path_.planning_time);
      path_info_msg.data.push_back(path_.avg_risk);
      debug.path_info_.publish(path_info_msg);
    }
    if (goal_state_.init) {
      PointCloudPtr goal_cloud(new pcl::PointCloud<PtsDefault>());
      PtsDefault    pt;
      pt.x = goal_state_.pose.x();
      pt.y = goal_state_.pose.y();
      pt.z = goal_state_.pose.z();
      goal_cloud->push_back(pt);
      publishCloud(frame_id, goal_cloud, pub.goal_);
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

void ROS1Node::debugTimer() {
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
      publishCloud(frame_id, cs_.obsPtr, debug.obs_map_);
    }
    float loop_time   = toc(start_loop, "ms");
    int   remain_time = 1000 / param_.debug_rate - loop_time;
    if (remain_time > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(remain_time));
    }
    thd.hz["debug"] = std::round(1000 / toc(start_loop, "ms") * 100) / 100;
  }
}

void ROS1Node::vizGraph(std::string type, ros::Publisher &pub) {
  ROS1Types::MarkerArray graph_marker;
  ROS1Types::Marker      delete_marker;
  delete_marker.id     = -1;
  delete_marker.action = ROS1Types::Marker::DELETEALL;
  graph_marker.markers.push_back(delete_marker);

  std::unordered_map<int, TRG::Node *> nodes;
  trg_->lockGraph();
  trg_->getGraph(nodes, type);

  if (!nodes.empty()) {
    ROS1Types::Marker node_marker;
    node_marker.header.frame_id = frame_id;
    node_marker.header.stamp    = ros::Time::now();
    node_marker.type            = ROS1Types::Marker::SPHERE_LIST;
    node_marker.id              = 0;
    node_marker.action          = ROS1Types::Marker::ADD;
    node_marker.scale.x = node_marker.scale.y = node_marker.scale.z = 0.1;
    node_marker.pose.orientation.w                                  = 1.0;

    ROS1Types::Marker edge_marker;
    edge_marker.header.frame_id = frame_id;
    edge_marker.header.stamp    = ros::Time::now();
    edge_marker.type            = ROS1Types::Marker::LINE_LIST;
    edge_marker.id              = 1;
    edge_marker.action          = ROS1Types::Marker::ADD;
    edge_marker.scale.x = edge_marker.scale.y = edge_marker.scale.z = 0.02;
    edge_marker.pose.orientation.w                                  = 1.0;

    float z_offset = 0.15;

    for (const auto &node : nodes) {
      if (node.second->state_ == TRG::NodeState::Invalid) {
        continue;
      }
      geometry_msgs::Point p;
      p.x = node.second->pos_.x();
      p.y = node.second->pos_.y();
      p.z = node.second->pos_.z() + z_offset;
      node_marker.points.push_back(p);
      std_msgs::ColorRGBA color;
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
      node_marker.colors.push_back(color);

      for (const auto &edge : node.second->edges_) {
        if (nodes.find(edge->dst_id_) == nodes.end()) {
          continue;
        }
        if (nodes.at(edge->dst_id_)->state_ == TRG::NodeState::Invalid) {
          continue;
        }
        geometry_msgs::Point p1, p2;
        p1.x = node.second->pos_.x();
        p1.y = node.second->pos_.y();
        p1.z = node.second->pos_.z() + z_offset;
        p2.x = nodes.at(edge->dst_id_)->pos_.x();
        p2.y = nodes.at(edge->dst_id_)->pos_.y();
        p2.z = nodes.at(edge->dst_id_)->pos_.z() + z_offset;
        edge_marker.points.push_back(p1);
        edge_marker.points.push_back(p2);
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0 * (1 - edge->weight_);
        color.b = 1.0 * (1 - edge->weight_);
        color.a = 1.0;
        edge_marker.colors.push_back(color);
        edge_marker.colors.push_back(color);
      }
    }
    graph_marker.markers.push_back(node_marker);
    graph_marker.markers.push_back(edge_marker);
  }
  pub.publish(graph_marker);
  trg_->unlockGraph();
}
