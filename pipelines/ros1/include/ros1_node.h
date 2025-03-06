#ifndef ROS1_NODE_H
#define ROS1_NODE_H

#include "planner/trg_planner.h"
#include "ros1_utils.hpp"
#include "utils/common.h"

class ROS1Node : public TRGPlanner {
 public:
  explicit ROS1Node(const ros::NodeHandle &nh);
  virtual ~ROS1Node();

  void getParams(const ros::NodeHandle &nh);
  void cbPose(const ROS1Types::Pose::ConstPtr &msg);
  void cbOdom(const ROS1Types::Odom::ConstPtr &msg);
  void cbCloud(const ROS1Types::PointCloud::ConstPtr &msg);
  void cbGrid(const ROS1Types::GridMap::ConstPtr &msg);
  void cbGoal(const ROS1Types::Pose::ConstPtr &msg);

  void publishTimer();
  void debugTimer();

 protected:
  //// ROS1
  ros::NodeHandle nh_;
  //// ROS1 Subscribers
  struct Subscribers {
    ros::Subscriber ego_pose_;
    ros::Subscriber ego_odom_;
    ros::Subscriber obs_cloud_;
    ros::Subscriber obs_grid_;
    ros::Subscriber goal_;
  } sub;
  //// ROS1 Publishers
  struct Publishers {
    ros::Publisher pre_map_;
    ros::Publisher goal_;
    ros::Publisher path_;
  } pub;
  struct Debug {
    ros::Publisher global_trg_;
    ros::Publisher local_trg_;
    ros::Publisher obs_map_;
    ros::Publisher path_info_;
  } debug;

  //// Parameters
  bool                                         isDebug;
  std::string                                  frame_id;
  std::unordered_map<std::string, std::string> topics_;

  //// TF
  struct TFCache {
    tf::StampedTransform  tf;
    tf::TransformListener listener;
    bool                  isTFCached = false;
  } tf_cache;

  void vizGraph(std::string type, ros::Publisher &pub);
};

#endif