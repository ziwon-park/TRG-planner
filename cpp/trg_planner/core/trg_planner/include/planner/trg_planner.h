#ifndef CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_PLANNER_TRG_PLANNER_H_
#define CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_PLANNER_TRG_PLANNER_H_

#include "trg_planner/include/graph/trg.h"
#include "trg_planner/include/utils/common.h"

template <typename State>
class FSM {
 public:
  FSM(State init_state, const std::unordered_map<State, std::string>& state_map)
      : curr_state_(init_state), prev_state_(init_state), state_map_(state_map) {}
  virtual ~FSM()                           = default;
  virtual void transition(State new_state) = 0;
  virtual void notice()                    = 0;

  State                                  curr_state_;
  State                                  prev_state_;
  std::unordered_map<State, std::string> state_map_;
};

enum struct graphState { INIT, UPDATE, LOAD, RESET, SAVE };

class GraphFSM : public FSM<graphState> {
 public:
  GraphFSM()
      : FSM(graphState::INIT,
            {{graphState::INIT, "INIT"},
             {graphState::UPDATE, "UPDATE"},
             {graphState::LOAD, "LOAD"},
             {graphState::RESET, "RESET"},
             {graphState::SAVE, "SAVE"}}) {}

  void transition(graphState new_state) override;
  void notice() override;
};

enum struct planningState {
  RESET,
  PLANNING,
  ONGOING,
};

class PlanningFSM : public FSM<planningState> {
 public:
  PlanningFSM()
      : FSM(planningState::RESET,
            {
                {planningState::RESET, "RESET"},
                {planningState::PLANNING, "PLANNING"},
                {planningState::ONGOING, "ONGOING"},
            }) {}

  void transition(planningState new_state) override;
  void notice() override;
};

class TRGPlanner {
 public:
  TRGPlanner();
  virtual ~TRGPlanner();

  void init();
  void loadPrebuiltMap();
  void setParams(const std::string& config_path);

  void runGraphFSM();
  void runPlanningFSM();

  struct plannedPath {
    void clear() {
      raw.clear();
      smooth.clear();
      direct_dist        = 0;
      raw_path_length    = 0;
      smooth_path_length = 0;
      planning_time      = 0;
    }
    std::vector<Eigen::Vector3f> raw;
    std::vector<Eigen::Vector3f> smooth;
    float                        direct_dist;
    float                        raw_path_length;
    float                        smooth_path_length;
    float                        planning_time;
    float                        avg_risk;
  } path_;

 protected:
  struct Mutex {
    std::mutex odom;
    std::mutex obs;
    std::mutex goal;
  } mtx;

  struct Thread {
    std::thread                            graph;
    std::thread                            planning;
    std::unordered_map<std::string, float> hz;
  } thd;

  struct Parameters {
    /// debug
    bool isVerbose = true;

    /// Timer parameters
    float graph_rate;
    float planning_rate;

    /// Map parameters
    bool        isPreMap;
    std::string preMapPath;
    bool        isVoxelize;
    float       VoxelSize;

    /// TRG parameters
    bool        isPreGraph;
    std::string preGraphPath;
    bool        isUpdate;
    float       expandDist;
    float       robotSize;
    int         sampleNum;
    float       heightThreshold;
    float       collisionThreshold;
    float       updateCollisionThreshold;
    float       safetyFactor;
    float       goal_tolerance;
  } param_;

  /// TRG
  std::shared_ptr<TRG> trg_ = nullptr;

  /// FSM
  struct fsm {
    GraphFSM    graph;
    PlanningFSM planning;
  } fsm_;

  struct robotState {
    std::string     frame_id = "map";
    Eigen::Vector3f pose3d   = Eigen::Vector3f::Zero();
    Eigen::Vector2f pose2d   = Eigen::Vector2f::Zero();
    Eigen::Vector4f quat     = Eigen::Vector4f::Zero(4);
    Eigen::Matrix4f T_B2M    = Eigen::Matrix4f::Identity();  // tf from map to base
  } state_;

  struct cSpace {
    PointCloudPtr preMapPtr = nullptr;
    PointCloudPtr obsPtr    = nullptr;
  } cs_;

  struct goalState {
    Eigen::Vector3f pose;
    Eigen::Vector4f quat;
    bool            init = false;
  } goal_state_;

  struct Flag {
    bool poseIn = false;
    bool obsIn  = false;
    bool goalIn = false;

    bool graphInit     = false;
    bool pathFound     = false;
    int  planningCount = 0;
  } flag_;

 public:
  /// TRG functions
  inline std::shared_ptr<TRG> getTRG() { return trg_; }

  /// Mutex functions
  inline void lockOdom() { mtx.odom.lock(); }
  inline void unlockOdom() { mtx.odom.unlock(); }
  inline void lockObs() { mtx.obs.lock(); }
  inline void unlockObs() { mtx.obs.unlock(); }
  inline void lockGoal() { mtx.goal.lock(); }
  inline void unlockGoal() { mtx.goal.unlock(); }

  /// Setters
  inline void setStateFrameId(const std::string& frame_id) { state_.frame_id = frame_id; }
  inline void setStatePose3d(const Eigen::Vector3f& pose3d) { state_.pose3d = pose3d; }
  inline void setStatePose2d(const Eigen::Vector2f& pose2d) { state_.pose2d = pose2d; }
  inline void setStateQuat(const Eigen::Vector4f& quat) { state_.quat = quat; }
  inline void setStateT_B2M(const Eigen::Matrix4f& T_B2M) { state_.T_B2M = T_B2M; }

  inline void setPreMapEigen(const Eigen::MatrixXf& preMap) {
    EigenToPointCloud(preMap, cs_.preMapPtr);
  }
  inline void setObsEigen(const Eigen::MatrixXf& obs) { EigenToPointCloud(obs, cs_.obsPtr); }

  inline void setGoalPose(const Eigen::Vector3f& pose) { goal_state_.pose = pose; }
  inline void setGoalQuat(const Eigen::Vector4f& quat) { goal_state_.quat = quat; }
  inline void setGoalInit(bool flag) { goal_state_.init = flag; }

  inline void setFlagPoseIn(bool flag) { flag_.poseIn = flag; }
  inline void setFlagObsIn(bool flag) { flag_.obsIn = flag; }
  inline void setFlagGoalIn(bool flag) { flag_.goalIn = flag; }
  inline void setFlagGraphInit(bool flag) { flag_.graphInit = flag; }
  inline void setFlagPathFound(bool flag) { flag_.pathFound = flag; }

  /// Getters
  inline std::vector<Eigen::Vector3f> getRawPath() const { return path_.raw; }
  inline std::vector<Eigen::Vector3f> getSmoothPath() const { return path_.smooth; }
  inline float                        getDirectDist() const { return path_.direct_dist; }
  inline float                        getRawPathLength() const { return path_.raw_path_length; }
  inline float getSmoothPathLength() const { return path_.smooth_path_length; }
  inline float getPlanningTime() const { return path_.planning_time; }
  inline float getAvgRisk() const { return path_.avg_risk; }

  inline std::string     getStateFrameId() const { return state_.frame_id; }
  inline Eigen::Vector3f getStatePose3d() const { return state_.pose3d; }
  inline Eigen::Vector2f getStatePose2d() const { return state_.pose2d; }
  inline Eigen::Vector4f getStateQuat() const { return state_.quat; }
  inline Eigen::Matrix4f getStateT_B2M() const { return state_.T_B2M; }

  inline Eigen::MatrixXf getPreMapEigen() const { return PointCloudToEigen(cs_.preMapPtr); }
  inline Eigen::MatrixXf getObsEigen() const { return PointCloudToEigen(cs_.obsPtr); }

  inline Eigen::Vector3f getGoalPose() const { return goal_state_.pose; }
  inline Eigen::Vector4f getGoalQuat() const { return goal_state_.quat; }
  inline bool            getGoalInit() const { return goal_state_.init; }

  inline bool getFlagPoseIn() const { return flag_.poseIn; }
  inline bool getFlagObsIn() const { return flag_.obsIn; }
  inline bool getFlagGoalIn() const { return flag_.goalIn; }
  inline bool getFlagGraphInit() const { return flag_.graphInit; }
  inline bool getFlagPathFound() const { return flag_.pathFound; }

  inline bool getParamIsVerbose() const { return param_.isVerbose; }
};

#endif  // CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_PLANNER_TRG_PLANNER_H_
