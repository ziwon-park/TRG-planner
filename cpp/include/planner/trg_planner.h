#ifndef CPP_INCLUDE_PLANNER_TRG_PLANNER_H_
#define CPP_INCLUDE_PLANNER_TRG_PLANNER_H_

#include "graph/trg.h"
#include "utils/common.h"

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

 public:
  struct Mutex {
    std::mutex odom;
    std::mutex obs;
    std::mutex goal;
  } mtx;

  struct Thread {
    std::thread                            graph;
    std::thread                            planning;
    std::thread                            publish;
    std::thread                            debug;
    std::unordered_map<std::string, float> hz;
  } thd;

  struct Parameters {
    /// debug
    bool isVerbose = true;

    /// Timer parameters
    float graph_rate;
    float planning_rate;
    float publish_rate;
    float debug_rate;

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
    std::string        frame_id = "map";
    Eigen::Vector3f    pose3d   = Eigen::Vector3f::Zero();
    Eigen::Vector2f    pose2d   = Eigen::Vector2f::Zero();
    Eigen::Quaternionf quat     = Eigen::Quaternionf::Identity();
    Eigen::Matrix4f    T_B2M    = Eigen::Matrix4f::Identity();  // tf from map to base
  } state_;

  struct cSpace {
    PointCloudPtr preMapPtr = nullptr;
    PointCloudPtr obsPtr    = nullptr;
  } cs_;

  struct goalState {
    Eigen::Vector3f    pose;
    Eigen::Quaternionf quat;
    bool               init = false;
  } goal_state_;

  struct Flag {
    bool poseIn = false;
    bool obsIn  = false;
    bool goalIn = false;

    bool graphInit     = false;
    bool pathFound     = false;
    int  planningCount = 0;
  } flag_;
};

#endif  // CPP_INCLUDE_PLANNER_TRG_PLANNER_H_
