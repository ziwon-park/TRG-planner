#include "planner/trg_planner.h"

TRGPlanner::TRGPlanner() {}
TRGPlanner::~TRGPlanner() {
  thd.graph.join();
  thd.planning.join();
}

void TRGPlanner::init() {
  //// Load TRG
  trg_ = std::make_shared<TRG>(param_.isVerbose,
                               param_.expandDist,
                               param_.robotSize,
                               param_.sampleNum,
                               param_.heightThreshold,
                               param_.collisionThreshold,
                               param_.updateCollisionThreshold,
                               param_.safetyFactor,
                               param_.goal_tolerance);

  if (trg_ == nullptr) {
    print_error("Failed to initialize TRG");
    exit(1);
  }

  //// Load prebuilt map
  if (param_.isPreMap) {
    cs_.preMapPtr.reset(new pcl::PointCloud<PtsDefault>());
    cs_.preMapPtr->clear();
    loadPrebuiltMap();
    if (cs_.preMapPtr == nullptr) {
      print_error("Failed to load prebuilt map");
      exit(1);
    }
    trg_->setGlobalMap(cs_.preMapPtr);
  }

  //// Initialize observation map
  cs_.obsPtr.reset(new pcl::PointCloud<PtsDefault>());

  //// TODO: Load prebuilt graph
  if (param_.isPreGraph) {
    print("Prebuilt graph is loaded");
  } else {
    print("Prebuilt graph is not loaded");
  }

  //// Initialize FSM Threads
  thd.graph    = std::thread(&TRGPlanner::runGraphFSM, this);
  thd.planning = std::thread(&TRGPlanner::runPlanningFSM, this);
}

void TRGPlanner::loadPrebuiltMap() {
  std::string map_type = param_.preMapPath.substr(param_.preMapPath.find_last_of(".") + 1);
  if (map_type != "pcd") {
    print_error("Unsupported map type");
    exit(1);
  }

  std::string                 abs_path = std::string(TRG_DIR) + "/" + param_.preMapPath;
  pcl::PointCloud<PtsDefault> rawMap   = pcl::PointCloud<PtsDefault>();
  if (pcl::io::loadPCDFile<PtsDefault>(abs_path, rawMap) == -1) {
    print_error("Failed to load prebuilt map");
    exit(1);
  }

  if (param_.isVoxelize) {
    pcl::VoxelGrid<PtsDefault> vg;
    vg.setInputCloud(rawMap.makeShared());
    vg.setLeafSize(param_.VoxelSize, param_.VoxelSize, param_.VoxelSize);
    vg.filter(*cs_.preMapPtr);
  } else {
    *cs_.preMapPtr = rawMap;
  }
  print("Prebuilt map size: " + std::to_string(rawMap.size()) + " -> " +
        std::to_string(cs_.preMapPtr->size()));
  print("Prebuilt map is loaded");
}

void TRGPlanner::runGraphFSM() {
  while (is_running.load()) {
    auto start_loop = tic();
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));

    fsm_.graph.notice();
    switch (fsm_.graph.curr_state_) {
      case graphState::INIT: {
        if (flag_.graphInit) {
          fsm_.graph.transition(graphState::UPDATE);
          break;
        }

        if (param_.isPreMap) {
          auto start_init_graph = tic();
          trg_->initGraph(param_.isPreMap, state_.pose3d);
          print_warning("Graph initialization time: " + std::to_string(toc(start_init_graph, "s")) +
                        " sec");
          flag_.graphInit = true;
          fsm_.graph.transition(graphState::UPDATE);
          break;
        }

        if (!flag_.poseIn || !flag_.obsIn) {
          print_warning("Pose: " + std::to_string(flag_.poseIn) +
                        ", Obs: " + std::to_string(flag_.obsIn));
          fsm_.graph.transition(graphState::INIT);
          break;
        }
        mtx.obs.lock();
        trg_->setGlobalMap(cs_.obsPtr);
        mtx.obs.unlock();

        auto start_init_graph = tic();
        trg_->initGraph(param_.isPreMap, state_.pose3d);
        print_warning("Graph initialization time: " + std::to_string(toc(start_init_graph, "s")) +
                      " sec");
        flag_.graphInit = true;
        fsm_.graph.transition(graphState::UPDATE);
        break;
      }
      case graphState::UPDATE: {
        if (param_.isUpdate) {
          if (!flag_.poseIn || !flag_.obsIn) {
            print_warning("Pose: " + std::to_string(flag_.poseIn) +
                          ", Obs: " + std::to_string(flag_.obsIn));
            fsm_.graph.transition(graphState::UPDATE);
            break;
          }
          auto start = tic();
          mtx.obs.lock();  // too slow, and not necessary (then, comment this line)
          trg_->setLocalMap(state_.pose2d, cs_.obsPtr);
          if (!param_.isPreMap) trg_->setGlobalMap(cs_.obsPtr);
          mtx.obs.unlock();
          trg_->updateGraph();
          print_success("Graph update time: " + std::to_string(toc(start, "ms")) + " ms");
        }
        fsm_.graph.transition(graphState::UPDATE);
        break;
      }
      case graphState::LOAD: {
        break;
      }
      case graphState::RESET: {
        break;
      }
      case graphState::SAVE: {
        break;
      }
      default: {
        print_error("Invalid graph state");
        exit(1);
      }
    }
    float loop_time   = toc(start_loop, "ms");
    int   remain_time = 1000 / param_.graph_rate - loop_time;
    if (remain_time > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(remain_time));
    }
    thd.hz["graph"] = std::round(1000 / toc(start_loop, "ms") * 100) / 100;
  }
}

void TRGPlanner::runPlanningFSM() {
  while (is_running.load()) {
    auto start_loop = tic();
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));

    if (flag_.goalIn) {
      flag_.goalIn = false;
      fsm_.planning.transition(planningState::PLANNING);
    }
    fsm_.planning.notice();
    switch (fsm_.planning.curr_state_) {
      case planningState::RESET: {
        flag_.pathFound = false;
        path_.clear();
        fsm_.planning.transition(planningState::RESET);
        break;
      }
      case planningState::PLANNING: {
        path_.clear();
        auto start = tic();
        if (trg_->planSafePath(state_.pose2d,
                               goal_state_.pose,
                               path_.raw,
                               path_.direct_dist,
                               path_.raw_path_length,
                               path_.avg_risk)) {
          path_.planning_time = toc(start, "ms");
          print_success("Path planning time: " + std::to_string(path_.planning_time) + " ms");
          flag_.pathFound     = true;
          flag_.planningCount = 0;
          trg_->refinePath(path_.raw, path_.smooth);
          fsm_.planning.transition(planningState::ONGOING);
        } else {
          print_error("Failed to find a path, count: " + std::to_string(flag_.planningCount));
          flag_.planningCount++;
          if (flag_.planningCount > 10) {
            fsm_.planning.transition(planningState::RESET);
          } else {
            fsm_.planning.transition(planningState::PLANNING);
          }
        }
        break;
      }
      case planningState::ONGOING: {
        if (trg_->checkReadched(state_.pose2d)) {
          print_success("Goal reached");
          fsm_.planning.transition(planningState::RESET);
          break;
        }
        if (trg_->checkReplan(state_.pose2d, path_.raw)) {
          print_warning("Replanning");
          fsm_.planning.transition(planningState::PLANNING);
          break;
        }
        fsm_.planning.transition(planningState::ONGOING);
        break;
      }
      default: {
        print_error("Invalid planning state");
        exit(1);
      }
    }
    float loop_time   = toc(start_loop, "ms");
    int   remain_time = 1000 / param_.planning_rate - loop_time;
    if (remain_time > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(remain_time));
    }
    thd.hz["planning"] = std::round(1000 / toc(start_loop, "ms") * 100) / 100;
  }
}

void GraphFSM::transition(graphState new_state) {
  prev_state_ = curr_state_;
  curr_state_ = new_state;
  if (prev_state_ != curr_state_) {
    print("[Graph] " + state_map_[prev_state_] + " -> " + state_map_[curr_state_]);
  }
}

void GraphFSM::notice() {
  if (prev_state_ != curr_state_) {
    print("[Graph] " + state_map_[curr_state_]);
  }
}

void PlanningFSM::transition(planningState new_state) {
  prev_state_ = curr_state_;
  curr_state_ = new_state;
  if (prev_state_ != curr_state_) {
    print("[Planning] " + state_map_[prev_state_] + " -> " + state_map_[curr_state_]);
  }
}

void PlanningFSM::notice() {
  if (prev_state_ != curr_state_) {
    print("[Planning] " + state_map_[curr_state_]);
  }
}
