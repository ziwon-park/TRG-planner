#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "trg_planner/include/planner/trg_planner.h"

namespace py = pybind11;

PYBIND11_MODULE(trg_planner, m) {
  m.doc()               = "Pybind11 bindings for TRG-planner library";
  m.attr("__version__") = "1.0.0";

  py::class_<TRG, std::shared_ptr<TRG>>(m, "TRG")
      .def(py::init<bool, float, float, int, float, float, float, float, float>())
      .def("lockGraph", &TRG::lockGraph)
      .def("unlockGraph", &TRG::unlockGraph)
      .def("getGraph", &TRG::getGraph, "Get the graph", py::arg("type"));

  py::class_<TRG::Edge>(m, "Edge")
      .def(py::init<int, float, float>())
      .def_readwrite("dst_id", &TRG::Edge::dst_id_)
      .def_readwrite("weight", &TRG::Edge::weight_)
      .def_readwrite("dist", &TRG::Edge::dist_);

  py::enum_<TRG::NodeState>(m, "NodeState")
      .value("Valid", TRG::NodeState::Valid)
      .value("Invalid", TRG::NodeState::Invalid)
      .value("Frontier", TRG::NodeState::Frontier);

  py::class_<TRG::Node>(m, "Node")
      .def(py::init<int, Eigen::Vector2f&, float, TRG::NodeState>())
      .def_readwrite("id", &TRG::Node::id_)
      .def_readwrite("pos", &TRG::Node::pos_)
      .def_readwrite("state", &TRG::Node::state_)
      .def_readwrite("edges", &TRG::Node::edges_);

  py::class_<TRGPlanner, std::shared_ptr<TRGPlanner>>(m, "TRGPlanner")
      .def(py::init<>())
      .def("init", &TRGPlanner::init)
      .def("setParams", &TRGPlanner::setParams, "Set parameters from config file")

      // TRG functions
      .def("getTRG", &TRGPlanner::getTRG)

      // Mutex functions
      .def("lockOdom", &TRGPlanner::lockOdom)
      .def("unlockOdom", &TRGPlanner::unlockOdom)
      .def("lockObs", &TRGPlanner::lockObs)
      .def("unlockObs", &TRGPlanner::unlockObs)
      .def("lockGoal", &TRGPlanner::lockGoal)
      .def("unlockGoal", &TRGPlanner::unlockGoal)

      // Setters
      .def("setStateFrameId", &TRGPlanner::setStateFrameId, py::arg("frame_id"))
      .def("setStatePose3d", &TRGPlanner::setStatePose3d, py::arg("pose3d"))
      .def("setStatePose2d", &TRGPlanner::setStatePose2d, py::arg("pose2d"))
      .def("setStateQuat", &TRGPlanner::setStateQuat, py::arg("quat"))
      .def("setStateT_B2M", &TRGPlanner::setStateT_B2M, py::arg("T_B2M"))

      .def("setPreMapEigen", &TRGPlanner::setPreMapEigen, py::arg("preMapPtr"))
      .def("setObsEigen", &TRGPlanner::setObsEigen, py::arg("obsPtr"))

      .def("setGoalPose", &TRGPlanner::setGoalPose, py::arg("pose"))
      .def("setGoalQuat", &TRGPlanner::setGoalQuat, py::arg("quat"))
      .def("setGoalInit", &TRGPlanner::setGoalInit, py::arg("flag"))

      .def("setFlagPoseIn", &TRGPlanner::setFlagPoseIn, py::arg("flag"))
      .def("setFlagObsIn", &TRGPlanner::setFlagObsIn, py::arg("flag"))
      .def("setFlagGoalIn", &TRGPlanner::setFlagGoalIn, py::arg("flag"))
      .def("setFlagGraphInit", &TRGPlanner::setFlagGraphInit, py::arg("flag"))
      .def("setFlagPathFound", &TRGPlanner::setFlagPathFound, py::arg("flag"))

      // Getters
      .def("getRawPath", &TRGPlanner::getRawPath)
      .def("getSmoothPath", &TRGPlanner::getSmoothPath)
      .def("getDirectDist", &TRGPlanner::getDirectDist)
      .def("getRawPathLength", &TRGPlanner::getRawPathLength)
      .def("getSmoothPathLength", &TRGPlanner::getSmoothPathLength)
      .def("getPlanningTime", &TRGPlanner::getPlanningTime)
      .def("getAvgRisk", &TRGPlanner::getAvgRisk)

      .def("getStateFrameId", &TRGPlanner::getStateFrameId)
      .def("getStatePose3d", &TRGPlanner::getStatePose3d)
      .def("getStatePose2d", &TRGPlanner::getStatePose2d)
      .def("getStateQuat", &TRGPlanner::getStateQuat)
      .def("getStateT_B2M", &TRGPlanner::getStateT_B2M)

      .def("getPreMapEigen", &TRGPlanner::getPreMapEigen)
      .def("getObsEigen", &TRGPlanner::getObsEigen)

      .def("getGoalPose", &TRGPlanner::getGoalPose)
      .def("getGoalQuat", &TRGPlanner::getGoalQuat)
      .def("getGoalInit", &TRGPlanner::getGoalInit)

      .def("getFlagPoseIn", &TRGPlanner::getFlagPoseIn)
      .def("getFlagObsIn", &TRGPlanner::getFlagObsIn)
      .def("getFlagGoalIn", &TRGPlanner::getFlagGoalIn)
      .def("getFlagGraphInit", &TRGPlanner::getFlagGraphInit)
      .def("getFlagPathFound", &TRGPlanner::getFlagPathFound);
}
