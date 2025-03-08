import yaml
import os
import time
import sys
import trg_planner
import numpy as np


# Get the path of root directory outside the package
root_directory = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Get the path to the configuration files
mag_config_file = os.path.join(root_directory, 'config', 'mountain.yaml')
# Get the path to the configuration files

print(f"Package share directory: {mag_config_file}")

tp = trg_planner.TRGPlanner()
tp.setParams(mag_config_file)
tp.init()

tp.setStateFrameId("mapp")
tp.setStatePose3d(np.array([0.0, 1.0, 2.0]))
tp.setStatePose2d(np.array([0.0, 1.0]))
print("frame_id: ", tp.getStateFrameId())
print("pose3d: ", tp.getStatePose3d())
print("pose2d: ", tp.getStatePose2d())
# tp.setStateQuat(np.array([1.0, 0.0, 0.0, 0.0]))
# T_B2M is 4by4 matrix
tp.setStateT_B2M(np.eye(4))
print("T_B2M: ", tp.getStateT_B2M())

# multiple rows with 3 elements
mat = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
tp.setPreMapEigen(mat)
print("PreMapEigen: ", tp.getPreMapEigen())

mat2 = np.array([[1.0, 2.0, 3.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
tp.setObsEigen(mat2)
print("ObsEigen: ", tp.getObsEigen())


trg = tp.getTRG()
print("TRG: ", trg)

trg.lockGraph()
print("TRG locked")



type = "global"
nodes = trg.getGraph(type)

print("node type: ", type)
# print("Nodes: ", nodes)
print("node size: ", len(nodes))
print("node 0: ", nodes[0])
cnt = 0
for key, value in nodes.items():
    if cnt > 1:
        break
    print(key, value.id, value.pos, value.state, len(value.edges))
    for edge in value.edges:
        print(edge.dst_id, edge.weight, edge.dist)
    cnt += 1
trg.unlockGraph()
print("TRG unlocked")

print("raw path: ", tp.getRawPath())


while True:
    time.sleep(1)
