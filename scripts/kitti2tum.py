import numpy as np
import os
import sys
from scipy.spatial.transform import Rotation as R
def matric_to_tum(pose):
    rotation = R.from_matrix(pose[:3,:3]).as_quat()
    transport = pose[:3,3].T
    out = list(transport) + list(rotation)
    return out

def get_pose(odom_path):
    pose_list = []
    f = open(os.path.join(odom_path), "r")
    for line in f.readlines():
        pose = np.array([float(x) for x in line.split(' ')]).reshape(3,4)
        pose = np.insert(pose,3,values=[0,0,0,1],axis=0)
        pose_list.append(pose) 
    return pose_list

inpose_path = sys.argv[1]
outpose_path = sys.argv[2]
pose_list = get_pose(inpose_path)
f = open(outpose_path, "w")
for pose in pose_list:
    f.write(' '.join(str(e) for e in matric_to_tum(pose)))
    f.write("\n")
f.close()