import numpy as np
import os
import sys
from scipy.spatial.transform import Rotation as R

def matric_to_tum(pose):
    rotation = R.from_matrix(pose[:3,:3]).as_quat()
    transport = pose[:3,3].T
    out = list(transport) + list(rotation)
    return out

def quan_to_matric(pose):
    x, y, z, w = pose
    q = np.array([x, y, z, w])  
    rotation = R.from_quat(q)
    rot_matrix = rotation.as_matrix()
    return rot_matrix

def normalize_quaternion(q):
    norm = np.linalg.norm(q)  # 计算四元数的模长
    if norm > 0:  # 防止除以0
        return q / norm  # 返回归一化的四元数
    else:
        raise ValueError("Cannot normalize a zero quaternion.") 

def tum_to_matric(pose):
    rotation = quan_to_matric(normalize_quaternion(pose[3:]))
    transport = pose[0:3]
    mat = np.hstack([np.array(rotation), np.array([transport]).T])
    mat = np.vstack([np.array(mat), np.array([[0,0,0,1]]) ])
    return mat

def get_pose(odom_path):
    pose_list = []
    f = open(os.path.join(odom_path), "r")
    for line in f.readlines():
        # pose = np.array([float(x) for x in line.split(' ')]).reshape(3,4)
        pose = tum_to_matric([float(x) for x in line.split(' ')])
        # pose = np.insert(pose,3,values=[0,0,0,1],axis=0)
        pose_list.append(pose) 
    return pose_list

inpose_path = sys.argv[1]
outpose_path = sys.argv[2]
pose_list = get_pose(inpose_path)

new_pose = np.empty((0, 12))

for pose in pose_list:
    new_pose = np.concatenate((new_pose, pose[:3].reshape(1, -1)))
np.savetxt(outpose_path, new_pose)
