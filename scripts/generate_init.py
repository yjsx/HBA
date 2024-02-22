import os
import json
import sys
folder_path = sys.argv[1] # 设置要查看的文件夹路径
file_count = len(os.listdir(folder_path+"/pcd"))  # 统计文件夹中文件数量
print('There are', file_count, 'files in the folder.')

# 生成pose.json文件
pose_data = '0 0 0 0 0 0 1\n' * file_count
with open(os.path.join(folder_path, 'pose.json'), 'w') as f:
    f.write(pose_data[:-1])