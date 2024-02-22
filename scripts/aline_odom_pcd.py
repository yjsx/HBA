import os
import numpy as np
import sys
import shutil
# 读取n*8数组文件
data_root = sys.argv[1]

out_path = os.path.join(data_root,"pcd_rm")
if not os.path.exists(out_path):

    os.makedirs(out_path)
    print(f"Directory '{out_path}' was created.")
else:
    print(f"Directory '{out_path}' already exists.")

data = np.loadtxt(os.path.join(data_root, "odom.txt")) # 替换为你的n*8数组文件路径
timestamps_data = data[:, 0] # 假设第一列是时间戳

# 读取pcd文件
pcd_folder = os.path.join(data_root, "pcd") # 替换为你的pcd文件夹路径
pcd_list = [f.replace('.pcd', '') for f in os.listdir(pcd_folder) if f.endswith('.pcd')]
pcd_list = sorted(pcd_list)
# 找出并集

i = 0
j = 0
data_new = []
while(i < len(timestamps_data) and j < len(pcd_list)):
    timestamps_pcd = float(pcd_list[j])
    if(abs(timestamps_data[i] - timestamps_pcd) < 0.005):
        data_new.append(data[i][1:])
        i += 1
        j += 1
    elif(timestamps_data[i] > timestamps_pcd):
        f = pcd_list[j] + ".pcd"
        shutil.move(os.path.join(data_root,"pcd", f), os.path.join(data_root,"pcd_rm", f))
        j += 1
        print("j:", j)
    elif(timestamps_data[i] < timestamps_pcd):
        i += 1
        print("i:", i)
print(np.array(data_new).shape)
np.savetxt(os.path.join(data_root, "pose.json"), np.array(data_new))