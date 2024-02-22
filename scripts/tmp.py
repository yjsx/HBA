import open3d as o3d
import os

for root, dirs, files in os.walk("/home/yjsx/dataset/hba/xyy_all/pcd"):
    for file in files:
        path = os.path.join(root, file)
        print(path)

        cloud = o3d.io.read_point_cloud(path)
        o3d.visualization.draw_geometries([cloud])
        o3d.io.write_point_cloud(path, cloud)