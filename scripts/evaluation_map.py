import open3d as o3d
import numpy as np
import random
import sys
from scipy.spatial import cKDTree
from tqdm import tqdm

def calculate_area(pcd):
    points = np.asarray(pcd.points)

    # XY平面上点的范围
    min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
    min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])

    # 网格的分辨率（每个格子的实际大小）
    resolution = 1  # 表示每个像素代表 0.1x0.1 平方单位的面积

    # 创建网格数组
    grid_size_x = int((max_x - min_x) / resolution)
    grid_size_y = int((max_y - min_y) / resolution)
    grid = np.zeros((grid_size_y+1, grid_size_x+1), dtype=int)

    # 投射点云到网格上
    for point in points:
        x, y = point[:2]
        grid_x = int((x - min_x) / resolution)
        grid_y = int((y - min_y) / resolution)
        grid[grid_y, grid_x] += 1  # 占据的像素设置为1

    # 计算占据的总面积
    occupied_cells = np.sum(grid >= 10)

    # 将占据的格子数量转换为面积
    occupied_area = occupied_cells * resolution**2
    print(f"Number of occupied cells: {occupied_cells}")
    print(f"Estimated occupied area: {occupied_area} square units")
    return occupied_area

def get_eigenvalues(points):
    # points = np.asarray(pcd.points)
    
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    cov_matrix = np.cov(centered_points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    # print("特征值：", eigenvalues)
    return min(eigenvalues)

def sample_points_and_extract_pcd(map_pcd, n_samples, radius, top_ratio):

    map_points = np.asarray(map_pcd.points)
    sampled_indices = []
    new_clouds = []
    i = 0
    while i < n_samples:
        # 随机采样一个点，确保不与之前采样的点在radius内重叠
        # while True:
        sample_index = random.randint(0, len(map_points) - 1)
        sample_point = map_points[sample_index]
            # overlap = any(np.linalg.norm(sample_point - map_points[i]) < radius*2 for i in sampled_indices)
            # if not overlap:
                # sampled_indices.append(sample_index)
                # break   
        
        # 为采样点创建一个球体搜索对象
        kdtree = cKDTree(map_points)
        indices = kdtree.query_ball_point(map_points[sample_index], radius)
        # 从主点云中提取球体内的点云
        new_points = map_points[indices]

        # 储存新的点云
        if(len(new_points) < 100):
            continue
        eigen = get_eigenvalues(new_points)
        new_clouds.append((new_points, eigen))
        i += 1

        # # 从候选点集中移除已采样的点云的点
        # mask = np.array([True] * len(points))
        # mask[idx] = False
        # points = points[mask]

        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(points)
    sorted_pcd_list = sorted(new_clouds, key=lambda x: x[1])
    plane_points =  []
    other_points =  []
    sum = 0
    num = 0
    for i, pair in enumerate(sorted_pcd_list):
        if(pair[1] == -1):
            continue
        if(i < top_ratio * len(sorted_pcd_list)):
            sum += pair[1]
            # print(pair[1])
            num += 1
            plane_points.append(pair[0])
        else:
            other_points.append(pair[0])
    plane_points = np.vstack(plane_points)
    other_points = np.vstack(other_points)


    plane_pcd = o3d.geometry.PointCloud()
    plane_pcd.points = o3d.utility.Vector3dVector(plane_points)
    plane_pcd.paint_uniform_color([1, 0, 0])
    other_pcd = o3d.geometry.PointCloud()
    other_pcd.points = o3d.utility.Vector3dVector(other_points)
    other_pcd.paint_uniform_color([0, 0, 1])
    # print(sum, n_samples)
    # print("radius: %d, top_ratio: %d, score: %d", radius, top_ratio, sum/n_samples / top_ratio * 1000)
    print("radius: {}, top_ratio: {}, score: {}".format(radius, top_ratio, sum / num * 1000))

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()	#创建窗口
    # render_option: o3d.visualization.RenderOption = vis.get_render_option()	#设置点云渲染参数
    # render_option.background_color = np.array([0, 0, 0])	#设置背景色（这里为黑色

    # vis.add_geometry(map_pcd)    
    # vis.add_geometry(plane_pcd)    
    # vis.add_geometry(other_pcd)    
    # vis.run()

# 读取pcd文件
def main():
    path = sys.argv[1]
    radius = float(sys.argv[2])
    pcd = o3d.io.read_point_cloud(path)
    pcd = pcd.voxel_down_sample(0.1)

    n_samples = int(calculate_area(pcd) / 5)
    # radius = 1.0   # 采样半径为1米
    # for radius in [0.5, 1, 2, 3]:
    for radius in [radius]:
        for top_ratio in [0.01, 0.02, 0.03, 0.04, 0.05, 0.1, 0.2, 0.3]:
            for _ in range(10):
                sample_points_and_extract_pcd(pcd, n_samples, radius, top_ratio)
main()


# 0.1417 0.0610 0.4668 0.396 0.2778 0.0581