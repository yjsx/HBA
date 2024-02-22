# 这个程序是评价评价地图指标的程序

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns


file_path = "/home/yjsx/catkin_yjsx/src/HBA/scripts/data/172_2.txt"
records = []
# 打开文件，逐行读取
with open(file_path, 'r') as file:
    for line in file:
        # 每一行是一个字典
        data_dict = {}
        # 拆分': '之后的部分然后以', '分割这几部分，得到值的数组
        parts = line.strip().split(', ')
        # 解析每个值（确保在分割字符串之前去除了空白符）
        for part in parts:
            # 以': '分割键和值
            key, value = part.split(': ')
            data_dict[key.strip()] = float(value.strip())
        print(data_dict)
        # 将解析后的字典添加到列表中
        records.append(data_dict)

df = pd.DataFrame(records)

# 按radius和top_ratio分组计算平均值
grouped = df.groupby(['radius', 'top_ratio'])
mean_scores = grouped['score'].mean().reset_index()

# Group the data
groups = df[['radius', 'top_ratio']].drop_duplicates()

# 预分配结果的 ndarray
mean_values = np.zeros(len(groups))
std_values = np.zeros(len(groups))
coef_variation_values = np.zeros(len(groups))
range_ratio = np.zeros(len(groups))

# 遍历每个唯一的组合
for i, (radius, top_ratio) in enumerate(groups.values):
    # 提取当前组的 'score'
    group_scores = df[(df['radius'] == radius) & (df['top_ratio'] == top_ratio)]['score'].values
    
    # 计算当前组的最小值和最大值
    min_score = np.min(group_scores)
    max_score = np.max(group_scores)
    range_score = max_score - min_score
    mean_values[i] = np.mean(group_scores)
    range_ratio[i] = range_score / mean_values[i]   
    if(radius == 1 and top_ratio == 0.02):
        print("####################", min_score, max_score, range_score, range_score / max_score, mean_values[i], range_score/mean_values[i])
    # 检查 range_score 是否为零，以避免除以零
    if range_score > 0:
        normalized_scores = (group_scores - min_score) / range_score
    else:
        normalized_scores = group_scores - min_score
    
    # 计算标准化后的得分的平均值和标准差
    mean_values[i] = np.mean(normalized_scores)
    std_values[i] = np.std(normalized_scores)
    
    # 计算变异系数，处理平均值为0的情况
    if mean_values[i] != 0:
        coef_variation_values[i] = std_values[i] / mean_values[i]
    else:
        coef_variation_values[i] = np.nan
    

# 将 numpy 数组转换为 DataFrame
result_df = pd.DataFrame({
    'radius': groups['radius'].values,
    'top_ratio': groups['top_ratio'].values,
    'mean_score': mean_values,
    'std_score': std_values,
    'coef_variation': coef_variation_values,
    'range_ratio': range_ratio
})

print(result_df)


# 将平均值转换成适合热力图绘制的格式



# 绘制热力图
plt.figure(figsize=(8, 6))
# range_ratio  coef_variation
pivot_table = result_df.pivot(index='radius', columns='top_ratio', values='range_ratio')
sns.heatmap(pivot_table, annot=True, fmt=".4f", cmap='Blues', cbar_kws={'label': 'range_ratio'}, vmin=0, vmax=1)
plt.title("Heatmap of range_ratio by Radius and Top Ratio")
plt.ylabel('Radius')
plt.xlabel('Top Ratio')

# 显示热力图
plt.show()
