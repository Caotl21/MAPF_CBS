import numpy as np
import matplotlib.pyplot as plt

# 定义32x32的栅格地图
grid_map = np.zeros((32, 32), dtype=int)

# 设置货架区域（值为1）
grid_map[2:30, 2:30] = 1  # 中间区域为货架

# 设置障碍物（值为2）
grid_map[10:12, 5:10] = 2  # 示例障碍物
grid_map[20:22, 15:20] = 2  # 示例障碍物

# 设置入口/出口（值为3）
grid_map[0, 16] = 3  # 顶部中间为入口
grid_map[31, 16] = 3  # 底部中间为出口

# 定义颜色映射
cmap = plt.cm.colors.ListedColormap(['white', 'blue', 'red', 'green'])
bounds = [0, 1, 2, 3, 4]
norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

# 绘制地图
plt.figure(figsize=(6, 6))
plt.imshow(grid_map, cmap=cmap, norm=norm)

# 添加网格线
plt.grid(which='major', axis='both', linestyle='-', color='black', linewidth=0.5)
plt.xticks(np.arange(0, 32, 1))
plt.yticks(np.arange(0, 32, 1))

# 添加图例
legend_labels = {'0: 空闲区域': 'white', '1: 货架': 'blue', '2: 障碍物': 'red', '3: 入口/出口': 'green'}
patches = [plt.plot([], [], marker="s", ms=10, ls="", color=color, label=label)[0] for label, color in legend_labels.items()]
plt.legend(handles=patches, bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

# 保存为图像文件
plt.savefig('warehouse_map.png', bbox_inches='tight', dpi=300)
plt.show()