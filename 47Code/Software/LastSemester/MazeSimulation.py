import numpy as np
import matplotlib.pyplot as plt
import math
import random
import heapq
from matplotlib.animation import FuncAnimation
import os
import json
from collections import deque
from matplotlib.patches import FancyArrowPatch
from breezyslam.sensors import RPLidarA1
from sklearn.cluster import DBSCAN
from scipy.spatial import distance
from scipy.ndimage import binary_dilation

MAP_SIZE_PIXELS = 300.0

class ImprovedMazeSLAM:
    def __init__(self, maze_json_path):
        """初始化SLAM系统，读取json文件"""
        if not os.path.exists(maze_json_path):
            raise FileNotFoundError(f"File not found: {maze_json_path}")
        
        # 读取json文件
        with open(maze_json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        self.segments = []
        for seg in data["segments"]:
            s = {"start": [seg["start"][0]+1, seg["start"][1]+1], "end": [seg["end"][0]+1, seg["end"][1]+1]}
            self.segments.append(s)
        self.start_point = [data["start_point"][0]+1, data["start_point"][1]+1]

        # 计算地图尺寸（自动适应线段最大范围）
        max_x = max(max(seg["start"][0], seg["end"][0]) for seg in self.segments)
        max_y = max(max(seg["start"][1], seg["end"][1]) for seg in self.segments)
        min_x = min(min(seg["start"][0], seg["end"][0]) for seg in self.segments)
        min_y = min(min(seg["start"][1], seg["end"][1]) for seg in self.segments)
        self.left_x = min_x
        self.right_x = max_x
        self.bottom_y = min_y
        self.top_y = max_y

        # 地图参数
        self.resolution = max(max_x, max_y) / MAP_SIZE_PIXELS  # 米/像素
        map_width = int((max_x + 1) / self.resolution)
        map_height = int((max_y + 1) / self.resolution)
        self.map_size = (map_height, map_width)
        # 起点边界修正
        three_pixel_width = 3 * self.resolution
        if abs(self.start_point[0] - self.left_x) < 1e-3:
            self.start_point[0] += three_pixel_width
        if abs(self.start_point[0] - self.right_x) < 1e-3:
            self.start_point[0] -= three_pixel_width
        if abs(self.start_point[1] - self.top_y) < 1e-3:
            self.start_point[1] -= three_pixel_width
        if abs(self.start_point[1] - self.bottom_y) < 1e-3:
            self.start_point[1] += three_pixel_width
        # 初始化地图
        self.occupancy_map = np.full(self.map_size, 0.5)  # 0.5=未知, 0=空闲, 1=占用
        self.maze_img = np.ones(self.map_size, dtype=np.float32)  # 1=空闲（白色）
        self.obstacle_mask = np.zeros(self.map_size, dtype=bool)  # 障碍物掩码

        # 生成迷宫障碍物（线段宽度3像素，膨胀）
        temp_mask = np.zeros(self.map_size, dtype=bool)
        for seg in self.segments:
            x0, y0 = seg["start"]
            x1, y1 = seg["end"]
            points = self._bresenham(
                int(x0 / self.resolution),
                int(self.map_size[0] - y0 / self.resolution),
                int(x1 / self.resolution),
                int(self.map_size[0] - y1 / self.resolution)
            )
            for x, y in points:
                if 0 <= x < self.map_size[1] and 0 <= y < self.map_size[0]:
                    temp_mask[y, x] = True
        # 膨胀3像素
        temp_mask = binary_dilation(temp_mask, iterations=1)
        temp_mask = binary_dilation(temp_mask, iterations=1)
        self.maze_img[temp_mask] = 0.0
        self.obstacle_mask[temp_mask] = True

        # 生成迷宫障碍物
        """for seg in self.segments:
            x0, y0 = seg["start"]
            x1, y1 = seg["end"]
            self._draw_line_on_maze(x0, y0, x1, y1)"""

        # 小车参数
        self.robot_pose = (self.start_point[0], self.start_point[1], 0.0)
        self.robot_path = [self.start_point[:2]]  # 记录路径
        self.robot_size = 0.3
        self.lidar_range = 12.0
        self.lidar_resolution = 1  # 降低分辨率提高性能
        
        # 探索策略参数
        self.frontier_cells = deque()
        self.current_target = None
        self.obstacle_threshold = 0.7  # 障碍物判断阈值

        # 记录所有被雷达探索过的目标点
        self.visited_targets = set()
        
        # 存储激光可达点
        self.laser_reachable_points = []
        
        # A*路径规划结果
        self.path = []
        self.path_points = None  # 用于可视化规划路径的点集

        # 出口记录
        self.exit_points = []  # [(x, y)]
        self.best_exit = None  # 当多个出口时选择最优出口
        
        # 探索状态
        self.exploration_state = "exploring"  # "exploring", "returning_to_exit"
        self.all_frontiers_explored = False  # 是否已探索完所有前沿点

        # 是否已完成返回出口，准备回到起点
        self.returned_to_exit = False

        # 可视化设置
        self.fig, self.ax = plt.subplots(1, 2, figsize=(14, 7))
        
        # 左侧：真实地图
        self.ax[0].imshow(self.maze_img, cmap='binary', 
                         extent=[0, self.map_size[1]*self.resolution, 
                                0, self.map_size[0]*self.resolution])
        # 直接画线段
        for seg in self.segments:
            x0, y0 = seg["start"]
            x1, y1 = seg["end"]
            self.ax[0].plot([x0, x1], [y0, y1], color='black', linewidth=2, zorder=10)
        # 创建箭头形状的小车
        x, y, theta = self.robot_pose
        arrow_length = 0.8
        dx = arrow_length * math.cos(theta)
        dy = arrow_length * math.sin(theta)
        self.robot_arrow = FancyArrowPatch((x, y), (x + dx, y + dy),
                                         arrowstyle='-|>', mutation_scale=15,
                                         color='red', zorder=5)
        self.ax[0].add_patch(self.robot_arrow)
        self.path_line, = self.ax[0].plot([], [], 'r-', alpha=0.5, linewidth=1)
        self.scan_points = self.ax[0].scatter([], [], c='blue', s=15, alpha=0.7)
        self.scan_lines = []
        self.target_icon = self.ax[0].scatter([], [], c='lime', marker='*', s=200, zorder=4)
        
        # 前沿聚落可视化
        self.frontier_points = self.ax[0].scatter([], [], c='magenta', s=10, alpha=0.7)
        self.cluster_centers = self.ax[0].scatter([], [], c='yellow', marker='x', s=100, zorder=3)
        # 添加A*规划路径的可视化
        self.path_points = self.ax[0].scatter([], [], c='cyan', marker='.', s=30, zorder=2)
        
        # 添加地图完整度文本
        self.completeness_text = self.ax[0].text(
            0.05, 0.95, '', transform=self.ax[0].transAxes, 
            fontsize=9, verticalalignment='top', 
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        
        self.ax[0].set_title('Ground Truth & Frontiers')
        
        # 右侧：SLAM地图
        self.slam_img = self.ax[1].imshow(self.occupancy_map.copy(), cmap='gray',
                                        extent=[0, self.map_size[1]*self.resolution,
                                               0, self.map_size[0]*self.resolution],
                                        vmin=0, vmax=1)
        # 创建探索区域的显示，使用普通数组而不是掩码数组避免警告
        explored_overlay = np.zeros(self.map_size)
        explored_overlay[self.occupancy_map != 0.5] = 1  # 探索过的区域标记为1
        self.explored_img = self.ax[1].imshow(explored_overlay,
                                            cmap='gray', alpha=0.3,
                                            extent=[0, self.map_size[1]*self.resolution,
                                                   0, self.map_size[0]*self.resolution])
        self.ax[1].set_title('SLAM Map')
        
        # 统一坐标范围并移除坐标轴
        for ax in self.ax:
            ax.set_xlim(0, self.map_size[1]*self.resolution)
            ax.set_ylim(0, self.map_size[0]*self.resolution)
            ax.grid(False)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.axis('off')

    def _draw_line_on_maze(self, x0, y0, x1, y1, thickness=3):
        """在maze_img上画线段（Bresenham算法），并将线段栅格设为障碍物，支持多像素宽度"""
        x0_pix = int(x0 / self.resolution)
        y0_pix = int(self.map_size[0] - y0 / self.resolution)
        x1_pix = int(x1 / self.resolution)
        y1_pix = int(self.map_size[0] - y1 / self.resolution)
        points = self._bresenham(x0_pix, y0_pix, x1_pix, y1_pix)
        radius = thickness // 2
        for x, y in points:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    xx, yy = x + dx, y + dy
                    if 0 <= xx < self.map_size[1] and 0 <= yy < self.map_size[0]:
                        self.maze_img[yy, xx] = 0.0  # 0为障碍物（黑色）
                        self.obstacle_mask[yy, xx] = True  # 标记为障碍物

    def _bresenham(self, x0, y0, x1, y1):
        """Bresenham直线算法"""
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return points

    def world_to_pixel(self, x, y):
        """世界坐标转像素坐标"""
        return int(x / self.resolution), int(self.map_size[0] - y / self.resolution)

    def pixel_to_world(self, px, py):
        """像素坐标转世界坐标"""
        x = px * self.resolution
        y = (self.map_size[0] - py) * self.resolution
        return x, y

    def simulate_lidar(self):
        """激光雷达扫描，检测障碍物和出口"""
        scan_points = []
        x, y, theta = self.robot_pose
        self.laser_reachable_points = []
        for line in self.scan_lines:
            line.remove()
        self.scan_lines = []

        for angle_deg in range(0, 360, self.lidar_resolution):
            angle_rad = math.radians(angle_deg) + theta
            dx, dy = math.cos(angle_rad), math.sin(angle_rad)
            hit_obstacle = False
            last_px, last_py = None, None
            for distance in np.arange(0.1, self.lidar_range, 0.1):
                px, py = x + distance * dx, y + distance * dy
                pix_x, pix_y = self.world_to_pixel(px, py)
                if not (0 <= pix_x < self.map_size[1] and 0 <= pix_y < self.map_size[0]):
                    # 检查与边界的交点
                    boundary_cross, cross_x, cross_y = self._find_boundary_cross(x, y, dx, dy)
                    if boundary_cross:
                        # 验证该交点是否为无障碍的已知点
                        cross_pix_x, cross_pix_y = self.world_to_pixel(cross_x, cross_y)
                        if (0 <= cross_pix_x < self.map_size[1] and 0 <= cross_pix_y < self.map_size[0]):
                            occ = self.occupancy_map[cross_pix_y, cross_pix_x]
                            obs = self.obstacle_mask[cross_pix_y, cross_pix_x]
                            if occ < 0.5 and not obs:
                                self._record_exit_point(cross_x, cross_y)
                    break
                self.laser_reachable_points.append((px, py))
                last_px, last_py = px, py
                if self.obstacle_mask[pix_y, pix_x]:
                    scan_points.append((px, py))
                    self._update_occupancy(px, py, is_obstacle=True)
                    hit_obstacle = True
                    line, = self.ax[0].plot([x, px], [y, py], 'c-', alpha=0.3, linewidth=0.5)
                    self.scan_lines.append(line)
                    break
                else:
                    self._update_occupancy(px, py, is_obstacle=False)
            if not hit_obstacle:
                end_x = x + self.lidar_range * dx
                end_y = y + self.lidar_range * dy
                line, = self.ax[0].plot([x, end_x], [y, end_y], 'c-', alpha=0.1, linewidth=0.3)
                self.scan_lines.append(line)
        return scan_points

    def _find_boundary_cross(self, x, y, dx, dy):
        """判断射线与迷宫边界的交点（世界坐标），返回(是否有交点, 交点x, 交点y)"""
        t_list = []
        # 与四条边的交点参数t
        if abs(dx) > 1e-6:
            t_left = (self.left_x - x) / dx
            t_right = (self.right_x - x) / dx
            if t_left > 0:
                t_list.append((t_left, self.left_x, y + t_left * dy))
            if t_right > 0:
                t_list.append((t_right, self.right_x, y + t_right * dy))
        if abs(dy) > 1e-6:
            t_bottom = (self.bottom_y - y) / dy
            t_top = (self.top_y - y) / dy
            if t_bottom > 0:
                t_list.append((t_bottom, x + t_bottom * dx, self.bottom_y))
            if t_top > 0:
                t_list.append((t_top, x + t_top * dx, self.top_y))
        if not t_list:
            return False, None, None
        t_min, cross_x, cross_y = min(t_list, key=lambda t: t[0])
        return True, cross_x, cross_y

    def _record_exit_point(self, x, y):
        sx, sy = self.start_point
        if math.hypot(x - sx, y - sy) <= 2.0:
            return
        for ex, ey in self.exit_points:
            if math.hypot(x - ex, y - ey) <= 2.0:
                return

        # 左/右边界：在该x处，扫描y方向的最大无障碍区间，取中点
        if abs(x - self.left_x) < 1e-3 or abs(x - self.right_x) < 1e-3:
            y_int = int(np.floor(y))
            # 扫描y方向，找最大连续无障碍区间
            y_candidates = []
            for dy in range(-10, 11):  # 假设通道不会超过20格
                y_test = y_int + dy
                if not (self.bottom_y <= y_test <= self.top_y):
                    continue
                px, py = self.world_to_pixel(x, y_test)
                if 0 <= px < self.map_size[1] and 0 <= py < self.map_size[0]:
                    if not self.obstacle_mask[py, px]:
                        y_candidates.append(y_test)
            # 找最大连续区间
            if y_candidates:
                y_candidates = sorted(y_candidates)
                max_len = 0
                best_start = best_end = y_candidates[0]
                start = end = y_candidates[0]
                for i in range(1, len(y_candidates)):
                    if y_candidates[i] == y_candidates[i-1] + 1:
                        end = y_candidates[i]
                    else:
                        if end - start + 1 > max_len:
                            max_len = end - start + 1
                            best_start, best_end = start, end
                        start = end = y_candidates[i]
                # 检查最后一段
                if end - start + 1 > max_len:
                    best_start, best_end = start, end
                y_center = (best_start + best_end) / 2
                self.exit_points.append((x, y_center))
            return

        # 上/下边界：在该y处，扫描x方向的最大无障碍区间，取中点
        if abs(y - self.top_y) < 1e-3 or abs(y - self.bottom_y) < 1e-3:
            x_int = int(np.floor(x))
            x_candidates = []
            for dx in range(-10, 11):
                x_test = x_int + dx
                if not (self.left_x <= x_test <= self.right_x):
                    continue
                px, py = self.world_to_pixel(x_test, y)
                if 0 <= px < self.map_size[1] and 0 <= py < self.map_size[0]:
                    if not self.obstacle_mask[py, px]:
                        x_candidates.append(x_test)
            if x_candidates:
                x_candidates = sorted(x_candidates)
                max_len = 0
                best_start = best_end = x_candidates[0]
                start = end = x_candidates[0]
                for i in range(1, len(x_candidates)):
                    if x_candidates[i] == x_candidates[i-1] + 1:
                        end = x_candidates[i]
                    else:
                        if end - start + 1 > max_len:
                            max_len = end - start + 1
                            best_start, best_end = start, end
                        start = end = x_candidates[i]
                if end - start + 1 > max_len:
                    best_start, best_end = start, end
                x_center = (best_start + best_end) / 2
                self.exit_points.append((x_center, y))
            return

    def _update_occupancy(self, x, y, is_obstacle):
        """概率化占据栅格更新"""
        px, py = self.world_to_pixel(x, y)
        if 0 <= px < self.map_size[1] and 0 <= py < self.map_size[0]:
            if is_obstacle:
                self.occupancy_map[py, px] = min(1.0, self.occupancy_map[py, px] + 0.5)
            else:
                self.occupancy_map[py, px] = max(0.0, self.occupancy_map[py, px] - 0.5)
                
    def detect_frontiers(self):
        """检测探索边界（已知与未知区域的边界）"""
        self.frontier_cells = deque()
        free_space = (self.occupancy_map < 0.5)
        unknown_space = (self.occupancy_map == 0.5)
        
        for i in range(1, self.map_size[0]-1):
            for j in range(1, self.map_size[1]-1):
                if free_space[i,j] and np.any(unknown_space[i-1:i+2, j-1:j+2]):
                    self.frontier_cells.append((i, j))
        
        # 按距离排序
        if self.frontier_cells:
            robot_px, robot_py = self.world_to_pixel(*self.robot_pose[:2])
            self.frontier_cells = deque(sorted(
                self.frontier_cells,
                key=lambda p: (p[0]-robot_py)**2 + (p[1]-robot_px)**2
            ))

    def detect_frontiers_with_clustering(self):
        """使用聚类技术检测和分组探索边界"""
        # 识别前沿点
        self.frontier_cells = []
        free_space = (self.occupancy_map < 0.5)
        unknown_space = (self.occupancy_map == 0.5)
        
        # 创建前沿候选掩码（自由空间与未知空间的边界）
        # 使用膨胀操作找出潜在的前沿区域
        dilated_unknown = binary_dilation(unknown_space, iterations=1)
        frontier_mask = free_space & dilated_unknown
        
        # 收集所有前沿点
        frontier_points = []
        for i in range(1, self.map_size[0]-1):
            for j in range(1, self.map_size[1]-1):
                if frontier_mask[i, j]:
                    # 验证该点确实是前沿（自由空间与未知空间的边界）
                    if free_space[i,j] and np.any(unknown_space[i-1:i+2, j-1:j+2]):
                        # 记录像素坐标和世界坐标
                        world_x = j * self.resolution
                        world_y = (self.map_size[0] - i) * self.resolution
                        
                        # 确保点在迷宫内部或边界上
                        if (self.left_x <= world_x <= self.right_x and 
                            self.bottom_y <= world_y <= self.top_y):
                            frontier_points.append((world_x, world_y, i, j))
        
        # 如果没有找到前沿点，则返回
        if len(frontier_points) < 5:
            self.frontier_clusters = []
            # 如果已经找到出口且没有更多前沿点，设置为已完成探索
            if self.exit_points and not self.all_frontiers_explored:
                self.all_frontiers_explored = True
            return
            
        # 提取世界坐标用于聚类
        if frontier_points:
            points_array = np.array([(p[0], p[1]) for p in frontier_points])
            
            # 使用DBSCAN进行聚类分析，eps是聚类的最大距离，min_samples是一个聚类至少需要的点数
            clustering = DBSCAN(eps=1.0, min_samples=3).fit(points_array)
            labels = clustering.labels_
            
            # 按聚类整理前沿点
            clusters = {}
            for i, point in enumerate(frontier_points):
                label = labels[i]
                if label != -1:  # -1表示噪声点
                    if label not in clusters:
                        clusters[label] = []
                    clusters[label].append(point)
            
            # 计算每个聚类的中心点和大小
            self.frontier_clusters = []
            for label, points in clusters.items():
                if len(points) < 5:  # 忽略太小的聚类
                    continue
                    
                # 计算中心点
                center_x = np.mean([p[0] for p in points])
                center_y = np.mean([p[1] for p in points])
                
                # 确保中心点在迷宫范围内
                center_x = max(self.left_x, min(center_x, self.right_x))
                center_y = max(self.bottom_y, min(center_y, self.top_y))
                
                # 计算聚类大小（点的数量）
                size = len(points)
                
                # 添加到聚类列表
                self.frontier_clusters.append({
                    'center': (center_x, center_y),
                    'size': size,
                    'points': points,
                    'visited': False  # 标记该聚类是否已被访问
                })
                
            # 计算到当前位置的距离
            robot_x, robot_y, _ = self.robot_pose
            for cluster in self.frontier_clusters:
                center_x, center_y = cluster['center']
                dist = math.sqrt((center_x - robot_x)**2 + (center_y - robot_y)**2)
                cluster['distance'] = dist
                
            # 按照聚类大小和距离的加权和排序（优先考虑大的聚类，但也考虑距离）
            self.frontier_clusters.sort(
                key=lambda c: c['size'] * 0.7 - c['distance'] * 0.3,
                reverse=True
            )
            
            # 为可视化准备前沿点列表
            self.frontier_cells = []
            for cluster in self.frontier_clusters:
                for _, _, i, j in cluster['points']:
                    self.frontier_cells.append((i, j))

    def select_target_original(self, frame=0):
        """
        原始的目标选择方法，作为备选策略。
        目标位置选择为雷达能扫描到的最远的地方，且下次选择的目标位置不在雷达探索过的区域。
        """
        x, y, theta = self.robot_pose
        candidate_points = []
        candidate_keys = []
        distances = []

        for angle_deg in range(0, 360, self.lidar_resolution):
            angle_rad = math.radians(angle_deg) + theta
            dx, dy = math.cos(angle_rad), math.sin(angle_rad)
            for distance in np.arange(0.1, self.lidar_range, 0.05):
                px = x + distance * dx
                py = y + distance * dy
                pix_x, pix_y = self.world_to_pixel(px, py)
                # target_key = (int(round(px / self.resolution)), int(round(py / self.resolution)))
                if not (0 <= pix_x < self.map_size[1] and 0 <= pix_y < self.map_size[0]):
                    break
                if self.obstacle_mask[pix_y, pix_x]:
                    # 选择障碍物前的最后一个点
                    last_px = x + (distance - 0.05) * dx
                    last_py = y + (distance - 0.05) * dy
                    last_key = (int(round(last_px / self.resolution)), int(round(last_py / self.resolution)))
                    if last_key not in self.visited_targets:
                        candidate_points.append((last_px, last_py))
                        candidate_keys.append(last_key)
                        distances.append(distance - 0.05)
                    break
            else:
                # 这是无障碍物阻挡的情况，激光能直达
                px = x + self.lidar_range * dx
                py = y + self.lidar_range * dy
                key = (int(round(px / self.resolution)), int(round(py / self.resolution)))
                if key not in self.visited_targets:
                    # 这些点一定是激光可达的
                    candidate_points.append((px, py))
                    candidate_keys.append(key)
                    distances.append(self.lidar_range)

        if not candidate_points:
            self.current_target = None
            return False

        # 选择距离最远的点
        idx = int(np.argmax(distances))
        self.current_target = candidate_points[idx]
        self.visited_targets.add(candidate_keys[idx])
        return True

    def select_frontier_target(self):
        """选择最佳的前沿聚落作为探索目标，并在到达出口后将目标设为起点"""
        # 如果已经完成前沿探索且有出口，准备返回出口
        if self.all_frontiers_explored and self.exit_points and self.exploration_state == "exploring":
            self.exploration_state = "returning_to_exit"
            if not self.best_exit:
                # 选择距离当前位置最近的出口
                robot_x, robot_y, _ = self.robot_pose
                closest_exit = min(self.exit_points, 
                                  key=lambda e: math.hypot(e[0]-robot_x, e[1]-robot_y))
                self.best_exit = closest_exit
            self.current_target = self.best_exit
            print(f"所有前沿点已探索完毕，前往出口: {self.best_exit}")
            return True
        
        # 如果正在返回出口
        if self.exploration_state == "returning_to_exit" and self.best_exit:
            # 检查是否已经到达出口
            robot_x, robot_y, _ = self.robot_pose
            ex, ey = self.best_exit
            if math.hypot(robot_x - ex, robot_y - ey) < 0.5 and not self.returned_to_exit:
                # 到达出口，准备回到起点
                self.returned_to_exit = True
                self.current_target = tuple(self.start_point)
                print("已到达出口，目标切换为起点")
                return True
            elif self.returned_to_exit:
                # 正在返回起点
                self.current_target = tuple(self.start_point)
                return True
            else:
                # 还未到达出口，继续前往出口
                self.current_target = self.best_exit
                return True

        # 标准前沿探索逻辑
        if not hasattr(self, 'frontier_clusters') or not self.frontier_clusters:
            # 如果没有更多前沿点且有出口，标记为探索完成
            if self.exit_points:
                self.all_frontiers_explored = True
            return False
            
        # 检查是否已有目标，并且还在有效探索范围内
        if self.current_target:
            target_x, target_y = self.current_target
            robot_x, robot_y, _ = self.robot_pose
            dist_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
            
            # 如果离目标很近或者目标区域已经探索完毕（占据栅格已更新），则选择新目标
            target_px, target_py = self.world_to_pixel(target_x, target_y)
            is_explored = (0 <= target_px < self.map_size[1] and 
                           0 <= target_py < self.map_size[0] and
                           self.occupancy_map[target_py, target_px] != 0.5)
            
            if dist_to_target < 1.0 or is_explored:
                # 目标已达到或已探索，需要新目标
                # 找到当前目标对应的聚类并标记为已访问
                for cluster in self.frontier_clusters:
                    center_x, center_y = cluster['center']
                    if math.hypot(center_x - target_x, center_y - target_y) < 1.0:
                        cluster['visited'] = True
                        break
            else:
                # 继续前往当前目标
                return True
                
        # 首先检查是否有未访问的聚类
        unvisited_clusters = [c for c in self.frontier_clusters if not c['visited']]
        if not unvisited_clusters:
            self.all_frontiers_explored = True
            # 如果所有聚类已访问且有出口，开始返回出口
            if self.exit_points:
                self.exploration_state = "returning_to_exit"
                if not self.best_exit:
                    # 选择距离当前位置最近的出口
                    robot_x, robot_y, _ = self.robot_pose
                    closest_exit = min(self.exit_points, 
                                      key=lambda e: math.hypot(e[0]-robot_x, e[1]-robot_y))
                    self.best_exit = closest_exit
                self.current_target = self.best_exit
                print(f"所有前沿点已探索完毕，前往出口: {self.best_exit}")
                return True
            return False
            
        # 选择最佳聚落
        for cluster in unvisited_clusters:
            center_x, center_y = cluster['center']
            
            # 检查该中心点是否有效
            px, py = self.world_to_pixel(center_x, center_y)
            if not (0 <= px < self.map_size[1] and 0 <= py < self.map_size[0]):
                continue

            # 不允许目标为起点（除非特殊回起点流程）
            if not self.returned_to_exit and math.hypot(center_x - self.start_point[0], center_y - self.start_point[1]) < 1.0:
                continue
                
            # 检查该点是否仍是未知区域（可能在选择过程中已被探索）
            if self.occupancy_map[py, px] == 0.5:
                # 检查附近是否有障碍物
                is_safe = True
                for i in range(-5, 6):
                    for j in range(-5, 6):
                        check_px, check_py = px + i, py + j
                        if (0 <= check_px < self.map_size[1] and 
                            0 <= check_py < self.map_size[0] and
                            self.obstacle_mask[check_py, check_px]):
                            is_safe = False
                            break
                    if not is_safe:
                        break
                
                if is_safe:
                    # 确保目标点在激光可达范围内（不被墙阻挡）
                    self.current_target = (center_x, center_y)
                    # 记录已访问过的目标
                    target_key = (int(center_x / self.resolution), int(center_y / self.resolution))
                    self.visited_targets.add(target_key)
                    return True
                    
        # 没有找到合适的聚落中心，尝试找单个的前沿点
        if self.frontier_cells:
            # 按距离排序选取最近的
            robot_px, robot_py = self.world_to_pixel(*self.robot_pose[:2])
            sorted_cells = sorted(
                self.frontier_cells,
                key=lambda p: (p[0]-robot_py)**2 + (p[1]-robot_px)**2
            )
            
            for i, j in sorted_cells:
                # 转换为世界坐标
                world_x = j * self.resolution
                world_y = (self.map_size[0] - i) * self.resolution
                
                # 确保点在迷宫内部或边界上
                if not (self.left_x <= world_x <= self.right_x and 
                        self.bottom_y <= world_y <= self.top_y):
                    continue

                # 不允许目标为起点（除非特殊回起点流程）
                if not self.returned_to_exit and math.hypot(world_x - self.start_point[0], world_y - self.start_point[1]) < 1.0:
                    continue
                
                # 检查该点附近是否安全
                is_safe = True
                for di in range(-3, 4):
                    for dj in range(-3, 4):
                        check_i, check_j = i + di, j + dj
                        if (0 <= check_i < self.map_size[0] and 
                            0 <= check_j < self.map_size[1] and
                            self.obstacle_mask[check_i, check_j]):
                            is_safe = False
                            break
                    if not is_safe:
                        break
                
                if is_safe:
                    # 确保目标点在激光可达范围内（不被墙阻挡）
                    self.current_target = (world_x, world_y)
                    target_key = (int(world_x / self.resolution), int(world_y / self.resolution))
                    self.visited_targets.add(target_key)
                    return True
            
        # 如果没有找到任何有效的前沿点，但有出口，开始返回出口
        if self.exit_points:
            self.all_frontiers_explored = True
            self.exploration_state = "returning_to_exit"
            if not self.best_exit:
                # 选择距离当前位置最近的出口
                robot_x, robot_y, _ = self.robot_pose
                closest_exit = min(self.exit_points, 
                                  key=lambda e: math.hypot(e[0]-robot_x, e[1]-robot_y))
                self.best_exit = closest_exit
            self.current_target = self.best_exit
            print(f"无有效前沿点，前往出口: {self.best_exit}")
            return True
            
        # 如果没有合适目标，则回退到原来的选择方法
        return False

    def calculate_map_completeness(self):
        """
        计算地图的完整度，用于评估探索进度
        """
        # 计算探索率：已知区域占整个地图的百分比
        known_cells = np.sum(self.occupancy_map != 0.5)
        total_cells = self.map_size[0] * self.map_size[1]
        exploration_rate = known_cells / total_cells * 100
        
        # 计算地图中的自由空间和障碍物比例
        free_space = np.sum(self.occupancy_map < 0.5)
        obstacles = np.sum(self.occupancy_map > 0.5)
        
        # 自由空间率和障碍物率
        free_rate = free_space / total_cells * 100
        obstacle_rate = obstacles / total_cells * 100
        
        return {
            'exploration_rate': exploration_rate,
            'free_rate': free_rate,
            'obstacle_rate': obstacle_rate,
            'unknown_rate': 100 - exploration_rate
        }

    def sliding_window_obstacle_avoidance(self, x, y, theta, goal_x, goal_y):
        """
        使用滑动窗口方法进行局部避障
        
        Args:
            x, y, theta: 当前机器人位置和朝向
            goal_x, goal_y: 目标点位置
        
        Returns:
            best_heading: 最佳前进方向，如果无法找到有效方向则返回None
        """
        # 滑动窗口参数
        window_size = 180  # 搜索窗口大小（度）
        resolution = 5     # 角度分辨率（度）
        max_depth = 1.0    # 最大检测深度
        depth_step = 0.05  # 深度步长
        
        # 目标方向（理想方向）
        goal_theta = math.atan2(goal_y - y, goal_x - x)
        
        # 归一化角度到[-pi, pi]
        goal_theta = (goal_theta - theta + math.pi) % (2 * math.pi) - math.pi + theta
        
        # 生成候选方向（以目标方向为中心的滑动窗口）
        half_window = window_size // 2
        directions = []
        for angle_offset in range(-half_window, half_window+1, resolution):
            angle = theta + math.radians(angle_offset)
            # 归一化角度
            angle = angle % (2 * math.pi)
            directions.append(angle)
        
        # 对每个方向进行评估
        best_direction = None
        best_score = float('-inf')
        
        for direction in directions:
            dx = math.cos(direction)
            dy = math.sin(direction)
            
            # 计算沿该方向的安全距离
            safe_distance = 0
            for depth in np.arange(0.1, max_depth, depth_step):
                test_x = x + depth * dx
                test_y = y + depth * dy
                px, py = self.world_to_pixel(test_x, test_y)
                
                if not (0 <= px < self.map_size[1] and 0 <= py < self.map_size[0]):
                    break
                    
                if self.obstacle_mask[py, px]:
                    break
                    
                safe_distance = depth
            
            if safe_distance < 0.7:  # 方向不安全
                continue
                
            # 计算与目标方向的角度差
            angle_diff = abs((direction - goal_theta + math.pi) % (2 * math.pi) - math.pi)
            
            # 评分函数：安全距离 - 角度偏差惩罚
            # 偏向于选择接近目标方向且安全距离较长的方向
            angle_weight = 0.7  # 角度权重
            score = safe_distance - angle_weight * angle_diff
            
            if score > best_score:
                best_score = score
                best_direction = direction
        
        # 添加一些随机性，避免陷入局部最小值
        if best_direction is not None and random.random() < 0.1:
            best_direction += math.radians(random.uniform(-10, 10))
            best_direction = best_direction % (2 * math.pi)
        
        return best_direction

    def a_star_path_planning(self, start_x, start_y, goal_x, goal_y):
        """使用A*算法规划从起点到终点的路径，确保路径在迷宫内部
        
        Args:
            start_x, start_y: 起点坐标（世界坐标）
            goal_x, goal_y: 终点坐标（世界坐标）
        
        Returns:
            path: 路径点列表，每个点为(x, y)世界坐标，如果无法到达则返回None
        """
        # 确保起点和终点在迷宫边界内
        start_x = max(self.left_x, min(start_x, self.right_x))
        start_y = max(self.bottom_y, min(start_y, self.top_y))
        goal_x = max(self.left_x, min(goal_x, self.right_x))
        goal_y = max(self.bottom_y, min(goal_y, self.top_y))
        
        # 转换为像素坐标
        start_px, start_py = self.world_to_pixel(start_x, start_y)
        goal_px, goal_py = self.world_to_pixel(goal_x, goal_y)
        
        # 检查起点和终点是否在地图范围内且不是障碍物
        if not (0 <= start_px <= self.map_size[1] and 0 <= start_py <= self.map_size[0]):
            return None
        if not (0 <= goal_px <= self.map_size[1] and 0 <= goal_py <= self.map_size[0]):
            return None
        if self.obstacle_mask[start_py, start_px] or self.obstacle_mask[goal_py, goal_px]:
            return None
        
        # 定义节点类
        class Node:
            def __init__(self, x, y, cost, parent=None):
                self.x = x
                self.y = y
                self.cost = cost  # g(n): 从起点到当前节点的成本
                self.heuristic = 0  # h(n): 启发式函数（从当前节点到目标的估计成本）
                self.total_cost = 0  # f(n) = g(n) + h(n)
                self.parent = parent
            
            def __lt__(self, other):
                return self.total_cost < other.total_cost
        
        # 定义启发式函数（曼哈顿距离）
        def heuristic(x, y):
            return abs(x - goal_px) + abs(y - goal_py)
        
        # 定义相邻节点的移动方向（8个方向）
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),  # 上下左右
            (-1, -1), (-1, 1), (1, -1), (1, 1)  # 对角线
        ]
        
        # 初始化开放列表和关闭列表
        open_list = []
        closed_set = set()
        
        # 创建起点节点并加入开放列表
        start_node = Node(start_px, start_py, 0)
        start_node.heuristic = heuristic(start_px, start_py)
        start_node.total_cost = start_node.cost + start_node.heuristic
        heapq.heappush(open_list, (start_node.total_cost, start_node))
        
        # 开始A*算法主循环
        while open_list:
            # 获取总成本最小的节点
            _, current = heapq.heappop(open_list)
            
            # 如果到达目标，回溯路径
            if current.x == goal_px and current.y == goal_py:
                path = []
                while current:
                    # 将像素坐标转回世界坐标
                    world_x = current.x * self.resolution
                    world_y = (self.map_size[0] - current.y) * self.resolution
                    
                    # 确保路径点在迷宫边界内
                    world_x = max(self.left_x, min(world_x, self.right_x))
                    world_y = max(self.bottom_y, min(world_y, self.top_y))
                    
                    path.append((world_x, world_y))
                    current = current.parent
                return path[::-1]  # 反转路径从起点到终点
            
            # 将当前节点加入关闭列表
            closed_set.add((current.x, current.y))
            
            # 探索相邻节点
            for dx, dy in directions:
                nx, ny = current.x + dx, current.y + dy
                
                # 检查是否在地图范围内
                if not (0 <= nx <= self.map_size[1] and 0 <= ny <= self.map_size[0]):
                    continue
                
                # 将像素坐标转换为世界坐标，检查是否在迷宫边界内
                world_x, world_y = self.pixel_to_world(nx, ny)
                if not (self.left_x <= world_x <= self.right_x and 
                        self.bottom_y <= world_y <= self.top_y):
                    continue
                
                # 检查是否是障碍物或已经在关闭列表中
                if self.obstacle_mask[ny, nx] or (nx, ny) in closed_set:
                    continue
                
                # 计算移动成本（对角线移动成本为1.414，直线移动成本为1）
                move_cost = 1.414 if dx != 0 and dy != 0 else 1
                
                # 创建邻居节点
                neighbor = Node(nx, ny, current.cost + move_cost, current)
                neighbor.heuristic = heuristic(nx, ny)
                neighbor.total_cost = neighbor.cost + neighbor.heuristic
                
                # 检查是否需要加入开放列表
                should_add = True
                for i, (_, node) in enumerate(open_list):
                    if node.x == nx and node.y == ny:
                        # 如果邻居已经在开放列表中，检查是否有更低的成本
                        if neighbor.cost < node.cost:
                            # 更新成本
                            node.cost = neighbor.cost
                            node.total_cost = node.cost + node.heuristic
                            node.parent = current
                            # 重新排序开放列表
                            heapq.heapify(open_list)
                        should_add = False
                        break
                
                if should_add:
                    heapq.heappush(open_list, (neighbor.total_cost, neighbor))
        
        # 如果开放列表为空且没有找到路径，则返回None
        return None

    def navigate_to_target(self):
        """使用A*算法规划路径，导航到目标点"""
        if not self.current_target:
            return False

        x, y, theta = self.robot_pose
        target_x, target_y = self.current_target

        # 计算到目标的距离
        distance = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
        
        # 如果已经到达目标附近
        if distance < 0.05:
            self.current_target = None
            return False
            
        # 是否需要重新规划路径
        if not hasattr(self, 'path') or not self.path or len(self.path) < 2:
            # 使用A*算法规划路径
            self.path = self.a_star_path_planning(x, y, target_x, target_y)
            
            # 如果找不到路径
            if not self.path:
                print("无法找到到目标的路径，寻找新目标...")
                self.current_target = None
                return False
        
        # 移除已经到达的路径点
        while len(self.path) >= 2:
            next_x, next_y = self.path[1]
            if math.sqrt((next_x - x)**2 + (next_y - y)**2) < 0.2:
                self.path.pop(0)  # 移除已到达的点
            else:
                break
                
        # 如果路径只剩最后一个点（目标点）
        if len(self.path) <= 1:
            # 如果当前已经很接近目标
            if distance < 1.0:
                self.current_target = None
                return False
                
            # 否则重新规划到目标的路径
            self.path = self.a_star_path_planning(x, y, target_x, target_y)
            if not self.path:
                self.current_target = None
                return False
        
        # 设置下一个导航点
        next_x, next_y = self.path[1]
        
        # 计算朝向下一个点的角度
        desired_theta = math.atan2(next_y - y, next_x - x)
        angle_diff = (desired_theta - theta + math.pi) % (2 * math.pi) - math.pi

        # 如果需要转向，优先转向
        if abs(angle_diff) > math.radians(15):
            return self._move_robot(0, angle_diff * 0.5)
        else:
            # 直线前进到下一个点
            step_distance = min(0.25, math.sqrt((next_x - x)**2 + (next_y - y)**2))
            success, blocked_info = self._move_robot(step_distance, return_blocked_info=True)
            
            if not success:
                # 如果被阻挡，使用滑动窗口方法进行避障
                if blocked_info is not None:
                    block_x, block_y, move_theta = blocked_info
                    
                    # 使用滑动窗口避障算法
                    new_heading = self.sliding_window_obstacle_avoidance(x, y, theta, next_x, next_y)
                    
                    if new_heading is not None:
                        # 使用新的航向角前进
                        step_distance = 0.5  # 短距离前进，避免碰撞
                        new_x = x + step_distance * math.cos(new_heading)
                        new_y = y + step_distance * math.sin(new_heading)
                        
                        # 检查新位置是否有效
                        px, py = self.world_to_pixel(new_x, new_y)
                        if (0 <= px < self.map_size[1] and 0 <= py < self.map_size[0] and 
                            not self.obstacle_mask[py, px]):
                            # 更新机器人位置和朝向
                            self.robot_pose = (new_x, new_y, new_heading)
                            self.robot_path.append((new_x, new_y))
                            self._update_occupancy(new_x, new_y, is_obstacle=False)
                            
                            # 在遇到障碍物后重新规划路径
                            self.path = self.a_star_path_planning(new_x, new_y, target_x, target_y)
                            return True
                
                # 如果滑动窗口避障失败，尝试重新规划A*路径
                self.path = self.a_star_path_planning(x, y, target_x, target_y)
                if not self.path:
                    self.current_target = None
                    return False
            
            return success

    def _move_robot(self, distance, theta_change=0, return_blocked_info=False):
        """安全的移动控制，遇障碍物返回碰撞点和方向"""
        x, y, theta = self.robot_pose
        new_theta = theta + theta_change
        new_x = x + distance * math.cos(new_theta)
        new_y = y + distance * math.sin(new_theta)

        # 碰撞检测（检查路径上的多个点）
        distance_total = math.hypot(new_x - x, new_y - y)
        steps = max(1, int(distance_total / self.resolution))
        for i in range(steps + 1):
            check_x = x + (new_x - x) * i / steps
            check_y = y + (new_y - y) * i / steps
            px, py = self.world_to_pixel(check_x, check_y)
            if not (0 <= px < self.map_size[1] and 0 <= py < self.map_size[0]):
                break
            if self.obstacle_mask[py, px]:
                if return_blocked_info:
                    # 返回碰撞点和当前移动方向
                    return False, (check_x, check_y, new_theta)
                else:
                    return False
        # 没有碰到障碍物，正常前进
        self.robot_pose = (new_x, new_y, new_theta)
        self.robot_path.append((new_x, new_y))
        self._update_occupancy(new_x, new_y, is_obstacle=False)
        if return_blocked_info:
            return True, None
        else:
            return True

    def update_visualization(self, frame):
        if not self.current_target:
            self.detect_frontiers_with_clustering()
            self.select_frontier_target()
        moved = self.navigate_to_target()
        if not moved:
            for angle in random.sample([30, 90, 120], 3):
                rad = math.radians(angle) * random.choice([1, -1])
                moved_try, _ = self._move_robot(0.3, rad, return_blocked_info=True)
                if moved_try:
                    break

        scan = self.simulate_lidar()
        x, y, theta = self.robot_pose
        arrow_length = 0.8
        dx = arrow_length * math.cos(theta)
        dy = arrow_length * math.sin(theta)
        self.robot_arrow.set_positions((x, y), (x + dx, y + dy))
        path_x, path_y = zip(*self.robot_path)
        self.path_line.set_data(path_x, path_y)
        if scan:
            scan_x, scan_y = zip(*scan)
            self.scan_points.set_offsets(np.c_[scan_x, scan_y])
        else:
            self.scan_points.set_offsets(np.empty((0, 2)))
        if self.current_target:
            self.target_icon.set_offsets([self.current_target])
        else:
            self.target_icon.set_offsets(np.empty((0, 2)))
        if self.frontier_cells:
            frontier_world_points = []
            for i, j in self.frontier_cells:
                world_x = j * self.resolution
                world_y = (self.map_size[0] - i) * self.resolution
                frontier_world_points.append((world_x, world_y))
            if frontier_world_points:
                frontier_x, frontier_y = zip(*frontier_world_points)
                self.frontier_points.set_offsets(np.c_[frontier_x, frontier_y])
            else:
                self.frontier_points.set_offsets(np.empty((0, 2)))
        else:
            self.frontier_points.set_offsets(np.empty((0, 2)))
        if hasattr(self, 'frontier_clusters') and self.frontier_clusters:
            cluster_centers = []
            # 区分已访问和未访问的聚类中心
            for cluster in self.frontier_clusters:
                if not cluster['visited']:  # 只显示未访问的聚类中心
                    cluster_centers.append(cluster['center'])
            if cluster_centers:
                center_x, center_y = zip(*cluster_centers)
                self.cluster_centers.set_offsets(np.c_[center_x, center_y])
            else:
                self.cluster_centers.set_offsets(np.empty((0, 2)))
        else:
            self.cluster_centers.set_offsets(np.empty((0, 2)))
        if hasattr(self, 'path') and self.path and len(self.path) > 1:
            path_x, path_y = zip(*self.path)
            self.path_points.set_offsets(np.c_[path_x, path_y])
        else:
            self.path_points.set_offsets(np.empty((0, 2)))
        self.slam_img.set_data(self.occupancy_map)
        explored_overlay = np.zeros(self.map_size)
        explored_overlay[self.occupancy_map != 0.5] = 1
        self.explored_img.set_data(explored_overlay)
        completeness = self.calculate_map_completeness()
        
        # 状态信息展示
        status_text = f"Explored: {completeness['exploration_rate']:.1f}%\n" \
                    f"Free: {completeness['free_rate']:.1f}%\n" \
                    f"Obstacle: {completeness['obstacle_rate']:.1f}%\n" \
                    f"Status: {self.exploration_state}\n" \
                    f"Exits found: {len(self.exit_points)}"
        self.completeness_text.set_text(status_text)
        
        # 出口点可视化（右图，绿色EXIT文本）
        # 安全地移除之前的文本对象
        if hasattr(self, 'exit_path_line') and hasattr(self.exit_path_line, 'remove'):
            try:
                self.exit_path_line.remove()
            except ValueError:
                pass  # 如果已经被移除，忽略错误
            
        # 清除之前所有的出口文本
        if hasattr(self, 'exit_texts'):
            for txt in list(self.exit_texts):  # 创建列表副本进行迭代
                try:
                    if hasattr(txt, 'remove'):
                        txt.remove()
                except ValueError:
                    pass  # 如果已经被移除，忽略错误
        
        self.exit_texts = []
        
        # 绘制新的出口标记
        for ex, ey in self.exit_points:
            # 高亮显示选中的最佳出口
            is_best = self.best_exit and (ex, ey) == self.best_exit
            color = 'red' if is_best else 'green'
            fontsize = 16 if is_best else 14
            txt = self.ax[1].text(
                ex, ey, 'EXIT', color=color, fontsize=fontsize, fontweight='bold',
                ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=color, lw=2, alpha=0.7), zorder=20
            )
            self.exit_texts.append(txt)
            
            # 如果机器人正在返回出口，显示到出口的路径
            if self.exploration_state == "returning_to_exit" and is_best:
                # 绘制新的路径线
                if hasattr(self, 'path') and self.path and len(self.path) > 1:
                    self.exit_path_line, = self.ax[1].plot(
                        [p[0] for p in self.path], [p[1] for p in self.path], 
                        'r--', linewidth=2, alpha=0.8
                    )
                    self.exit_texts.append(self.exit_path_line)  # 只添加一次

        # 检查是否已完成“回到入口”任务，若完成则关闭窗口终止仿真
        if self.returned_to_exit:
            x, y, _ = self.robot_pose
            sx, sy = self.start_point
            if math.hypot(x - sx, y - sy) < 0.5:
                print("已从出口返回入口，探索任务完成，程序即将终止。")
                plt.close(self.fig)

        return [self.robot_arrow, self.path_line, self.scan_points, self.frontier_points,
                self.cluster_centers, self.target_icon, self.completeness_text, self.path_points,
                self.slam_img, self.explored_img] + self.scan_lines + self.exit_texts
    # 其余方法如a_star_path_planning、navigate_to_target等保持原有实现

    def run_simulation(self):
        """运行动画"""
        ani = FuncAnimation(
            self.fig,
            self.update_visualization,
            frames=600,  # 增加帧数以确保有足够时间完成探索
            interval=100,
            blit=True,
            cache_frame_data=False
        )
        plt.tight_layout()
        plt.show()
        
        # 打印结果统计信息
        print(f"Exploration Completed!")
        print(f"Number of Exits Found: {len(self.exit_points)}")
        print(f"Selected Best Exit: {self.best_exit}")
        print(f"Exploration Path Length: {len(self.robot_path)}")

        # 计算地图完整度
        completeness = self.calculate_map_completeness()
        print(f"Map Exploration Rate: {completeness['exploration_rate']:.2f}%")
        print(f"Free Space Rate: {completeness['free_rate']:.2f}%")
        print(f"Obstacle Rate: {completeness['obstacle_rate']:.2f}%")


if __name__ == "__main__":
    # 使用绝对路径确保json加载
    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(script_dir, "1.json")
    slam = ImprovedMazeSLAM(json_path)
    slam.run_simulation()