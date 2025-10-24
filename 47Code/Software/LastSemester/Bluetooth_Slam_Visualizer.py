import serial
import matplotlib.pyplot as plt
import numpy as np
import re
import threading
import time
from collections import deque
import matplotlib.patches as patches
import json

# 设置中文字体 - 使用Windows更常见的字体
plt.rcParams["font.family"] = ["SimHei", "Microsoft YaHei", "Arial Unicode MS", "sans-serif"]
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

class BluetoothSLAMVisualizer:
    def __init__(self, port='COM3', baudrate=115200, demo_mode=False):
        # 初始化串口
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.demo_mode = demo_mode  # 演示模式，即使没有实际数据也能显示动画效果
        
        # 初始化数据存储
        self.current_position = (0, 0)  # (x, y)
        self.current_angle = 0.0  # 弧度制
        self.map_points = set()  # 用于存储地图点
        self.scan_data = deque(maxlen=50)  # 存储最近的扫描数据
        self.imu_data = {'yaw': 0.0, 'gx': 0, 'gy': 0, 'gz': 0}
        self.obstacle_detected = False
        
        # 演示模式的计数器
        self.demo_counter = 0
        
        # 初始化可视化
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        self.running = True
        
        # 数据接收线程
        self.data_thread = threading.Thread(target=self.receive_data)
        self.data_thread.daemon = True
        
        # 初始化迷宫地图
        self.maze_map = []
        self.load_maze()
        
    def connect_serial(self):
        """连接到串口"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"已连接到串口 {self.port}")
            return True
        except Exception as e:
            print(f"无法连接到串口 {self.port}: {e}")
            return False
    
    def start(self):
        """启动可视化程序"""
        # 在演示模式下，不需要连接串口
        if not self.demo_mode and not self.connect_serial():
            print("无法连接到串口，是否要以演示模式启动？")
            self.demo_mode = True
        
        if not self.demo_mode:
            self.data_thread.start()
        
        # 开始可视化循环
        try:
            while self.running:
                if self.demo_mode:
                    self.update_demo_data()
                self.update_visualization()
                plt.pause(0.1)  # 短暂暂停以允许GUI更新
        except KeyboardInterrupt:
            print("程序被用户中断")
        finally:
            self.stop()
    
    def stop(self):
        """停止程序"""
        self.running = False
        if self.data_thread.is_alive():
            self.data_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("程序已停止")
        
    def load_maze(self):
        """从1.json加载迷宫地图数据"""
        try:
            # 加载1.json文件
            with open('1.json', 'r', encoding='utf-8') as f:
                maze_data = json.load(f)
                
            self.maze_map = []
            # 存储迷宫的起点
            if 'start_point' in maze_data:
                self.start_point = tuple(maze_data['start_point'])
            else:
                self.start_point = (1, 0)  # 默认起点
            
            # 转换线段到迷宫地图
            if 'segments' in maze_data:
                for segment in maze_data['segments']:
                    start_x, start_y = segment['start']
                    end_x, end_y = segment['end']
                    # 将坐标转换为适合显示的比例
                    scale = 0.2  # 缩放因子，使迷宫适合显示窗口
                    offset_x = -7  # X轴偏移，使迷宫居中
                    offset_y = -7  # Y轴偏移，使迷宫居中
                    
                    # 转换坐标
                    x1 = (start_x + offset_x) * scale
                    y1 = (start_y + offset_y) * scale
                    x2 = (end_x + offset_x) * scale
                    y2 = (end_y + offset_y) * scale
                    
                    self.maze_map.append((x1, y1, x2, y2))
        except Exception as e:
            print(f"加载迷宫文件失败: {e}")
            # 如果加载失败，使用默认迷宫
            self.maze_map = []
            self.start_point = (0, 0)
    
    def on_close(self, event):
        """处理窗口关闭事件"""
        self.running = False
    
    def receive_data(self):
        """接收并解析串口数据"""
        line_buffer = ""
        
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    # 读取所有可用数据
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    line_buffer += data
                    
                    # 处理每一行
                    while '\n' in line_buffer:
                        line, line_buffer = line_buffer.split('\n', 1)
                        self.parse_data(line.strip())
            except Exception as e:
                print(f"数据接收错误: {e}")
            
            time.sleep(0.01)  # 短暂延迟以减少CPU使用率
    
    def parse_data(self, line):
        """解析接收到的数据行"""
        if not line:
            return
        
        # 打印接收到的原始数据（用于调试）
        print(f"接收到: {line}")
        
        # 解析IMU和电机控制数据
        imu_match = re.search(r'Yaw:\s*([-+]?\d+\.\d+)\s+Gx:\s*(\d+)\s+Gy:\s*(\d+)\s+Gz:\s*(\d+)', line)
        if imu_match:
            yaw, gx, gy, gz = imu_match.groups()
            self.imu_data = {
                'yaw': float(yaw),
                'gx': int(gx),
                'gy': int(gy),
                'gz': int(gz)
            }
            # 更新当前角度（转换为弧度制）
            self.current_angle = float(yaw) * np.pi / 180.0
        
        # 解析障碍物数据
        obstacle_match = re.search(r'Obstacle\s+at\s+([-+]?\d+\.\d+)\s+deg\s+([-+]?\d+\.\d+)\s+mm', line)
        if obstacle_match:
            angle_deg, distance_mm = obstacle_match.groups()
            angle_rad = float(angle_deg) * np.pi / 180.0
            distance_m = float(distance_mm) / 1000.0  # 转换为米
            
            # 计算障碍物的世界坐标
            obstacle_x = self.current_position[0] + distance_m * np.cos(self.current_angle + angle_rad)
            obstacle_y = self.current_position[1] + distance_m * np.sin(self.current_angle + angle_rad)
            
            # 添加到地图点
            self.map_points.add((round(obstacle_x, 3), round(obstacle_y, 3)))
            self.obstacle_detected = True
        
        # 解析雷达扫描数据
        scan_match = re.search(r'Angle:\s*([-+]?\d+\.\d+)\s+deg\s+Dist:\s*([-+]?\d+\.\d+)\s+mm', line)
        if scan_match:
            angle_deg, distance_mm = scan_match.groups()
            angle_rad = float(angle_deg) * np.pi / 180.0
            distance_m = float(distance_mm) / 1000.0  # 转换为米
            
            # 存储扫描数据
            self.scan_data.append((angle_rad, distance_m))
            
            # 如果距离有效且不是特别远，则添加到地图
            if 0.05 < distance_m < 3.0:  # 5cm到3m的有效距离
                # 计算障碍物的世界坐标
                obstacle_x = self.current_position[0] + distance_m * np.cos(self.current_angle + angle_rad)
                obstacle_y = self.current_position[1] + distance_m * np.sin(self.current_angle + angle_rad)
                
                # 添加到地图点
                self.map_points.add((round(obstacle_x, 3), round(obstacle_y, 3)))
        
        # 解析路径清除信息
        if 'Path cleared' in line:
            self.obstacle_detected = False
    
    def update_demo_data(self):
        """更新演示模式下的数据 - 在迷宫中移动"""
        self.demo_counter += 1
        
        # 初始化时，将小车位置设置为迷宫起点
        if self.demo_counter == 1 and hasattr(self, 'start_point'):
            scale = 0.2
            offset_x = -7
            offset_y = -7
            start_x = (self.start_point[0] + offset_x) * scale
            start_y = (self.start_point[1] + offset_y) * scale
            self.current_position = (start_x, start_y)
            self.current_angle = 0.0
        
        # 在迷宫中的移动策略：随机游走，但避开墙壁
        speed = 0.01
        
        # 每30帧随机改变一次方向
        if self.demo_counter % 30 == 0:
            # 随机调整角度（-15度到+15度）
            angle_change = np.random.uniform(-15, 15) * np.pi / 180
            self.current_angle = (self.current_angle + angle_change) % (2 * np.pi)
        
        # 计算新位置
        dx = speed * np.cos(self.current_angle)
        dy = speed * np.sin(self.current_angle)
        new_x = self.current_position[0] + dx
        new_y = self.current_position[1] + dy
        
        # 检查是否与墙壁碰撞
        collision = False
        if self.maze_map:
            for wall in self.maze_map:
                if len(wall) == 4:
                    x1, y1, x2, y2 = wall
                    # 简单的碰撞检测
                    if abs(new_x - (x1 + x2) / 2) < 0.05 and ((y1 <= new_y <= y2) or (y2 <= new_y <= y1)):
                        collision = True
                        break
                    if abs(new_y - (y1 + y2) / 2) < 0.05 and ((x1 <= new_x <= x2) or (x2 <= new_x <= x1)):
                        collision = True
                        break
        
        # 如果发生碰撞，改变方向
        if collision:
            self.current_angle = (self.current_angle + np.pi) % (2 * np.pi)  # 反转方向
        else:
            # 确保小车在可视范围内
            if -2.5 <= new_x <= 2.5 and -2.5 <= new_y <= 2.5:
                self.current_position = (new_x, new_y)
        
        # 模拟IMU数据
        self.imu_data = {
            'yaw': self.current_angle * 180 / np.pi % 360,  # 转换为角度
            'gx': int(np.sin(self.current_angle) * 100),
            'gy': int(np.cos(self.current_angle) * 100),
            'gz': 50
        }
        
        # 模拟雷达扫描数据
        self.scan_data.clear()
        for i in range(20):
            scan_angle = self.current_angle + (i - 10) * 0.1
            # 模拟一些障碍物
            if ((self.demo_counter % 50 < 25 and i > 5 and i < 15) or 
                (self.current_position[0] > 0.5 and i < 5) or 
                (self.current_position[1] > 0.5 and i > 15)):
                scan_distance = 0.3  # 障碍物较近
            else:
                scan_distance = 1.0 + 0.5 * np.sin(self.demo_counter * 0.1 + i)  # 随机距离
            
            self.scan_data.append((scan_angle - self.current_angle, scan_distance))
            
            # 添加到地图点
            if scan_distance < 0.6:
                obstacle_x = self.current_position[0] + scan_distance * np.cos(scan_angle)
                obstacle_y = self.current_position[1] + scan_distance * np.sin(scan_angle)
                self.map_points.add((round(obstacle_x, 3), round(obstacle_y, 3)))
        
        # 模拟障碍物检测
        self.obstacle_detected = self.demo_counter % 100 < 10
    
    def update_visualization(self):
        """更新可视化界面"""
        self.ax.clear()
        
        # 设置坐标轴 - 适配迷宫大小
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-2.5, 2.5)  # 扩大范围以适应迷宫
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_title('SLAM 实时地图与小车位置' + (' [演示模式]' if self.demo_mode else ''))
        self.ax.set_xlabel('X坐标 (米)')
        self.ax.set_ylabel('Y坐标 (米)')
        
        # 绘制网格
        self.ax.grid(True, linestyle='--', alpha=0.7)
        
        
        # 绘制地图点
        if self.map_points:
            map_x, map_y = zip(*self.map_points)
            self.ax.scatter(map_x, map_y, c='blue', s=10, alpha=0.7, label='地图点')
        
        # 绘制当前扫描数据（相对于小车）
        if self.scan_data:
            scan_x = []
            scan_y = []
            for angle_rad, distance_m in self.scan_data:
                # 计算相对于小车的坐标
                rel_x = distance_m * np.cos(angle_rad)
                rel_y = distance_m * np.sin(angle_rad)
                # 转换为世界坐标
                world_x = self.current_position[0] + rel_x * np.cos(self.current_angle) - rel_y * np.sin(self.current_angle)
                world_y = self.current_position[1] + rel_x * np.sin(self.current_angle) + rel_y * np.cos(self.current_angle)
                scan_x.append(world_x)
                scan_y.append(world_y)
            
            if scan_x and scan_y:
                self.ax.scatter(scan_x, scan_y, c='red', s=5, alpha=0.5, label='当前扫描')
        
        # 绘制小车位置和方向
        car_size = 0.1
        # 小车中心点
        self.ax.plot(self.current_position[0], self.current_position[1], 'go', markersize=10, label='小车位置')
        # 小车方向指示器
        direction_x = self.current_position[0] + car_size * np.cos(self.current_angle)
        direction_y = self.current_position[1] + car_size * np.sin(self.current_angle)
        self.ax.plot([self.current_position[0], direction_x], [self.current_position[1], direction_y], 'g-', linewidth=3)
        
        # 显示IMU数据
        imu_text = f"IMU数据:\nYaw: {self.imu_data['yaw']:.2f}°\nGx: {self.imu_data['gx']}\nGy: {self.imu_data['gy']}\nGz: {self.imu_data['gz']}"
        self.ax.text(0.02, 0.98, imu_text, transform=self.ax.transAxes, 
                     verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # 如果检测到障碍物，显示警告
        if self.obstacle_detected:
            self.ax.text(0.02, 0.02, "⚠️ 检测到障碍物！", transform=self.ax.transAxes, 
                         verticalalignment='bottom', color='red', fontsize=12, 
                         bbox=dict(boxstyle='round', facecolor='red', alpha=0.2))
        
        # 添加图例
        self.ax.legend(loc='upper right')
        
        # 更新画布
        self.fig.canvas.draw_idle()

if __name__ == "__main__":
    import sys
    
    # 默认端口和是否使用演示模式
    port = 'COM3'  # 根据实际端口修改
    demo_mode = False
    
    # 从命令行参数获取设置（如果有）
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2 and sys.argv[2].lower() == 'demo':
        demo_mode = True
    
    print(f"启动蓝牙SLAM可视化器...")
    print(f"端口: {port}")
    print(f"演示模式: {demo_mode}")
    print("提示：如果无法连接到串口，请尝试使用演示模式运行\n")
    
    # 创建并启动可视化器
    visualizer = BluetoothSLAMVisualizer(port=port, demo_mode=demo_mode)
    visualizer.start()