#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
import subprocess
import signal
import threading
import time
import math
import os
import psutil

class ExploreNavigateNode(Node):
    def __init__(self):
        super().__init__('explore_navigate_node')
        
        # 配置参数（根据实际环境修改）
        self.start_point = Point(x=0.0, y=0.0, z=0.0)  # 起点坐标
        self.goal_point = Point(x=8.0, y=4.0, z=0.0)    # 终点坐标
        self.map_threshold = 0.5  # 地图点有效阈值（米）
        self.explore_timeout = 600  # 探索超时时间（秒）
        self.nav_timeout = 120     # 导航超时时间（秒）
        self.stuck_threshold = 10  # 卡住检测阈值（秒）
        
        # 状态变量
        self.explore_process = None
        self.map_received = False
        self.goal_detected = False
        self.navigation_complete = False
        self.last_distance = float('inf')
        self.last_distance_time = time.time()
        
        # 创建地图订阅
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        # 创建导航动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("探索导航节点已启动，开始探索建图...")
        self.start_exploration()

    def start_exploration(self):
        """启动探索进程"""
        def run_explore():
            self.explore_process = subprocess.Popen(
                ['ros2', 'launch', 'explore_lite', 'explore.launch.py'],
                preexec_fn=os.setsid  # 创建新的进程组
            )
            self.explore_process.wait()
        
        # 在单独线程中运行探索
        self.explore_thread = threading.Thread(target=run_explore)
        self.explore_thread.start()
        
        # 设置探索超时定时器
        self.explore_timer = self.create_timer(
            self.explore_timeout, 
            self.explore_timeout_callback)

    def map_callback(self, msg):
        """处理地图更新"""
        self.map_received = True
        
        # 检查目标点是否已出现在地图中
        if not self.goal_detected:
            self.check_goal_in_map(msg)

    def check_goal_in_map(self, map_msg):
        """检查目标点是否已建图"""
        # 将目标点转换为地图坐标
        map_resolution = map_msg.info.resolution
        map_origin_x = map_msg.info.origin.position.x
        map_origin_y = map_msg.info.origin.position.y
        
        grid_x = int((self.goal_point.x - map_origin_x) / map_resolution)
        grid_y = int((self.goal_point.y - map_origin_y) / map_resolution)
        
        # 检查坐标是否在地图范围内
        if 0 <= grid_x < map_msg.info.width and 0 <= grid_y < map_msg.info.height:
            index = grid_y * map_msg.info.width + grid_x
            
            # 检查该位置是否已知（非未知区域）
            if map_msg.data[index] != -1:  # -1表示未知区域
                self.goal_detected = True
                self.get_logger().info("终点已出现在地图中，停止探索...")
                self.stop_exploration()
                self.navigate_to_goal()

    def stop_exploration(self):
        """停止探索进程"""
        if self.explore_process and self.explore_process.poll() is None:
            # 终止整个进程组
            try:
                os.killpg(os.getpgid(self.explore_process.pid), signal.SIGTERM)
                self.get_logger().info("已发送SIGTERM信号给探索进程组")
            except ProcessLookupError:
                self.get_logger().warn("进程组不存在，可能已退出")
            
            # 等待进程退出
            try:
                self.explore_process.wait(timeout=5.0)
                self.get_logger().info("探索进程已停止")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("探索进程未正常退出，强制终止...")
                try:
                    os.killpg(os.getpgid(self.explore_process.pid), signal.SIGKILL)
                    self.explore_process.wait(timeout=2.0)
                except:
                    pass
            
            # 确保所有相关进程都被终止
            self.kill_ros_processes('explore_lite')
            
            self.explore_process = None

    def kill_ros_processes(self, process_name):
        """杀死所有指定名称的ROS进程"""
        self.get_logger().info(f"检查并终止所有{process_name}进程...")
        for proc in psutil.process_iter(['name', 'cmdline']):
            try:
                if proc.info['cmdline'] and any(process_name in cmd for cmd in proc.info['cmdline']):
                    self.get_logger().info(f"终止进程: {proc.pid} {proc.info['cmdline']}")
                    proc.terminate()
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass

    def explore_timeout_callback(self):
        """探索超时处理"""
        if not self.goal_detected:
            self.get_logger().warn("探索超时，未检测到目标点")
            self.stop_exploration()
            self.navigate_to_goal()

    def navigate_to_goal(self):
        """导航到终点位置"""
        self.get_logger().info("开始导航到终点位置...")
        self.navigate_to_point(self.goal_point, self.goal_navigation_complete)

    def goal_navigation_complete(self, result):
        """导航到终点完成回调"""
        if result:
            self.get_logger().info("已成功到达终点，开始导航回起点...")
            self.navigate_to_start()
        else:
            self.get_logger().error("导航到终点失败，任务终止")
            self.cleanup_and_shutdown()

    def navigate_to_start(self):
        """导航回起点位置"""
        self.get_logger().info("开始导航回起点位置...")
        self.navigate_to_point(self.start_point, self.start_navigation_complete)

    def start_navigation_complete(self, result):
        """导航回起点完成回调"""
        if result:
            self.get_logger().info("已成功返回起点，任务完成！")
        else:
            self.get_logger().error("导航回起点失败")
        self.cleanup_and_shutdown()

    def navigate_to_point(self, point, callback):
        """导航到指定点"""
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = point
        goal_pose.pose.orientation.w = 1.0  # 默认朝向
        
        goal_msg.pose = goal_pose
        
        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback)
        
        self.send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, callback))

    def goal_response_callback(self, future, callback):
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            callback(False)
            return
        
        self.get_logger().info('目标已接受，导航中...')
        self.result_future = goal_handle.get_result_async()
        
        # 设置导航超时定时器
        self.nav_timer = self.create_timer(
            self.nav_timeout,
            lambda: self.navigation_timeout_callback(goal_handle))
        
        # 设置卡住检测定时器
        self.stuck_timer = self.create_timer(
            1.0,  # 每秒检查一次
            self.check_stuck)
        
        self.result_future.add_done_callback(
            lambda future: self.navigation_result_callback(future, callback))

    def navigation_result_callback(self, future, callback):
        """导航结果回调"""
        # 取消导航超时定时器
        if hasattr(self, 'nav_timer'):
            self.nav_timer.cancel()
        
        # 取消卡住检测定时器
        if hasattr(self, 'stuck_timer'):
            self.stuck_timer.cancel()
        
        result = future.result().result
        # 检查导航结果是否成功
        # 实际应用中应根据导航状态进行判断
        # 这里简化处理，假设动作完成即成功
        callback(True)

    def navigation_timeout_callback(self, goal_handle):
        """导航超时处理"""
        self.get_logger().warn("导航超时，取消当前目标")
        goal_handle.cancel_goal_async()
        self.result_future.cancel()
        self.navigation_result_callback(self.result_future, lambda result: False)

    def check_stuck(self):
        """检查机器人是否卡住"""
        current_time = time.time()
        if current_time - self.last_distance_time > self.stuck_threshold:
            if abs(self.last_distance - self.current_distance) < 0.1:  # 10厘米内移动视为卡住
                self.get_logger().warn("机器人可能卡住，尝试重新规划路径")
                # 这里可以添加重新规划路径的逻辑
                # 或者取消当前目标并重新发送
                self.navigation_timeout_callback(self.current_goal_handle)
        
        # 更新最后记录的距离和时间
        self.last_distance = self.current_distance
        self.last_distance_time = current_time

    def navigation_feedback_callback(self, feedback_msg):
        """导航反馈回调"""
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'distance_remaining'):
            self.current_distance = feedback.distance_remaining
            self.get_logger().info(f'剩余距离: {self.current_distance:.2f}米')
        else:
            self.get_logger().info('收到导航反馈...')

    def cleanup_and_shutdown(self):
        """清理资源并关闭节点"""
        self.get_logger().info("清理资源并关闭节点...")
        self.navigation_complete = True
        
        # 确保所有定时器都被取消
        if hasattr(self, 'explore_timer'):
            self.explore_timer.cancel()
        if hasattr(self, 'nav_timer'):
            self.nav_timer.cancel()
        if hasattr(self, 'stuck_timer'):
            self.stuck_timer.cancel()
        
        # 安全关闭节点
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ExploreNavigateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()