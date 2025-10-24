import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

class AutoMazeMission(Node):
    def __init__(self):
        super().__init__('auto_maze_mission')

        # 当前状态： exploring -> going_exit -> going_entry -> done
        self.state = "exploring"

        # 已知迷宫入口和出口坐标（可以修改）
        self.entry_pose = (0.0, 0.0)           #入口位置
        self.exit_pose = (5.0, 10.0)           #出口位置

        # frontier判断
        self.frontier_empty_count = 0

        # 发布目标导航
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # 控制探索暂停/恢复
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)

        # 订阅探索前沿
        self.frontier_sub = self.create_subscription(
            MarkerArray, '/explore/frontiers', self.frontier_callback, 10
        )

        self.get_logger().info("🤖 自动迷宫任务启动：正在探索中...")

    def frontier_callback(self, msg):
        # 当frontier为空时计数
        if len(msg.markers) == 0:
            self.frontier_empty_count += 1
        else:
            self.frontier_empty_count = 0

        # 连续多次frontier为空，认为探索完成
        if self.frontier_empty_count > 5 and self.state == "exploring":
            self.get_logger().info("🗺️ 探索完成，暂停探索，准备前往出口")
            self.pause_exploration()
            self.goto_pose(self.exit_pose)
            self.state = "going_exit"

    def pause_exploration(self):
        msg = Bool()
        msg.data = False
        self.resume_pub.publish(msg)

    def resume_exploration(self):
        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)

    def goto_pose(self, pose):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = pose[0]
        goal.pose.position.y = pose[1]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    # 你可以订阅 Nav2 的 result topic 或 action反馈来调用这个函数
    def on_goal_reached(self):
        if self.state == "going_exit":
            self.get_logger().info("✅ 到达出口，返回入口")
            self.goto_pose(self.entry_pose)
            self.state = "going_entry"
        elif self.state == "going_entry":
            self.get_logger().info("🎯 回到入口，任务完成！")
            self.state = "done"

def main():
    rclpy.init()
    node = AutoMazeMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
