from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from copy import deepcopy

def navigate_to_pose(navigator, pose):
    navigator.goToPose(pose)
    while not navigator.isNavComplete():
        feedback = navigator.getFeedback()
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()
            return TaskResult.CANCELED
    return navigator.getResult()

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    
    # 设置初始位姿
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # 目标位置列表
    goal_poses = [
        {'x': 1.5, 'y': 1.5},
        {'x': -1.5, 'y': -1.5},
        {'x': -1.5, 'y': -1.5}
    ]

    for goal in goal_poses:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal['x']
        goal_pose.pose.position.y = goal['y']
        goal_pose.pose.orientation.w = 1.0

        result = navigate_to_pose(navigator, goal_pose)
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f'到达目标 ({goal["x"]}, {goal["y"]}) 成功')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn(f'到达目标 ({goal["x"]}, {goal["y"]}) 被取消')
        elif result == TaskResult.FAILED:
            navigator.get_logger().error(f'到达目标 ({goal["x"]}, {goal["y"]}) 失败')

    rclpy.shutdown()

if __name__ == '__main__':
    main()