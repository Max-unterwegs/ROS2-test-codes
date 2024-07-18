import math
import time
from irobot_create_msgs.action import  DriveDistance, RotateAngle
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
class driveTest(Node):
    def __init__(self):
        super().__init__('driveTest')
        driveTests()

def driveTests(self):

        # 定义行驶距离和旋转角度
        drive_distance = 0.25
        rotate_angle = math.pi/2

        # 创建行驶和旋转的动作客户端
        drive_action_client = ActionClient(self, DriveDistance, '/drive_distance')
        rotate_action_client = ActionClient(self, RotateAngle, '/rotate_angle')
        # 初始化行驶和旋转的目标消息
        drive_goal_msg = DriveDistance.Goal()
        drive_goal_msg.distance = drive_distance
        rotate_goal_msg = RotateAngle.Goal()
        rotate_goal_msg.angle = rotate_angle


        # 通知测试开始，并等待用户确认
        print('The robot will drive forwards 0.25m then turn 90 degrees 4 times.')
        input('Press enter to start.')

        # 执行4次行驶和旋转的动作序列
        for i in range(0, 4):
            # 等待动作服务器就绪，并发送行驶目标
            drive_action_client.wait_for_server()
            drive_goal_result = drive_action_client.send_goal(drive_goal_msg)
            time.sleep(1)  # 等待动作完成

            # 等待动作服务器就绪，并发送旋转目标
            rotate_action_client.wait_for_server()
            rotate_goal_result = rotate_action_client.send_goal(rotate_goal_msg)
            time.sleep(1)  # 等待动作完成


        # 关闭动作客户端
        self.destroy_client(drive_action_client)
        self.destroy_client(rotate_action_client)
        
        # 测试执行完成，返回True
        return True

def main(args=None):
    rclpy.init(args=args)
    driveTest = driveTest()
    rclpy.spin(driveTest)
    #level_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
