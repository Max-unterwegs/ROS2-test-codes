import os
def main():
    os.system('ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0,max_translation_speed: 0.7}"')
    # 倒车入库
    os.system('ros2 action send_goal /drive_arc irobot_create_msgs/action/DriveArc "{angle: 1.57,radius: 0.6,translate_direction: 1,max_translation_speed: 1.0}"')
    # 转向
    # os.system('sleep 2')
    os.system('ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: -1.57,max_rotation_speed: 0.9}"')
    # os.system('sleep 2')
    # 向前走一段距离
    os.system('ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.2,max_translation_speed: 1.0}"')
    # os.system('sleep 2')
    # 转个弯
    os.system('ros2 action send_goal /drive_arc irobot_create_msgs/action/DriveArc "{angle: 1.57,radius: 0.3,translate_direction: 1,max_translation_speed: 1.0}"')


if __name__ == '__main__':
    main()
