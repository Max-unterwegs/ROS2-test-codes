ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0,max_translation_speed: 0.7}"
ros2 action send_goal /drive_arc irobot_create_msgs/action/DriveArc "{angle: 1.57,radius: 0.6,translate_direction: -1,max_translation_speed: 1.0}"
