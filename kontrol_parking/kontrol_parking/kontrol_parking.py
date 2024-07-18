import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import tf_transformations
import math
from enum import Enum
from std_msgs.msg import UInt8, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class ControlParking(Node):
    def __init__(self):
        super().__init__('control_parking')

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cbOdom, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.sub_max_vel = self.create_publisher(Float64, '/control/max_vel', self.isfnParking, QoSProfile(depth=1))
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=1))
        self.pub_parking_finished = self.create_publisher(UInt8, '/control/parking_finished', QoSProfile(depth=1))
        self.pub_parking_start = self.create_publisher(UInt8, '/control/parking_start', QoSProfile(depth=1))
        self.pub_max_vel = self.create_publisher(Float64, '/control/max_vel', QoSProfile(depth=1))

        self.pub_parking_lot_return = self.create_publisher(UInt8, '/detect/parking_lot_stamped', QoSProfile(depth=1))

        self.StepOfParking = Enum('StepOfParking', 'idle outer_turn_first parking_lot_entry parking_lot_turn_first parking_lot_stop parking_lot_turn_second parking_lot_exit outer_turn_second')
        self.current_step_of_parking = self.StepOfParking.outer_turn_first.value

        self.theta = 0.0
        self.last_current_theta = 0.0
        
        self.is_step_start = False
        
        self.lastError = 0.0
        
        print("1111111")

#############################################################################################################################################################################
    # 控制停车

    def cbOdom(self, odom_msg):
        print("000000000000")
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)

        self.current_theta = self.euler_from_quaternion(quaternion)
        # print(self.current_theta)
        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

    def euler_from_quaternion(self, quaternion):
        # 将四元素转换为欧拉角
        theta = tf_transformations.euler_from_quaternion(quaternion)[2]
        return theta

    def fnParking(self):
        print("33333333")
        if self.current_step_of_parking == self.StepOfParking.outer_turn_first.value:
            self.get_logger().info("outer_turn_first")
            if self.current_theta != 0.0:
                if self.is_step_start == False:
                    self.lastError = 0.0
                    self.desired_theta = self.current_theta - 1.57   # 计算期望角度，从当前角度减去90度
                    self.is_step_start = True

            error = self.fnTurn()  # 计算转向误差

            if math.fabs(error) < 0.05:    # 检测转向是否完成，误差绝对值小于0.05
                self.get_logger().info("outer_turn_first finished")
                self.current_step_of_parking = self.StepOfParking.parking_lot_entry.value
                self.is_step_start = False

        elif self.current_step_of_parking == self.StepOfParking.parking_lot_entry.value:
            self.get_logger().info("parking_lot_entry")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.25)

            if math.fabs(error) < 0.005:
                self.get_logger().info("parking_lot_entry finished")
                self.current_step_of_parking = self.StepOfParking.parking_lot_turn_first.value
                self.is_step_start = False            

        elif self.current_step_of_parking == self.StepOfParking.parking_lot_turn_first.value:
            self.get_logger().info("parking_lot_turn_first")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                self.get_logger().info("parking_lot_turn_first finished")
                self.current_step_of_parking = self.StepOfParking.parking_lot_stop.value
                self.is_step_start = False

        elif self.current_step_of_parking == self.StepOfParking.parking_lot_stop.value:
            self.get_logger().info("parking_lot_stop")
            self.fnStop()

            # rospy.sleep(2)
            rclpy.shutdown()

            self.get_logger().info("parking_lot_stop finished")
            self.current_step_of_parking = self.StepOfParking.parking_lot_turn_second.value

        elif self.current_step_of_parking == self.StepOfParking.parking_lot_turn_second.value:
            if self.is_step_start == False:
                self.get_logger().info("parking_lot_turn_second")
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                self.get_logger().info("parking_lot_turn_second finished")
                self.current_step_of_parking = self.StepOfParking.parking_lot_exit.value
                self.is_step_start = False

        elif self.current_step_of_parking == self.StepOfParking.parking_lot_exit.value:
            self.get_logger().info("parking_lot_exit")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.25)

            if math.fabs(error) < 0.005:
                self.get_logger().info("parking_lot_exit finished")
                self.current_step_of_parking = self.StepOfParking.outer_turn_second.value
                self.is_step_start = False   

        elif self.current_step_of_parking == self.StepOfParking.outer_turn_second.value:
            self.get_logger().info("outer_turn_second")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                self.get_logger().info("outer_turn_second finished")
                self.current_step_of_parking = self.StepOfParking.idle.value
                self.is_step_start = False

        else:
            self.get_logger().info("idle (if finished to go out from parking lot)")

            self.fnStop()
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.20
            self.pub_max_vel.publish(msg_pub_max_vel)
            msg_parking_finished = UInt8()
            msg_parking_finished.data = 1
            self.pub_parking_finished.publish(msg_parking_finished)
    def isfnParking(self, msg):
        if msg.data == 0.02:
            self.fnParking()
        
###################################################################################################################################################

    def fnTurn(self):    # 转向控制函数
        err_theta = self.current_theta - self.desired_theta    # 计算角度差值
        
        self.get_logger().info("Parking_Turn")
        print("err_theta  desired_theta  current_theta : %f  %f  %f", err_theta, self.desired_theta, self.current_theta)
        # PD控制算法
        Kp = 0.8   # 比例控制

        Kd = 0.03  # 微分控制

        angular_z = Kp * err_theta + Kd * (err_theta - self.lastError)
        self.lastError = err_theta

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

        print("angular_z : %f", angular_z)

        return err_theta

    def fnStraight(self, desired_dist):
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
          
        self.get_logger().info("Parking_Straight")


        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.07
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

        return err_pos

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)


    def fnShutDown(self):
        self.get_logger().info("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist) 



    def main(self):

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    control_parking = ControlParking()
    rclpy.spin(control_parking)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
