import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy import parameter
from geometry_msgs.msg import Twist
from odrive_ep_interfaces.msg import Motor
from odrive_ep_interfaces.srv import OdriveMode


class ControlVelNode(Node):
    def __init__(self):
            super().__init__('livo_driver')
            self.QoS_ = 10
            self.declare_parameter('mobile_robot_length', 0.30)
            self.length = self.get_parameter('mobile_robot_length').value # 중심점으로부터 모터의 세로 위치
            self.declare_parameter('mobile_robot_width', 0.40)
            self.width = self.get_parameter('mobile_robot_width').value # 중심점으로부터 모터의 가로 위치
            self.declare_parameter('mobile_robot_radius', 0.0625)
            self.radius = self.get_parameter('mobile_robot_radius').value # 메카넘휠 반지름
            self.vel_x = 0.0
            self.vel_y = 0.0
            self.ori_z = 0.0
            self.fl, self.fr, self.rl, self.rr = 0,0,0,0
            self.vel_publisher = self.create_publisher(Motor, 'goal_posvel_double', self.QoS_)
            #wheel/odometry
            # 모터의 실제 각속도를 받아온다.
            self.vel_subscription = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                self.QoS_)


    def cmd_vel_callback(self,msg):
        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y
        self.ori_z = msg.angular.z
        self.twist_to_vel()
        self.publish()


    def twist_to_vel(self):
        self.fl = (1/self.radius) * (self.vel_x - self.vel_y - (self.width + self.length) * self.ori_z)
        self.fr = (1/self.radius) * (self.vel_x + self.vel_y + (self.width + self.length) * self.ori_z)
        self.rl = (1/self.radius) * (self.vel_x + self.vel_y - (self.width + self.length) * self.ori_z)
        self.rr = (1/self.radius) * (self.vel_x - self.vel_y + (self.width + self.length) * self.ori_z)

    def publish(self):
        msg_motor = Motor()
        msg_motor.motor0.pos = 0.0
        msg_motor.motor1.pos = 0.0
        msg_motor.motor2.pos = 0.0
        msg_motor.motor3.pos = 0.0

        msg_motor.motor0.vel = self.fl
        msg_motor.motor1.vel = self.fr
        msg_motor.motor2.vel = self.rl
        msg_motor.motor3.vel = self.rr

        self.vel_publisher.publish(msg_motor)

def main(args=None):
    rclpy.init(args=args)
    node = ControlVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()