import rclpy
from rclpy.node import Node
import time
import numpy as np
from odrive_ep_interfaces.msg import Motor
from odrive_ep_interfaces.srv import OdriveMode
from sensor_msgs.msg import Joy
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor



#           front m0    |--|            |--|     front m1
#                       |--|  --------  |--|
#                             |      |
#                             |      |
#                             |      |
#           back m0     |--|  --------  |--|     back m1
#                       |--|            |--|
#
#




class ControlClass(Node):

    def __init__(self, seconds_sleeping=10):
        super().__init__('odrive_joy_node')
        self._seconds_sleeping = seconds_sleeping
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()

        self.get_logger().info("Init...")


        self.motorf_pub = self.create_publisher(Motor, 'motor_front', 10)
        self.motorb_pub = self.create_publisher(Motor, 'motor_back', 10)
        self.motorf = Motor()
        self.motorb = Motor()
        # self.motorf_client = self.create_client(OdriveMode, 'odrive_mode_srv_fornt', callback_group=self.group2)
        # self.motorb_client = self.create_client(OdriveMode, 'odrive_mode_srv_back', callback_group=self.group3)
#        while not self.motorf_client.wait_for_service(timeout_sec=1.0):
#            self.get_logger().info('motorf service not available, waiting again...')
#        while not self.motorb_client.wait_for_service(timeout_sec=1.0):
#            self.get_logger().info('motorb service not available, waiting again...')
        

        self.reqf = OdriveMode.Request()
        
        self.reqb = OdriveMode.Request()

        self.posarr = [0.0, 0.0, 0.0, 0.0]
        self.velarr = [0.0, 0.0, 0.0, 0.0]
        self.curMode = 'vel'

        self.max_speed = 10
        
        
        self.LEFT_STICK_X = 0
        self.LEFT_STICK_Y = 1
        #self.LEFT_TRIGGER = 2
        self.RIGHT_STICK_X = 2
        self.RIGHT_STICK_Y = 3
        #self.RIGHT_TRIGGER = 5
        self.D_PAD_X = 4
        self.D_PAD_Y = 5
    
        self.A = 1
        self.B = 0
        self.X = 2
        self.Y = 3
        self.LEFT_BUMPER = 5
        self.RIGHT_BUMPER = 6
        self.PLUS = 9
        self.MINUS = 10
        self.CAP = 4
        self.HOME = 17
        self.ZL = 7
        self.ZR = 8
        self.LEFT_STICK_CLICK = 11
        self.RIGHT_STICK_CLICK = 12





        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10,  callback_group=self.group1)
        self.timerf = self.create_timer(
            0.1, self.timer_callbackf, callback_group=self.group2)
        self.timerb = self.create_timer(
            0.1, self.timer_callbackb, callback_group=self.group3)
        
        self.get_logger().info("Done.")


    # def send_request_motorf(self, mode):
    #     self.reqf.mode = mode
    #     self.futuref = self.motorf_client.call_async(self.reqf)    

    # def send_request_motorb(self, mode):
    #     self.reqb.mode = mode
    #     self.futureb = self.motorb_client.call_async(self.reqb)   



    def joy_callback(self, msg):
        self.get_logger().info(f"Input : {msg}")

        if msg.buttons[self.PLUS] == 1 :
            if self.curMode != "vel":
                self.send_request_motorf("vel")
                while not self.futuref.done():
                    if self.check_res_motorf():
                        self.curMode = "vel"
                self.send_request_motorb("vel")
                while not self.futuref.done():
                    if self.check_res_motorb():
                        self.curMode = "vel"

        if msg.buttons[self.MINUS] == 1 :
            if self.curMode != "pos":
                self.send_request_motorf("pos")
                while not self.futuref.done():
                    if self.check_res_motorf():
                        self.curMode = "pos"
                self.send_request_motorb("pos")
                while not self.futuref.done():
                    if self.check_res_motorb():
                        self.curMode = "pos"


        if abs(msg.axes[self.LEFT_STICK_Y]) > 0.09:
            self.move_y(msg.axes[self.LEFT_STICK_Y] * self.max_speed)
        elif abs(msg.axes[self.LEFT_STICK_X]) > 0.09:
            self.move_x(msg.axes[self.LEFT_STICK_X] * self.max_speed)
        else:
            self.stop()


            
    def move_y(self, data):
        for i in range(4):
            self.velarr[i] = data
    def move_x(self, data):
        self.velarr[0] = data
        self.velarr[1] = -1 * data
        self.velarr[2] = data
        self.velarr[3] = -1 * data

    def stop(self):
        self.velarr = [0.0,0.0,0.0,0.0]






        

    def timer_callbackf(self):
        msg = Motor()
        msg.motor0.pos = self.posarr[0]
        msg.motor0.vel = self.velarr[0]
        msg.motor1.pos = self.posarr[1]
        msg.motor1.vel = self.velarr[1]
        self.motorf_pub.publish(msg)


    def timer_callbackb(self):
        msg = Motor()
        msg.motor0.pos = self.posarr[2]
        msg.motor0.vel = self.velarr[2]
        msg.motor1.pos = self.posarr[3]
        msg.motor1.vel = self.velarr[3]
        self.motorb_pub.publish(msg)

    # def check_res_motorf(self):
    #     if self.futuref.done():
    #         try:
    #             response_f = self.futuref.result()
    #             if response_f:
    #                 self.get_logger().warn(f"motor_front service call executed. command : {self.reqf}")
    #             else:
    #                 self.get_logger().warn(f"motor_front service call execute failed. command : {self.reqf}")
    #             return response_f
    #         except Exception as e:
    #             self.get_logger().info(
    #                 'motor_front Service call failed %r' % (e,))
    # def check_res_motorb(self):
    #     if self.futureb.done():
    #         try:
    #             response_b = self.futureb.result()
    #             if response_b:
    #                 self.get_logger().warn(f"motor_back service call executed. command : {self.reqb}")
    #             else:
    #                 self.get_logger().warn(f"motor_back service call execute failed. command : {self.reqb}")
    #             return response_b
    #         except Exception as e:
    #             self.get_logger().info(
    #                 'motor_back Service call failed %r' % (e,))
        
        


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlClass()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(control_node)

    # control_node.send_request_motorf("clear errors")
    # while not control_node.futuref.done():
    #     control_node.check_res_motorf()

    # control_node.send_request_motorb("clear errors")
    # while not control_node.futureb.done():
    #     control_node.check_res_motorb()

    # control_node.send_request_motorf("closed loop")
    # while not control_node.futuref.done():
    #     control_node.check_res_motorf()

    # control_node.send_request_motorb("closed loop")
    # while not control_node.futureb.done():
    #     control_node.check_res_motorb()

    while rclpy.ok():
        executor.spin_once()
        # control_node.check_res_motorf()
        # control_node.check_res_motorf()

    control_node.get_logger().warn("destory node!!")
    executor.shutdown()
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
