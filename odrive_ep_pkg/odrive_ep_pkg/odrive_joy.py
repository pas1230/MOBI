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
#           back m2     |--|  --------  |--|     back m3
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


        self.motor_pub = self.create_publisher(Motor, 'goal_posvel_double', 10)
        self.motor = Motor()
        self.motor_client = self.create_client(OdriveMode, 'odrive_mode_srv', callback_group=self.group3)
        while not self.motor_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('motor service not available, waiting again...')
        self.srv_busy = False
        

        self.request = OdriveMode.Request()

        self.posarr = [0.0, 0.0, 0.0, 0.0]
        self.velarr = [0.0, 0.0, 0.0, 0.0]
        self.curMode = 'vel'

        self.max_speed = 15
        self.rotate_speed = 6.0
        
        
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
            Joy, '/joy', self.joy_callback, 10,  callback_group=self.group2)

        self.timer = self.create_timer(
            0.1, self.timer_callback, callback_group=self.group1)
        
        self.get_logger().info('\033[96m' + 'Done!' + '\033[0m')


    def send_request_motor(self, mode):
        while self.srv_busy:
            pass
        self.request.mode = mode
        self.future = self.motor_client.call_async(self.request) 
        self.srv_busy = True     



    def joy_callback(self, msg):
        self.get_logger().info(f"Input : {msg}")

        if msg.buttons[self.PLUS] == 1 :
            if self.curMode != "vel":
                self.send_request_motor("vel")
                while not self.future.done():
                    if self.check_res_motor():
                        self.curMode = "vel"

        if msg.buttons[self.MINUS] == 1 :
            if self.curMode != "pos":
                self.send_request_motor("pos")
                while not self.future.done():
                    if self.check_res_motor():
                        self.curMode = "pos"
        

        if abs(msg.axes[self.LEFT_STICK_Y]) > 0.09:
            self.move_y(msg.axes[self.LEFT_STICK_Y] * self.max_speed)
        elif abs(msg.axes[self.LEFT_STICK_X]) > 0.09:
            self.move_x(msg.axes[self.LEFT_STICK_X] * self.max_speed)
        elif msg.buttons[self.LEFT_BUMPER] == 1:
            self.rotate_l()
        elif msg.buttons[self.RIGHT_BUMPER] == 1:
            self.rotate_r()
        else:
            self.stop()


            
    def move_y(self, data):
        self.velarr[0] = -1 * data
        self.velarr[1] = data
        self.velarr[2] = data
        self.velarr[3] = -1 * data
        
    def move_x(self, data):
        self.velarr[0] = data
        self.velarr[1] = data
        self.velarr[2] = -1 * data
        self.velarr[3] = -1 * data
        
    def rotate_r(self):
        self.velarr[0] = self.rotate_speed
        self.velarr[1] = self.rotate_speed
        self.velarr[2] = self.rotate_speed
        self.velarr[3] = self.rotate_speed
        
    def rotate_l(self):
        self.velarr[0] = -1 * self.rotate_speed
        self.velarr[1] = -1 * self.rotate_speed
        self.velarr[2] = -1 * self.rotate_speed
        self.velarr[3] = -1 * self.rotate_speed

    def stop(self):
        self.velarr = [0.0,0.0,0.0,0.0]






        

    def timer_callback(self):
        msg = Motor()
        msg.motor0.pos = self.posarr[0]
        msg.motor0.vel = self.velarr[0]
        msg.motor1.pos = self.posarr[1]
        msg.motor1.vel = self.velarr[1]
        msg.motor2.pos = self.posarr[2]
        msg.motor2.vel = self.velarr[2]
        msg.motor3.pos = self.posarr[3]
        msg.motor3.vel = self.velarr[3]
        self.motor_pub.publish(msg)

    def check_res_motor(self):
        if self.future.done() and self.srv_busy:
            try:
                response = self.future.result()
                if response:
                    self.get_logger().warn(f"motor service call executed. command : {self.request}")
                else:
                    self.get_logger().warn(f"motor service call execute failed. command : {self.request}")
                self.srv_busy = False
                return response
            except Exception as e:
                self.get_logger().info(
                    'motor Service call failed %r' % (e,))
        
        


def main(args=None):
    time.sleep(15)
    rclpy.init(args=args)
    control_node = ControlClass()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(control_node)
    count = 0

    # control_node.send_request_motor("clear errors")
    # while not control_node.future.done():
    #     control_node.get_logger().info("clear error...")
    #     control_node.check_res_motor()
    #     count += 1
    #     time.sleep(1)
    #     if count >= 10 :
    #         control_node.get_logger().error("error clear failed!!")
    #         count = 0
    #         break


    control_node.send_request_motor("closed loop")
    while not control_node.future.done():
        control_node.get_logger().info("set closed loop...")
        control_node.check_res_motor()
        count += 1
        time.sleep(1)
        if count >= 10 :
            control_node.get_logger().error("closed loop failed!!")
            count = 0
            break



    while rclpy.ok():
        executor.spin_once()
        control_node.check_res_motor()

    control_node.get_logger().warn("destory node!!")
    executor.shutdown()
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
