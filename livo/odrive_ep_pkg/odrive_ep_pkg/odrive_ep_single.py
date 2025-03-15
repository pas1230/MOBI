import odrive
#import the Odrivetool libraries
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from odrive_ep_interfaces.msg import Motorsingle
from odrive_ep_interfaces.srv import OdriveMode

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile

import time

num_threads = 4
#odrive_serial_id1 = "356F30543133"                  #바꿔줄것 
#odrive_serial_id2 = "356F30733133"

odrive_serial_id1 = "3369356B3233"                  #바꿔줄것 




calibration_time = 15




class Odrive_ep(Node):

    def __init__(self):
        super().__init__('odrive_ep_single')
        

        self.goal_pos = [0.0, 0.0]
        self.goal_vel = [0.0, 0.0]
        self.cur_pos = [0.0, 0.0]
        self.cur_vel = [0.0, 0.0]

        self.odrv1 = odrive.find_any(serial_number=odrive_serial_id1)
        self.get_logger().info(f"odrv1 connected.  voltage : {self.odrv1.vbus_voltage}")
        self.init_odrive()
        
        self.get_logger().info("## DEFAULT Node Callback Group=" +
              str(self.default_callback_group))

        self.get_logger().warning("Setting MutallyExclusive Groups")
        # If you set the group reentrant, any Callback inside will be executed in parallel
        # If there are enough threads
        #self.group1 = ReentrantCallbackGroup()
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = MutuallyExclusiveCallbackGroup()
        self.get_logger().warning("MutallyExclusiveCallbackGroup Set")

        self.publisher_ = self.create_publisher(MotorSingle, 'current_posvel_single', 10, callback_group=self.group1)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.group2)

        self.subscriber = self.create_subscription(
            Motorsingle,
            'goal_posvel_single',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group3)

        self.srv = self.create_service(
            OdriveMode, 'odrive_mode_srv_single', self.SetMode_callback, callback_group=self.group4)
        
    def timer_callback(self):
        
        msg = Motorsingle()
        posvel = self.read_posvel()

        self.cur_pos = posvel[0]
        self.cur_vel = posvel[1]

        self.odrv1.axis0.watchdog_feed()
        self.odrv1.axis1.watchdog_feed()

        self.read_error()

        msg.motor0.pos = self.cur_pos[0]
        msg.motor0.vel = self.cur_vel[0]

        msg.motor1.pos = self.cur_pos[1]
        msg.motor1.vel = self.cur_vel[1]

        

        # Publish the message to the Topic
        self.publisher_.publish(msg)
        # Display the message on the console
        #self.get_logger().info(f"{msg}")

        self.set_pos(self.goal_pos)
        self.set_vel(self.goal_vel)

    
    def listener_callback(self, msg):
        #self.get_logger().info(f"set goal {msg}")
        self.goal_pos = [msg.motor0.pos, msg.motor1.pos]
        self.goal_vel = [msg.motor0.vel, msg.motor1.vel]


    def SetMode_callback(self, request, response):
        mode = request.mode
        self.get_logger().warning(f"Set control mode to {mode}...")
        if mode == "pos":
            self.set_traj_control()
            self.get_logger().warning(f"Set control mode to pos_traj...DONE")
            response.success = True
        elif mode == "vel":
            self.set_vel_control()
            self.get_logger().warning(f"Set control mode to vel_ramp...DONE")
            response.success = True
        elif mode == "clear errors":
            self.odrv1.axis0.clear_errors()
            self.odrv1.axis1.clear_errors()
            self.get_logger().warning(f"Error clear...DONE")
            response.success = True
        elif mode == "closed loop":
            self.get_logger().warning(f"Set motor mode to closed loop...")
            self.odrv1.axis0.requested_state = 8         # CLOSED_LOOP_CONTROL
            self.odrv1.axis1.requested_state = 8         # CLOSED_LOOP_CONTROL
            #time.sleep(0.2)
            self.get_logger().warning(f"Set motor mode to closed loop...DONE")
        else:
            self.get_logger().error(f"no mode for {mode}")
            response.success = False

        # return the response parameters
        return response
        

    def init_odrive(self):

        self.odrv1.axis0.config.enable_watchdog = False
        self.odrv1.axis1.config.enable_watchdog = False
        time.sleep(0.1)
        self.odrv1.axis0.clear_errors()
        self.odrv1.axis1.clear_errors()
        time.sleep(0.1)
        
    
        self.get_logger().info("----------------------------Calibration--------------------------------")
        self.get_logger().info("Calibrating motors...")
        self.odrv1.axis0.requested_state = 3 # AXIS_STATE_FULL_CALIBRATION
        self.odrv1.axis1.requested_state = 3 # AXIS_STATE_FULL_CALIBRATION
        time.sleep(calibration_time)
        if self.odrv1.axis0.motor.is_calibrated == 1 and self.odrv1.axis1.motor.is_calibrated == 1:
            self.get_logger().info("Motor successfully calibrated! Proceeding...")
        else:
            self.get_logger().fatal("Could not calibrate motor. Something is wrong.")

        if self.odrv1.axis0.encoder.is_ready == 1 and self.odrv1.axis1.encoder.is_ready == 1:
            self.get_logger().info("Encoder offset successfully calibrated! Proceeding...")
        else:
            self.get_logger().fatal("Could not calibrate encoder offset. Something is wrong.")
        
        self.get_logger().info("Enable Closed loop...")
        time.sleep(1)
        self.odrv1.axis0.requested_state = 8         # CLOSED_LOOP_CONTROL
        self.odrv1.axis1.requested_state = 8         # CLOSED_LOOP_CONTROL
        time.sleep(3)
        self.get_logger().info("Enable Closed loop...DONE")


        self.odrv1.axis0.config.watchdog_timeout = 2
        self.odrv1.axis1.config.watchdog_timeout = 2
        self.odrv1.axis0.watchdog_feed()
        self.odrv1.axis1.watchdog_feed()
        time.sleep(0.1)

        self.odrv1.axis0.trap_traj.config.vel_limit = 10
        self.odrv1.axis0.trap_traj.config.accel_limit = 2
        self.odrv1.axis0.trap_traj.config.decel_limit = 2
        self.odrv1.axis1.trap_traj.config.vel_limit = 10
        self.odrv1.axis1.trap_traj.config.accel_limit = 2
        self.odrv1.axis1.trap_traj.config.decel_limit = 2


        self.odrv1.axis0.config.enable_watchdog = True
        self.odrv1.axis1.config.enable_watchdog = True


        


    '''self.odrv1.axis0.trap_traj.config.vel_limit = 40
        self.odrv1.axis0.trap_traj.config.accel_limit = 20
        self.odrv1.axis0.trap_traj.config.decel_limit = 20
        self.odrv1.axis1.trap_traj.config.vel_limit = 40
        self.odrv1.axis1.trap_traj.config.accel_limit = 20
        self.odrv1.axis1.trap_traj.config.decel_limit = 20 '''
    
        
        

        


    
    def set_traj_control(self):

        #self.odrv1.axis0.controller.input_pos = self.odrv1.axis0.encoder.pos_estimate
        #self.odrv1.axis1.controller.input_pos = self.odrv1.axis1.encoder.pos_estimate
        self.odrv1.axis0.controller.config.input_mode = 1         #PASSTHROUGH
        self.odrv1.axis1.controller.config.input_mode = 1         #PASSTHROUGH

        self.odrv1.axis0.controller.input_pos = self.odrv1.axis0.encoder.pos_estimate
        self.odrv1.axis1.controller.input_pos = self.odrv1.axis1.encoder.pos_estimate

        self.odrv1.axis0.controller.config.control_mode = 3         #pos_mode
        self.odrv1.axis1.controller.config.control_mode = 3         #pos_mode

        self.odrv1.axis0.controller.config.input_mode = 5         #TRAP_TRAJ
        self.odrv1.axis1.controller.config.input_mode = 5         #TRAP_TRAJ

    def set_vel_control(self):
        self.odrv1.axis0.controller.config.control_mode = 2         #VEL_mode
        self.odrv1.axis1.controller.config.control_mode = 2         #VEL_mode
        self.odrv1.axis0.controller.config.input_mode = 2         #VEL_RAMP
        self.odrv1.axis1.controller.config.input_mode = 2         #VEL_RAMP
        self.odrv1.axis0.controller.config.vel_ramp_rate = 40
        self.odrv1.axis1.controller.config.vel_ramp_rate = 40

    def set_pos(self, pos):
        self.odrv1.axis0.controller.input_pos = pos[0]
        self.odrv1.axis1.controller.input_pos = pos[1]
    def set_vel(self, vel):
        self.odrv1.axis0.controller.input_vel = vel[0]
        self.odrv1.axis1.controller.input_vel = vel[1]
    def read_posvel(self):
        try:
            axis0_pos = self.odrv1.axis0.encoder.pos_estimate
            axis1_pos = self.odrv1.axis1.encoder.pos_estimate
            axis0_vel = self.odrv1.axis0.encoder.vel_estimate
            axis1_vel = self.odrv1.axis1.encoder.vel_estimate
            return [[axis0_pos, axis1_pos], [axis0_vel, axis1_vel]]
        except Exception as e:
            self.get_logger().warn(f"error : {e}")

    
    def read_error(self):
        axis_error0 = self.odrv1.axis0.error
        axis_error1 = self.odrv1.axis1.error
        motor_error0 = self.odrv1.axis0.motor.error
        motor_error1 = self.odrv1.axis1.motor.error
        controller_error0 = self.odrv1.axis0.controller.error
        controller_error1 = self.odrv1.axis1.controller.error
        encoder_error0 = self.odrv1.axis0.encoder.error
        encoder_error1 = self.odrv1.axis1.encoder.error


        Errors = {'axis_error0' : axis_error0,
                  'axis_error1' : axis_error1,
                  'motor_error0' : motor_error0,
                  'motor_error1' : motor_error1,
                  'controller_error0' : controller_error0,
                  'controller_error1' : controller_error1,
                  'encoder_error0' : encoder_error0,
                  'encoder_error1' : encoder_error1}
        for i in Errors.keys():
            if Errors[i] != 0:
                self.get_logger().fatal(f"{i} : {Errors[i]}")





def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    odrive_controller = Odrive_ep()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    odrive_controller.get_logger().info(
        'Odrive controller Started with threads='+str(num_threads))
    
    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(odrive_controller)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        odrive_controller.destroy_node()

    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
