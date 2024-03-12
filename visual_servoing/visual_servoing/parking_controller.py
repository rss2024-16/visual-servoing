#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)
        
        self.MAX_TURN = .34

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.get_logger().info("Parking Controller Initialized")


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        kx = 1
        ky = 1
        ka = 1
        x_des = 0
        y_des = 0
        angle_des = 0

        kp = 1/3

        dist = ( self.relative_x**2 + self.relative_y**2 ) ** (1/2)
        angle = np.arctan2(self.relative_y,self.relative_x)
        eps = 0.1

        if abs(dist - self.parking_distance) < eps and abs(angle-angle_des) < eps:
            speed = 0.0
            turning_angle = 0.0
        else:
            # Px = -kx * (x_des-self.relative_x)
            # Py = -ky * (y_des-self.relative_y)

            P = self.parking_distance - dist

            Pangle = -ka * (angle_des - angle)

            turning_angle = Pangle
            speed = -kp * P

            if P < 0 and self.relative_x < 0:
                speed = -speed
                turning_angle = -turning_angle

            if abs(speed) < 1:
                speed = float(1) if speed > 0 else float(-1)

            self.get_logger().info(str(speed)+','+str(turning_angle)+','+str(self.relative_x))
            # self.get_logger().info(str(dist))

            # # if dist < circle_radius and Pangle > 0.5:
            # #     self.get_logger().info('driving back')
            # #     speed = -speed
            # if self.relative_x < 0:
            #     speed = -speed
            #     turning_angle = -turning_angle



        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = turning_angle

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    try:
        rclpy.spin(pc)
    except KeyboardInterrupt:
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        pc.drive_pub.publish(msg)
    rclpy.shutdown()

if __name__ == '__main__':
    main()