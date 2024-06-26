#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

import math

import time

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

        self.parking_distance = .1 # meters; try playing with this number!

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)
        self.turn_angle_pub = self.create_publisher(Float32,'/turning_angle',10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.look_ahead_pub = self.create_publisher(Float32,'/updated_look_ahead',10)
        
        self.MAX_TURN = .2

        self.relative_x = 0
        self.relative_y = 0
        self.dist = 0

        self.BASE_LENGTH = 0.3

        self.previous_angles = []

        self.distance_check = False

        # self.get_logger().info("Parking Controller Initialized")

    def error(self,actual,desired):
        return (desired - actual)
    
    def circle_intersection(self, line_slope, line_intercept, circle_radius):
        # Coefficients of the quadratic equation
        a = line_slope**2 + 1
        b = 2 * line_slope * line_intercept
        c = line_intercept**2 - circle_radius**2

        # Solve the quadratic equation for x
        x_solutions = np.roots([a, b, c])

        # Find corresponding y values using the line equation
        intersection_points = [(float(x_val), float(line_slope * x_val + line_intercept)) for x_val in x_solutions]
        
        # +X forward, +Y left, -Y right

        if intersection_points[0][0] > 0 and intersection_points[1][0] < 0:
            return intersection_points[0]
        elif intersection_points[0][0] < 0 and intersection_points[1][0] > 0:
            return intersection_points[1]
        elif intersection_points[0][1] > 0 and self.SIDE == -1 or intersection_points[0][1] < 0 and self.SIDE == 1:
            return intersection_points[0]
        else:
            return intersection_points[1]


    def relative_cone_callback(self, msg):
        """
        Given the cone location msg, in terms of real life coordinates x, y, publish a drive command
        """
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        # pubstr = f"Relative x: {self.relative_x}\nRelative y: {self.relative_y}\n---------------"
        # self.get_logger().info(pubstr)
        drive_cmd = AckermannDriveStamped()

        angle_des = 0

        kp = 1/3 #Kp value for speed

        self.dist = ( self.relative_x**2 + self.relative_y**2 ) ** (1/2)
        look_ahead = 4.0

        angle = np.arctan2(self.relative_y,self.relative_x)

        dist_eps = 0.15
        angle_eps = 0.1

        if abs(self.dist - self.parking_distance) < dist_eps and abs(angle-angle_des) < angle_eps:
            #goal check
            speed = 0.0
            turning_angle = 0.0
        else:
            slope = self.relative_y/self.relative_x
            intersect = self.circle_intersection(slope, 0, look_ahead)
            turning_angle = math.atan2(2 * self.BASE_LENGTH * intersect[1], look_ahead**2)

            turning_angle += -0.03
            speed = 4.0

        self.get_logger().info(f'{turning_angle}')
                    
        if abs(turning_angle) > self.MAX_TURN:
            turning_angle = self.MAX_TURN if turning_angle > 0 else -self.MAX_TURN


        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = turning_angle

        self.drive_pub.publish(drive_cmd)
        # self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        x_des = self.parking_distance
        y_des = 0.0

        x_error = self.error(self.relative_x,x_des)
        y_error = self.error(self.relative_y,y_des)
        dist_error = self.error(self.dist,self.parking_distance)

        error_msg.x_error = x_error
        error_msg.y_error = y_error
        error_msg.distance_error = dist_error
        
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