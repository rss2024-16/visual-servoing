#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import time

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

        self.parking_distance = 0.75 # meters; try playing with this number!

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)
        
        self.MAX_TURN = .34

        self.get_logger().info(str(self.parking_distance))
        self.relative_x = 0
        self.relative_y = 0
        self.dist = 0

        self.distance_check = False

        self.get_logger().info("Parking Controller Initialized")

    def error(self,actual,desired):
        return (desired - actual)/actual


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        ka = 1
        angle_des = 0

        kp = 1/3

        self.dist = ( self.relative_x**2 + self.relative_y**2 ) ** (1/2)
        angle = np.arctan2(self.relative_y,self.relative_x)

        eps = 0.1

        if abs(self.dist - self.parking_distance) < eps and abs(angle-angle_des) < eps:
            #goal check
            speed = 0.0
            turning_angle = 0.0
        else:
            P = self.parking_distance - self.dist
            Pangle = -ka * (angle_des - angle)

            if P>0 or self.distance_check:
                #if P>0, we are too close to the cone and may need to back up
                #if self.distance_check is true, we are already backing up and need to check distance
                
                if self.distance_check:
                    
                    if self.dist < 1.5*self.parking_distance:
                        #if we have not backed up far enough, continue
                        speed = float(-1)
                        turning_angle = -Pangle

                    else:
                        #if we have backed up far enough, set distance check to false
                        #and resume regular control
                        self.distance_check = False
                        speed = 0.0
                        turning_angle = 0.0
                
                else:
                    #if we are too close but have not started backing up
                    #set distance check to true to enable the backup sequence
                    self.distance_check = True
                    speed = 0.0
                    turning_angle = 0.0


            else:
                turning_angle = Pangle
                speed = -kp * P

                if P < 0 and self.relative_x < 0:
                    #if the cone is behind us, go backward instead of forward
                    speed = -speed
                    turning_angle = -turning_angle

                if abs(speed) < 1:
                    #this statement may need tuning on actual robot depending on the
                    #motor issues that we faced
                    speed = float(1) if speed > 0 else float(-1)
                    
        if abs(turning_angle) > self.MAX_TURN:
            turning_angle = self.MAX_TURN if turning_angle > 0 else -self.MAX_TURN



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