#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from vs_msgs.msg import ConeLocation, ConeLocationPixel
from geometry_msgs.msg import Point

from rclpy.parameter import Parameter

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[340, 213],
                   [254, 227],
                   [525, 258],
                   [346, 182]] # dummy points
######################################################

# 340, 213 -> 27, 2.5
# 254, 227 -> 20, 11
# 525, 258 -> 12, -11
# 346, 182 -> 54, 0



# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[27, 2.5],
                    [20, 11],
                    [12, -11],
                    [54, 0]] # dummy points
######################################################

METERS_PER_INCH = 0.0254

class HomographyTransformer(Node):
    def __init__(self):
        super().__init__("homography_transformer")

        self.cone_pub = self.create_publisher(ConeLocation, "/relative_cone", 10)
        self.marker_pub = self.create_publisher(Marker, "/cone_marker", 1)
        self.cone_px_sub = self.create_subscription(ConeLocationPixel, "/relative_cone_px", self.cone_detection_callback, 1)

        self.pixel_sub = self.create_subscription(ConeLocationPixel, "/zed/zed_node/rgb/image_rect_color_mouse_left", self.image_callback, 10)

        self.look_ahead_pub = self.create_publisher(Float32,'/look_ahead_v',10)
        self.look_ahead_sub = self.create_subscription(Float32,'/look_ahead',self.lookAheadCallback,10)
        self.midpoint_sub = self.create_subscription(Point,'/line_detector/midpoint',self.cone_detection_callback)
        
        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rclpy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        self.get_logger().info("Homography Transformer Initialized")

    def cone_detection_callback(self, msg):
        #Extract information from message
        u = msg.u
        v = msg.v

        #Call to main function
        x, y = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        relative_xy_msg = ConeLocation()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y

        self.cone_pub.publish(relative_xy_msg)
        self.draw_marker(x, y, 'map')


    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        self.get_logger().info('x:' + str(x) + 'y:' + str(y))
        return x, y

    def transormXyToUv(self, x, y):
        """
        x and y are in meters.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of uv pixel coordinates of the point
        on the image plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.
        """
        self.get_logger().info(str(self.h))
        homogeneous_point = np.array([[x], [y], [1]])
        uv = np.dot(np.linalg.inv(self.h), homogeneous_point)
        scaling_factor = 1.0 / uv[2, 0]
        homogeneous_uv = uv * scaling_factor
        u = homogeneous_uv[0, 0]
        v = homogeneous_uv[1, 0]
        # param = Parameter('look_ahead_v',Parameter.Type.FLOAT,v)
        # self.set_parameters([param])
        return u,v
    
    def lookAheadCallback(self,look_ahead):
        """
        x and y are in meters.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of uv pixel coordinates of the point
        on the image plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.
        """
        x,y=look_ahead.data,0.0
        homogeneous_point = np.array([[x], [y], [1]])
        uv = np.dot(np.linalg.inv(self.h), homogeneous_point)
        scaling_factor = 1.0 / uv[2, 0]
        homogeneous_uv = uv * scaling_factor
        u = homogeneous_uv[0, 0]
        v = homogeneous_uv[1, 0]
        msg = Float32()
        msg.data = v
        self.look_ahead_pub.publish(msg)

    def draw_marker(self, cone_x, cone_y, message_frame):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        self.marker_pub.publish(marker)

    def image_callback(self, msg):
        '''
        Debugging tool 
        '''
        u, v, z = msg.x, msg.y, msg.z
        x, y = self.transformUvToXy(u, v)
        self.get_logger().info('x:' + str(x) + 'y:' + str(y))
        self.draw_marker(x, y, 'map')

def main(args=None):
    rclpy.init(args=args)
    homography_transformer = HomographyTransformer()
    homography_transformer.transormXyToUv(1.5, 0.0)
    homography_transformer.transformUvToXy(355, 202)
    homography_transformer.transformUvToXy(223, 212)
    homography_transformer.transformUvToXy(418, 206)
    homography_transformer.transformUvToXy(256, 177)
    homography_transformer.transformUvToXy(466, 262)
    rclpy.spin(homography_transformer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
