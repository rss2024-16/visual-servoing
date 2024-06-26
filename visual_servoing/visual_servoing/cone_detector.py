#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from vs_msgs.msg import ConeLocationPixel, ParkingError

from std_msgs.msg import Float32

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = False
        self.look_ahead_v = None

        # Subscribe to ZED camera RGB frames
        self.cone_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)

        self.dist_pub = self.create_publisher(Float32,'/look_ahead',10)

        self.look_ahead_v_sub = self.create_subscription(Float32,'/look_ahead_v',self.vSub,10)

        self.cam_look_ahead = self.create_subscription(Float32,'/updated_look_ahead',self.laSub,10)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.crop_scale = 0.12

        self.look_ahead = 1.5
        self.look_ahead_v = 168.7

        self.data = {'x':[],'y':[],'dist':[],'turn_angle':[]}

        self.stats_sub = self.create_subscription(ParkingError,'/parking_error',self.stats,10)

        self.get_logger().info("Cone Detector Initialized")

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        #344,178.5
        height = image.shape[1]

        # self.get_logger().info('Look ahead: '+str(self.look_ahead))

        msg = Float32()
        msg.data = self.look_ahead
        self.dist_pub.publish(msg)

        v = self.look_ahead_v
        
        if v is None:
            v = 168.7

        lower = int((1-self.crop_scale)*v)
        upper = int((1+self.crop_scale)*v)
        if upper > height:
            upper = height
        image_copy = np.copy(image)
        image_copy[0:lower,:,:] = 0
        image_copy[upper:,:,:] = 0
        # image_copy[:,0:100,:] = 0
        # image_copy[:,560:,:] = 0

        x, y, w, h, img = cd_color_segmentation(image_copy,None)

        if x is not None:
            #send the bottom center pixel
            center_pixel = ConeLocationPixel()
            center_pixel.u = float(x + w/2)
            center_pixel.v = float(y + h/2)

            self.cone_pub.publish(center_pixel)

            debug_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.debug_pub.publish(debug_msg)
            self.crop_scale = 0.2
        else:
            # if self.crop_scale > 4:
            #     self.crop_scale = 0.1
            # else:
            # self.crop_scale += 0.1
            pass

    def vSub(self,msg):
        if msg.data is not None:
            self.look_ahead_v = msg.data

    def laSub(self,msg):
        if msg.data is not None:
            self.look_ahead = msg.data
    
    def stats(self,stats):
        self.data['x'].append(stats.x_error)
        self.data['y'].append(stats.y_error)
        self.data['dist'].append(stats.distance_error)

def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    try:
        rclpy.spin(cone_detector)
    except KeyboardInterrupt:
        pass
    np.save('stats3',cone_detector.data)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
