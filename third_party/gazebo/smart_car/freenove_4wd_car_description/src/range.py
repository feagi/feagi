#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('controller_subscriber')
        """
        # Single scan from a planar laser range-finder
        #
        # If you have another ranging device with different behavior (e.g. a sonar
        # array), please find or create a different message, since applications
        # will make fairly laser-specific assumptions about this data

        std_msgs/Header header # timestamp in the header is the acquisition time of
                                     # the first ray in the scan.
                                     #
                                     # in frame frame_id, angles are measured around
                                     # the positive Z axis (counterclockwise, if Z is up)
                                     # with zero angle being forward along the x axis

        float32 angle_min            # start angle of the scan [rad]
        float32 angle_max            # end angle of the scan [rad]
        float32 angle_increment      # angular distance between measurements [rad]

        float32 time_increment       # time between measurements [seconds] - if your scanner
                                     # is moving, this will be used in interpolating position
                                     # of 3d points
        float32 scan_time            # time between scans [seconds]

        float32 range_min            # minimum range value [m]
        float32 range_max            # maximum range value [m]

        float32[] ranges             # range data [m]
                                     # (Note: values < range_min or > range_max should be discarded)
        float32[] intensities        # intensity data [device-specific units].  If your
                                     # device does not provide intensities, please leave
                                     # the array empty.

        """

        self.subscription = self.create_subscription(
        LaserScan,
        'ultrasonic',
        self.listener_callback,
        qos_profile = qos_profile_sensor_data)

        """
        # This message contains an uncompressed image
        # (0, 0) is at top-left corner of image

        std_msgs/Header header # Header timestamp should be acquisition time of image
                                     # Header frame_id should be optical frame of camera
                                     # origin of frame should be optical center of cameara
                                     # +x should point to the right in the image
                                     # +y should point down in the image
                                     # +z should point into to plane of the image
                                     # If the frame_id here and the frame_id of the CameraInfo
                                     # message associated with the image conflict
                                     # the behavior is undefined

        uint32 height                # image height, that is, number of rows
        uint32 width                 # image width, that is, number of columns

        # The legal values for encoding are in file src/image_encodings.cpp
        # If you want to standardize a new string format, join
        # ros-users@lists.ros.org and send an email proposing a new encoding.

        string encoding       # Encoding of pixels -- channel meaning, ordering, size
                              # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

        uint8 is_bigendian    # is this data bigendian?
        uint32 step           # Full row length in bytes
        uint8[] data          # actual matrix data, size is (step * rows)
        """

        # self.subscription = self.create_subscription(
        #     Image,
        #     '/depth_camera_right',
        #     self.listener_callback,
        #     qos_profile=qos_profile_sensor_data)
        # self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("distance: {}".format(msg.ranges[1]))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

