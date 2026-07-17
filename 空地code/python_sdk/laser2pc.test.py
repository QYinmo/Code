#!/usr/bin/env python

import struct
import time

import message_filters
import rclpy
from geometry_msgs.msg import PoseStamped
# from laser_geometry import LaserProjection
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

"""

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
	#self.pathPub = rospy.Publisher("/trajectory",Path, queue_size=10)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
"""


def callback(data, msg):
    xz = msg.pose.pose.position.z
    xx = msg.pose.pose.position.x
    xy = msg.pose.pose.position.y
    # cloud_out = laserProj.projectLaser(data)
    isinstance(cloud_out, pc2)
    # gen = point_cloud2.read_points(cloud_out, field_names=("x", "y", "z"), skip_nans=True)
    # time.sleep(1)
    points = []
    for p in gen:
        x = p[0]
        y = p[1]
        z = p[2]
        # x=x+xx
        # z=z+xz
        # y=y+xy
        r = 255
        g = 0
        b = 0
        a = 255
        rgb = struct.unpack("I", struct.pack("BBBB", b, g, r, a))[0]
        pt = [x, y, z, rgb]
        points.append(pt)
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        # PointField('rgb', 12, PointField.UINT32, 1),
        PointField("rgba", 12, PointField.UINT32, 1),
    ]
    header = Header()
    header.frame_id = "base_laser"
    rusult = point_cloud2.create_cloud(header, fields, points)
    pcPub.publish(rusult)

    this_pose_stamped = PoseStamped()
    this_pose_stamped.pose.position.x = xx
    this_pose_stamped.pose.position.y = xy
    this_pose_stamped.pose.position.z = xz
    # print " x : %.3f  y: %.3f  z: %.3f" %(xx,xy,xz)
    this_pose_stamped.pose.orientation.x = msg.pose.pose.orientation.x
    this_pose_stamped.pose.orientation.y = msg.pose.pose.orientation.y
    this_pose_stamped.pose.orientation.z = msg.pose.pose.orientation.z
    this_pose_stamped.pose.orientation.w = msg.pose.pose.orientation.w
    this_pose_stamped.header = header
    odempub.publish(this_pose_stamped)
    #      print type(gen)

    # rr=msg.pose.pose.orientation
    
if __name__ == "__main__":
    xz = 1
    rclpy.init()
    node = Node("laser2PointCloud")
    # laserProj = LaserProjection()
    pcPub = node.create_publisher(pc2, "/laserPointCloud", 10)
    odempub = node.create_publisher(PoseStamped, "/trajectory", 10)
    t1 = message_filters.Subscriber("/scan", LaserScan)
    t2 = message_filters.Subscriber("/camera/odom/sample", Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)

    # self.pathPub = rospy.Publisher("/trajectory",Path, queue_size=10)
    # odemsub=rospy.Subscriber("",,odomcallback)
    # laserSub = rospy.Subscriber("/scan", , laserCallback)
    # l2pc = Laser2PC()
    rclpy.spin()
