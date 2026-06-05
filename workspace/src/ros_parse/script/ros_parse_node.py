#!/usr/bin/env python3

from PySide6 import QtOpenGLWidgets
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import tf2_ros
from tf.transformations import quaternion_matrix



from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QWidget

import random
from QOpenGL3DViewer import QOpenGL3DViewer, Cube, GroupPoints
import numpy as np
import sys
from ui_form import Ui_Q3DViewer
import pandas as pd


class Q3DViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_Q3DViewer()
        self.ui.setupUi(self)

        ###ADD In de componet
        self.opengl_widget = QOpenGL3DViewer(FPS=60, parent=self)
        self.ui.viewer.addWidget(self.opengl_widget)
        ###

        rospy.init_node("bag_parser")

        topics = rospy.get_published_topics()
        print("Published topics:", topics)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/hsrb/base_scan", LaserScan, self.laser_cb)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, self.image_cb)
        rospy.Subscriber("/dynamic_obstacle_map", OccupancyGrid, self.map_cb)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, self.pointcloud_cb, queue_size=1)

        data = pd.read_json("/root/experiments/01/01_poses/1_occupancy_xy_points.json").to_numpy()
        self.poses = pd.read_csv("/root/experiments/01/01_poses/1_robot_and_participants.csv")

        self.ocupancy_points = GroupPoints(points=np.hstack((data, np.zeros((data.shape[0],1)))), color=np.array((1,1,0)))
        self.lidar_points = GroupPoints(np.array([]))
        self.camera_points = GroupPoints(np.array([]))
        self.cubes = np.array([])

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_view)
        self.timer.start(100)

        self.timer_poses = QTimer(self)
        self.timer_poses.timeout.connect(self.poses_update)
        self.timer_poses.start(100)
        
    def __del__(self):
         self.timer.stop()
         self.timer_poses.stop()

    def laser_cb(self, msg):
        transform = self.tf_buffer.lookup_transform(
            "map",                         
            msg.header.frame_id,           
            msg.header.stamp,              
            rospy.Duration(0.5)
        )
        t = transform.transform.translation
        q = transform.transform.rotation

        T = quaternion_matrix([
            q.x,
            q.y,
            q.z,
            q.w
        ])

        T[0,3] = t.x
        T[1,3] = t.y
        T[2,3] = t.z

        ranges = np.array(msg.ranges, dtype=np.float32)

        valid = np.isfinite(ranges)

        ranges = ranges[valid]

        angles = (
            msg.angle_min +
            np.arange(len(msg.ranges)) * msg.angle_increment
        )[valid]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        points = np.column_stack((x, y, z))

        R = T[:3,:3]
        t = T[:3,3]

        points_map = points @ R.T
        points_map += t

        self.lidar_points = GroupPoints(
            points=points_map,
            color=np.array((1.0, 0.0, 0.0))
        )

    def pointcloud_cb(self, msg):
        transform = self.tf_buffer.lookup_transform(
            "map",                         
            msg.header.frame_id,           
            msg.header.stamp,              
            rospy.Duration(0.5)
        )
        t = transform.transform.translation
        q = transform.transform.rotation

        T = quaternion_matrix([
            q.x,
            q.y,
            q.z,
            q.w
        ])

        T[0,3] = t.x
        T[1,3] = t.y
        T[2,3] = t.z

        cloud_points = np.array(
            list(
                pc2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True
                )
            ),
            dtype=np.float32
        )
        
        R = T[:3,:3]
        t = T[:3,3]

        # cloud_points = np.ndarray(
        #     shape=(len(msg.data)//16, 3),
        #     dtype=np.float32,
        #     buffer=msg.data,
        #     strides=(16, 4)
        # )
        


        points = cloud_points[::100] @ R.T
        points += t

        self.camera_points = GroupPoints(
            points=points,
            color=np.array((0.0, 1.0, 0.0))
        )

    def image_cb(self, msg):
        # rospy.loginfo(f"Image: {msg.width}x{msg.height}")
        pass

    def map_cb(self,  msg):
        pass# rospy.loginfo(f"Map: {msg.info.width}x{msg.info.height}")


    def update_view(self):
        """Generar puntos aleatorios para el renderizado."""
        rospy.loginfo(f"lidar")

        self.opengl_widget.set_points(
            np.array([self.ocupancy_points, self.lidar_points, self.camera_points])
        )

    def poses_update(self):
        rospy.loginfo(f"Poses")

        now_ns = rospy.Time.now().to_nsec()-15058455523
        print(now_ns)

        idx = (self.poses["timestamp"] - now_ns).abs().idxmin()

        row = self.poses.loc[idx].to_numpy()
        #roboto
        cubes = np.array((
            Cube(
                position=np.array((row[1], row[2], 0)),
                size=np.array((0.5,0.5,1)),
                rotation=np.array((0, 0, row[3]*180/3.1416)),
                color=np.array((0.5,0.5,0.5)))
            ))
        #personas
        for i in range(4,len(row),2):
            cube = Cube(
                position=np.array((row[i], row[i+1], 0)),
                size=np.array((0.5,0.5,1)),
                rotation=np.array((0,0,0)),
                color=np.array((0.5,0,0.5))
            )
            cubes = np.append(cubes, cube)


        self.opengl_widget.set_cube(cubes)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = Q3DViewer()
    widget.show()
    sys.exit(app.exec())