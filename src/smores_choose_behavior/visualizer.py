#!/usr/bin/env python

import threading

import rospy
from visualization_msgs.msg import Marker, MarkerArray

class Visualizer(object):
    def __init__(self):
        self._current_pub_path = []
        self.run = False
        self.t = None

        self.t = threading.Thread(target=self._startPathPub)
        self.t.setDaemon(True)
        self.run = True
        self.t.start()

    def stop(self):
        self.run = False
        self.t.join()

    def _startPathPub(self):
        path_marker_pub = rospy.Publisher('PathMarker', MarkerArray, queue_size=10)
        rate = rospy.Rate(1) # 10hz
        while self.run and not rospy.is_shutdown():
            id = 1
            m_array = MarkerArray()
            for pt in self._current_pub_path:
                m = Marker()
                m.header.frame_id = "camera_link";
                m.header.stamp = rospy.Time();
                m.ns = "my_namespace";
                m.id = id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = pt
                m.scale.x = 0.05
                m.scale.y = 0.05
                m.scale.z = 0.05
                m.color.r = 0.5
                m.color.a = 1.0
                m_array.markers.append(m)
                id += 1

            path_marker_pub.publish(m_array)
            rate.sleep()

    def setPubPath(self, path):
        self._current_pub_path = path
