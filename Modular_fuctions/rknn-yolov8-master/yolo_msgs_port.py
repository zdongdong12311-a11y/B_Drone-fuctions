#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from vision_msgs.msg import Detection2DArray

def callback(msg):
    for det in msg.detections:
        if len(det.results) == 0:
            continue

        class_id = det.results[0].id
        x = det.bbox.center.x
        y = det.bbox.center.y

        rospy.loginfo("id: %d, center_x: %.1f, center_y: %.1f", class_id, x, y)

def main():
    rospy.init_node("read_yolo_result")
    rospy.Subscriber("/yolo_msgs", Detection2DArray, callback, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()