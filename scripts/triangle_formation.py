#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped

class TriangleFormationNode:
    def __init__(self, radius=0.3):
        self.radius = radius
        self.pubs = [
            rospy.Publisher('/tb3_0/move_base_simple/goal', PoseStamped, queue_size=1),
            rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=1),
            rospy.Publisher('/tb3_2/move_base_simple/goal', PoseStamped, queue_size=1)
        ]
        # Subscribe to the triangle center goal published from RViz
        rospy.Subscriber('/triangle_center_goal', PoseStamped, self.goal_callback)
        rospy.loginfo("Triangle formation node started, waiting for center goal on /triangle_center_goal")

    def goal_callback(self, msg):
        x_c = msg.pose.position.x
        y_c = msg.pose.position.y
        theta = 0.0

        # Triangle vertices relative to the center
        offsets = [
            (self.radius, 0),  # First robot: right of center
            (-self.radius/2, self.radius * math.sqrt(3)/2),   # Second robot: upper left
            (-self.radius/2, -self.radius * math.sqrt(3)/2)   # Third robot: lower left
        ]

        for i, (dx, dy) in enumerate(offsets):
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x_c + dx
            goal.pose.position.y = y_c + dy
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            self.pubs[i].publish(goal)
            rospy.loginfo("Published goal for tb3_%d: x=%.3f, y=%.3f" % (i, goal.pose.position.x, goal.pose.position.y))

if __name__ == '__main__':
    rospy.init_node('triangle_formation_node')
    TriangleFormationNode(radius=0.3)
    rospy.spin()
