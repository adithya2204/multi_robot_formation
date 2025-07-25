#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PentagonFormationNode:
    def __init__(self, side=0.3, tolerance=0.08):
        self.n = 5
        self.side = side
        # Compute radius for regular pentagon
        self.radius = side / (2 * math.sin(math.pi / self.n))
        self.tolerance = tolerance

        self.robot_names = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3', 'tb3_4']
        self.goal_pubs = [rospy.Publisher(f'/{name}/move_base_simple/goal', PoseStamped, queue_size=1)
                          for name in self.robot_names]
        self.odom_subs = []
        self.current_positions = [None] * self.n
        self.goals_sent = False
        self.swap_sent = False
        self.pentagon_points = []
        self.arrived = [False] * self.n

        for i, name in enumerate(self.robot_names):
            sub = rospy.Subscriber(f'/{name}/odom', Odometry, self._odom_callback_factory(i))
            self.odom_subs.append(sub)

        rospy.Subscriber('/pentagon_center_goal', PoseStamped, self.center_goal_cb)

        rospy.loginfo('Pentagon formation node initialized for 5 robots.')

    def _odom_callback_factory(self, idx):
        def callback(msg):
            self.current_positions[idx] = msg
        return callback

    def center_goal_cb(self, msg):
        x_c, y_c = msg.pose.position.x, msg.pose.position.y
        r = self.radius
        self.pentagon_points = []
        for i in range(self.n):
            angle = 2 * math.pi * i / self.n
            x = x_c + r * math.cos(angle)
            y = y_c + r * math.sin(angle)
            self.pentagon_points.append((x, y))
        self.arrived = [False] * self.n
        self.swap_sent = False
        self._send_pentagon_goals()
        self.goals_sent = True
        rospy.loginfo(f"Sent pentagon goals to {self.n} robots.")

    def _send_pentagon_goals(self):
        for i in range(self.n):
            self._publish_goal(i, self.pentagon_points[i])

    def _publish_goal(self, idx, point):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = point[0]
        goal.pose.position.y = point[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pubs[idx].publish(goal)
        rospy.loginfo(f"Goal for {self.robot_names[idx]}: x={point[0]:.3f}, y={point[1]:.3f}")

    def all_arrived(self):
        if not self.pentagon_points or any(pos is None for pos in self.current_positions):
            return False
        arrived = []
        for i in range(self.n):
            pos = self.current_positions[i].pose.pose
            x, y = pos.position.x, pos.position.y
            goal_x, goal_y = self.pentagon_points[i]
            dist = math.hypot(goal_x - x, goal_y - y)
            arrived.append(dist < self.tolerance)
        self.arrived = arrived
        return all(arrived)

    def all_swapped(self):
        swap_indices = [(i + 1) % self.n for i in range(self.n)]
        arrived = []
        for i in range(self.n):
            pos = self.current_positions[i].pose.pose
            x, y = pos.position.x, pos.position.y
            goal_x, goal_y = self.pentagon_points[swap_indices[i]]
            dist = math.hypot(goal_x - x, goal_y - y)
            arrived.append(dist < self.tolerance)
        return all(arrived)

    def send_swap_goals(self):
        swap_indices = [(i + 1) % self.n for i in range(self.n)]
        for i in range(self.n):
            self._publish_goal(i, self.pentagon_points[swap_indices[i]])
        self.swap_sent = True
        rospy.loginfo('Sent swap goals (neighbor exchange) to all robots!')

    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.goals_sent and not self.swap_sent and self.all_arrived():
                rospy.loginfo('All robots arrived at pentagon vertices. Swapping...')
                rospy.sleep(1.0)
                self.send_swap_goals()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pentagon_formation_node')
    node = PentagonFormationNode(side=0.3)
    node.spin()
