#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class SquareFormationNode:
    def __init__(self, side=0.5, tolerance=0.15, wait_time=30.0):
        self.side = side
        self.radius = side / math.sqrt(2)
        self.tolerance = tolerance
        self.wait_time = wait_time

        self.robot_names = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
        self.goal_pubs = []
        self.position_subs = []
        self.current_positions = [None] * 4
        self.goals_sent = False
        self.square_points = []
        self.active_targets = []
        self.next_swap_time = None

    def _pose_callback_factory(self, idx):
        def callback(msg):
            self.current_positions[idx] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        return callback

    def setup(self):
        for i, name in enumerate(self.robot_names):
            pub = rospy.Publisher(f'/{name}/move_base_simple/goal', PoseStamped, queue_size=1)
            self.goal_pubs.append(pub)
            sub = rospy.Subscriber(f'/{name}/amcl_pose', PoseWithCovarianceStamped, self._pose_callback_factory(i))
            self.position_subs.append(sub)
        rospy.Subscriber('/square_center_goal', PoseStamped, self.center_goal_cb)
        rospy.loginfo(f"Square formation node initialized for: {', '.join(self.robot_names)}")

    def center_goal_cb(self, msg):
        x_c, y_c = msg.pose.position.x, msg.pose.position.y
        r = self.radius
        # Corner order: 0→1→2→3 (clockwise)
        self.square_points = [
            (x_c + r, y_c + r),
            (x_c - r, y_c + r),
            (x_c - r, y_c - r),
            (x_c + r, y_c - r)
        ]
        self.active_targets = list(self.square_points)
        self.goals_sent = True
        self.next_swap_time = None
        self._send_square_goals()
        rospy.loginfo('Sent initial square goals.')

    def _send_square_goals(self):
        for i in range(4):
            self._publish_goal(i, self.active_targets[i])

    def _publish_goal(self, idx, point):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = point
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pubs[idx].publish(goal)
        rospy.loginfo(f"Goal for {self.robot_names[idx]}: x={point[0]:.2f}, y={point[1]:.2f}")

    def all_arrived(self):
        if not self.active_targets or any(pos is None for pos in self.current_positions):
            return False
        for i, (goal_x, goal_y) in enumerate(self.active_targets):
            x, y = self.current_positions[i]
            if math.hypot(goal_x - x, goal_y - y) > self.tolerance:
                return False
        return True

    def rotate_targets(self):
        # Rotate vertices so each robot moves clockwise to the next one
        self.active_targets = self.active_targets[1:] + [self.active_targets[0]]

    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.goals_sent:
                if self.all_arrived():
                    if self.next_swap_time is None:
                        rospy.loginfo(f'All robots reached target. Waiting {self.wait_time:.1f} seconds before swapping...')
                        self.next_swap_time = rospy.Time.now() + rospy.Duration(self.wait_time)
                    elif rospy.Time.now() >= self.next_swap_time:
                        self.rotate_targets()
                        self._send_square_goals()
                        rospy.loginfo('Sent swap goals along square edge to all robots.')
                        self.next_swap_time = None  # Wait for arrival again, repeat
                else:
                    self.next_swap_time = None  # Reset if any robot hasn't arrived
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('square_formation_node')
    node = SquareFormationNode(side=0.5, tolerance=0.15, wait_time=30.0)
    node.setup()
    node.spin()
