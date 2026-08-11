import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class TurtleSquare(Node):

    def __init__(self):
        super().__init__('turtle_square')

        # Publisher
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # Turtle pose
        self.pose = None

        # Square size
        self.side_length = 2.0

        # Square state: 0 = move, 1 = turn
        self.state = 0

        # Current side (0 to 3)
        self.side = 0

        # Starting point
        self.start_x = None
        self.start_y = None
        self.start_theta = None

        # Targets list
        self.targets = []

        self.target_x = None
        self.target_y = None
        self.target_theta = None

        # Timer
        self.timer = self.create_timer(
            0.01,
            self.timer_callback
        )

    def pose_callback(self, msg):
        self.pose = msg

        # Initialize targets once based on sequential corner coordinates
        if self.start_x is None:
            self.start_x = msg.x
            self.start_y = msg.y
            self.start_theta = msg.theta

            curr_x = self.start_x
            curr_y = self.start_y

            # حساب رؤوس المربع الأربعة بالتسلسل
            for i in range(4):
                angle = self.start_theta + i * (math.pi / 2)
                curr_x += self.side_length * math.cos(angle)
                curr_y += self.side_length * math.sin(angle)
                self.targets.append((curr_x, curr_y))

            # Target for the first side
            self.target_x = self.targets[0][0]
            self.target_y = self.targets[0][1]
            self.target_theta = self.start_theta

    def normalize_angle(self, angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def timer_callback(self):
        if self.pose is None or self.start_x is None:
            return

        msg = Twist()

        # ==========================================
        # MOVE TO CORNER
        # ==========================================
        if self.state == 0:
            dx = self.target_x - self.pose.x
            dy = self.target_y - self.pose.y

            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance < 0.02:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.state = 1
            else:
                # حساب اتجاه الحركة نحو الركن المستهدف
                desired_angle = math.atan2(dy, dx)
                angle_error = self.normalize_angle(desired_angle - self.pose.theta)

                msg.linear.x = 1.0 if distance > 0.3 else 0.3
                msg.angular.z = max(-0.3, min(0.3, 1.5 * angle_error))

        # ==========================================
        # TURN 90 DEGREES
        # ==========================================
        elif self.state == 1:
            next_theta = self.normalize_angle(
                self.start_theta + (self.side + 1) * (math.pi / 2)
            )

            angle_error = self.normalize_angle(next_theta - self.pose.theta)

            if abs(angle_error) < 0.01:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

                self.side = (self.side + 1) % 4

                # تحديد الركن المستهدف للضلع الجديد
                self.target_x = self.targets[self.side][0]
                self.target_y = self.targets[self.side][1]
                self.target_theta = next_theta

                self.state = 0
            else:
                msg.linear.x = 0.0
                msg.angular.z = max(-1.0, min(1.0, 2.0 * angle_error))

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
