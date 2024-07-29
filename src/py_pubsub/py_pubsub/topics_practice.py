import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.timer_period = 1.0  # seconds
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.count_ = 5  # Countdown starting from 5
        self.drawing_spiral = False
        self.spiral_angle = 30.0  # Angle must be a float
        self.spiral_length = 1.0  # Length must be a float (MODIFICADO)
        self.current_pose = None

    def timer_callback(self):
        if self.count_ > 0:
            self.get_logger().info('Countdown: %d' % self.count_)
            self.count_ -= 1
        elif self.count_ == 0 and not self.drawing_spiral:
            self.get_logger().info('Drawing spiral')
            self.drawing_spiral = True
            self.timer_.cancel()  # Stop the countdown timer
            self.timer_ = self.create_timer(0.1, self.draw_spiral)  # Start drawing spiral

    def pose_callback(self, msg):
        self.current_pose = msg

    def draw_spiral(self):
        if self.current_pose is None:
            return

        twist = Twist()
        twist.linear.x = self.spiral_length
        twist.angular.z = self.spiral_angle
        self.publisher_.publish(twist)
        self.spiral_length += 10.1  # Increase the length for the spiral effect (MODIFICADO)

        # Check boundaries
        if (self.current_pose.x < 1.0 or self.current_pose.x > 10.0 or
                self.current_pose.y < 1.0 or self.current_pose.y > 10.0):
            self.get_logger().info('Going straight')
            self.drawing_spiral = False
            self.timer_.cancel()  # Stop the spiral timer
            self.timer_ = self.create_timer(0.1, self.go_straight)  # Start going straight

    def go_straight(self):
        twist = Twist()
        twist.linear.x = 2.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

        if (self.current_pose.x < 1.0 or self.current_pose.x > 10.0 or
                self.current_pose.y < 1.0 or self.current_pose.y > 10.0):
            twist.linear.x = 2.0  # Turn around if hitting the boundary (MODIFICADO)
            self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
