import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math

class OdometryPublisher(Node):

    '''
    This is the python translation of https://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom/
    It publishes a fake odom frame
    '''

    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.1
        self.vy = -0.1
        self.vth = 0.1

        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(1.0, self.update_odometry)

    def update_odometry(self):
        self.current_time = self.get_clock().now()

        dt = (self.current_time - self.last_time).nanoseconds / 1e9

        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        quaternion = self.get_quaternion(self.th)

        transform = TransformStamped()
        transform.header.stamp = self.current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion

        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        self.last_time = self.current_time

    def get_quaternion(self, th):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(th / 2)
        quaternion.w = math.cos(th / 2)
        return quaternion


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
