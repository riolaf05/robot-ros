import rclpy
from rclpy.node import Node

from nav_msgs.msg._odometry import Odometry
from tf2_ros import TransformBroadcaster
# from nav_msgs.msg import Twist

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.theta = 0.0
        self.x_position = 0.0
        self.y_position = 0.0

        # Other unused variables and constants can be removed

    def update_odometry(self, left_distance, right_distance):
        # Implement the logic from updateOdom function here
        # This includes calculating dCenter, phi, updating theta,
        # x, y positions, velocity components, and covariance.

        # Generate quaternion from yaw (assuming zero roll and pitch)
        odom_quat = tf2_ros.transformations.quaternion_from_euler(0, 0, self.theta)

        # Create and populate the Odometry message
        current_time = self.get_clock().now()
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"

        # Set position and orientation
        odom_msg.pose.pose.position.x = self.x_position
        odom_msg.pose.pose.position.y = self.y_position
        odom_msg.pose.pose.orientation = odom_quat

        print(odom_msg)

        # Set child frame ID and twist components (implement logic)
        odom_msg.child_frame_id = "base_footprint"
        # ... (set twist based on velocity calculations)

        # Set covariance (consider using appropriate values)
        for i in range(36):
            odom_msg.pose.covariance[i] = 0.0
        odom_msg.pose.covariance[0] = 0.01
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[14] = 0.01
        odom_msg.pose.covariance[21] = 0.1
        odom_msg.pose.covariance[28] = 0.1
        odom_msg.pose.covariance[35] = 0.1

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Broadcast the transform (consider using appropriate frame names)
        self.odom_broadcaster.sendTransform(
            (self.x_position, self.y_position, 0.0),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

def main():
    rclpy.init()
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()