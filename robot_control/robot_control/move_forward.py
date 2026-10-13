import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')

        self.declare_parameter('speed', 0.3)
        self.declare_parameter('duration', 2.0)
        self.speed = self.get_parameter('speed').value
        self.duration = self.get_parameter('duration').value

        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

        self.start_time = self.get_clock().now()
        self.done = False

        # publish at 20Hz so the motor_control safety watchdog (0.15s) stays satisfied
        self.timer = self.create_timer(0.05, self.publish_command)
        self.get_logger().info(
            f'Moving forward at speed={self.speed:.2f} for {self.duration:.1f}s')

    def send(self, left, right):
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.publisher.publish(msg)

    def publish_command(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.duration:
            self.send(self.speed, self.speed)
        else:
            self.send(0.0, 0.0)
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send(0.0, 0.0)
            node.get_logger().info('Stopped')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
