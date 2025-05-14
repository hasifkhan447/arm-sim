import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np
import matplotlib.pyplot as plt


class TorquePlotter(Node):
    def __init__(self):
        super().__init__('torque_plotter')
        # Data storage for each joint: list of (time, torque_magnitude)
        self.torque_data = {
            'joint_a1': [],
            'joint_a2': [],
            'joint_a3': [],
            'joint_a4': [],
            'joint_a5': []
        }
        self.start_time = None  # To normalize timestamps
        # Create subscriptions for each wrench topic
        self.subscriptions = {
            'joint_a1': self.create_subscription(
                WrenchStamped, '/joint_a1_ft_broadcaster/wrench',
                lambda msg: self.wrench_callback(msg, 'joint_a1'), 10),
            'joint_a2': self.create_subscription(
                WrenchStamped, '/joint_a2_ft_broadcaster/wrench',
                lambda msg: self.wrench_callback(msg, 'joint_a2'), 10),
            'joint_a3': self.create_subscription(
                WrenchStamped, '/joint_a3_ft_broadcaster/wrench',
                lambda msg: self.wrench_callback(msg, 'joint_a3'), 10),
            'joint_a4': self.create_subscription(
                WrenchStamped, '/joint_a4_ft_broadcaster/wrench',
                lambda msg: self.wrench_callback(msg, 'joint_a4'), 10),
            'joint_a5': self.create_subscription(
                WrenchStamped, '/joint_a5_ft_broadcaster/wrench',
                lambda msg: self.wrench_callback(msg, 'joint_a5'), 10)
        }
        # Timer to periodically plot data (every 5 seconds)
        self.plot_timer = self.create_timer(5.0, self.plot_torques)
        self.get_logger().info('TorquePlotter node started, subscribing to wrench topics')

    def wrench_callback(self, msg, joint_name):
        # Initialize start time on first message
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Calculate current time relative to start
        current_time = (msg.header.stamp.sec +
                        msg.header.stamp.nanosec * 1e-9) - self.start_time
        # Calculate torque magnitude
        torque = msg.wrench.torque
        torque_magnitude = np.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
        # Store data
        self.torque_data[joint_name].append((current_time, torque_magnitude))
        self.get_logger().debug(f'{joint_name}: Torque magnitude = {
            torque_magnitude:.3f} Nm at t = {current_time:.3f} s')

    def plot_torques(self):
        # Plot torque data for each joint
        for joint_name, data in self.torque_data.items():
            if not data:  # Skip if no data
                continue
            # Unzip time and torque lists
            times, torques = zip(*data)
            # Create plot
            plt.figure(figsize=(10, 6))
            plt.plot(times, torques, 'b-', label=f'{joint_name} Torque')
            plt.title(f'Torque Magnitude for {joint_name}')
            plt.xlabel('Time (s)')
            plt.ylabel('Torque Magnitude (Nm)')
            plt.grid(True)
            plt.legend()
            # Save plot to file
            output_file = f'{joint_name}_torque.png'
            plt.savefig(output_file)
            plt.close()
            self.get_logger().info(f'Saved plot for {
                joint_name} to {output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = TorquePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TorquePlotter node')
    finally:
        # Ensure final plots are saved on shutdown
        node.plot_torques()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
