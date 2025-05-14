#!/usr/bin/env python3
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
        # Store subscriptions in a list to avoid attribute error
        self._subs = []
        # Create subscriptions for each wrench topic
        for joint in ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5']:
            sub = self.create_subscription(
                WrenchStamped,
                f'/{joint}_ft_broadcaster/wrench',
                lambda msg, j=joint: self.wrench_callback(msg, j),
                10
            )
            self._subs.append(sub)
        
        # Initialize Matplotlib interactive plot with 3x2 grid
        plt.ion()  # Enable interactive mode
        self.fig, self.axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
        self.axes = self.axes.flatten()  # Flatten for easier indexing
        self.lines = {}  # Store plot lines for each joint
        self.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5']
        
        # Setup subplots
        for idx, joint in enumerate(self.joint_names):
            ax = self.axes[idx]
            ax.set_title(f'Torque Magnitude for {joint}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Torque (Nm)')
            ax.grid(True)
            # Initialize empty line
            line, = ax.plot([], [], 'b-', label=f'{joint} Torque')
            self.lines[joint] = line
            ax.legend()
        
        # Hide unused subplot (if any)
        for idx in range(len(self.joint_names), len(self.axes)):
            self.axes[idx].set_visible(False)
        
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Timer to update plot (every 0.1 seconds)
        self.plot_timer = self.create_timer(0.1, self.update_plot)
        self.get_logger().info('TorquePlotter node started, subscribing to wrench topics')

    def wrench_callback(self, msg, joint_name):
        # Initialize start time on first message
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Calculate current time relative to start
        current_time = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - self.start_time
        # Calculate torque magnitude
        torque = msg.wrench.torque
        torque_magnitude = np.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
        # Store data (limit to last 1000 points to prevent memory issues)
        self.torque_data[joint_name].append((current_time, torque_magnitude))
        if len(self.torque_data[joint_name]) > 1000:
            self.torque_data[joint_name] = self.torque_data[joint_name][-1000:]
        self.get_logger().debug(f'{joint_name}: Torque magnitude = {torque_magnitude:.3f} Nm at t = {current_time:.3f} s')

    def update_plot(self):
        # Update plot data for each joint
        for idx, joint_name in enumerate(self.joint_names):
            data = self.torque_data[joint_name]
            if not data:
                continue
            times, torques = zip(*data)
            # Update line data
            self.lines[joint_name].set_xdata(times)
            self.lines[joint_name].set_ydata(torques)
            # Adjust axis limits
            ax = self.axes[idx]
            ax.relim()
            ax.autoscale_view()
        
        # Redraw plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def destroy_node(self):
        # Close Matplotlib figure on node shutdown
        plt.close(self.fig)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TorquePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TorquePlotter node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
