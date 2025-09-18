import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GantryCommander(Node):
    def __init__(self):
        super().__init__('gantry_commander')
        self.pub = self.create_publisher(JointTrajectory,
                                         '/prismatic_chain_controller/joint_trajectory',
                                         10)
    
    def move_to(self, x, y, z, rot):
        traj = JointTrajectory()
        traj.joint_names = ['map_to_xaxis', 'xaxis_to_yaxis', 'yaxis_to_zaxis', 'zaxis_to_eemount']

        point = JointTrajectoryPoint()
        point.positions = [x, y, z, rot]
        point.time_from_start.sec = 3
        
        traj.points.append(point)
        self.pub.publish(traj)
        self.get_logger().info(f"Sent trajectory to ({x}, {y}, {z}, {rot})")

def main():
    rclpy.init()
    node = GantryCommander()
    node.move_to(0.2, 0.1, 0.3, 0.3)  # example target
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
