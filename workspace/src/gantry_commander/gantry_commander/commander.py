#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveitpy import MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

class GantryCommander(Node):
    def __init__(self):
        super().__init__('gantry_commander')

        # Initialize moveit_commander (needs to happen once)
        roscpp_initialize([])

        # Setup robot and planning group
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("prismatic_chain")  # <- must match your MoveIt group name

        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)

        # Example sequence
        self.move_to([0.2, 0.0, 0.0])
        self.move_to([0.2, 0.2, 0.0])
        self.move_to([0.0, 0.0, 0.0])

        # shutdown
        roscpp_shutdown()
        rclpy.shutdown()

    def move_to(self, target):
        self.get_logger().info(f"Moving gantry to {target}")
        self.group.set_position_target(target)

        plan = self.group.plan()

        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        if success:
            self.get_logger().info("✅ Motion executed successfully")
        else:
            self.get_logger().warn("⚠️ Motion failed")


def main(args=None):
    rclpy.init(args=args)
    node = GantryCommander()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
