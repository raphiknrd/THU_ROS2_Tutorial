#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import tf_transformations
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Robot Control Node gestartet. Warte auf Action-Server...')

    def send_goal(self, x, y, z, r_deg, p_deg, yaw_deg):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action Server nicht gefunden!')
            return

        goal_msg = MoveGroup.Goal()
        
        # TODO 1: Tragt hier den korrekten Namen der MoveIt-Planungsgruppe ein
        goal_msg.request.group_name = ''

        q = tf_transformations.quaternion_from_euler(
            math.radians(r_deg), math.radians(p_deg), math.radians(yaw_deg)
        )

        pos_con = PositionConstraint()
        pos_con.header.frame_id = 'world'
        
        # TODO 2: Tragt den exakten Namen des Endeffektor-Links ein
        pos_con.link_name = '' 
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.BOX
        s.dimensions = [0.01, 0.01, 0.01] 
        
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        
        pos_con.constraint_region.primitives.append(s)
        pos_con.constraint_region.primitive_poses.append(target_pose)
        pos_con.weight = 1.0

        ori_con = OrientationConstraint()
        ori_con.header.frame_id = 'world'
        ori_con.link_name = pos_con.link_name 
        ori_con.orientation.x = q[0]
        ori_con.orientation.y = q[1]
        ori_con.orientation.z = q[2]
        ori_con.orientation.w = q[3]
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_con)
        goal_constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 10

        self.get_logger().info('Sende Ziel...')
        
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    
    
    node.send_goal(-0.2, 0.6, 1.1, 90.0, 0.0, 180.0)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
