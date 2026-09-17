#!/usr/bin/env python3
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

class EvaluadorOMPL(Node):
    def __init__(self):
        super().__init__('evaluador_ompl')
        self.cli = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Esperando al servicio de MoveIt2 (/plan_kinematic_path)...')

    def evaluar(self, planner_id):
        req = GetMotionPlan.Request()
        
        # ATENCIÓN: Verifica que el nombre del grupo sea el correcto en tu Setup Assistant
        req.motion_plan_request.group_name = 'kuka_arm' 
        link_name = 'tool0' 
        
        req.motion_plan_request.planner_id = planner_id
        req.motion_plan_request.num_planning_attempts = 3
        req.motion_plan_request.allowed_planning_time = 15.0

        # 1. Restricción de Posición (pre-pick: X=0.380, Y=0.409, Z=0.250)
        goal_constraint = Constraints()
        pos_c = PositionConstraint()
        pos_c.header.frame_id = "world"
        pos_c.link_name = link_name
        
        # Tolerancia esférica de 1 cm para que el solver IK no falle por decimales
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]) 
        pose = Pose()
        pose.position.x = 0.380
        pose.position.y = 0.409
        pose.position.z = 0.250
        
        pos_c.constraint_region.primitives.append(sphere)
        pos_c.constraint_region.primitive_poses.append(pose)
        pos_c.weight = 1.0
        goal_constraint.position_constraints.append(pos_c)

        # 2. Restricción de Orientación (extraída de tu matriz PICK)
        rot_matrix = np.array([
            [ 0.311, -0.947, -0.085],
            [-0.128, -0.131,  0.983],
            [-0.942, -0.295, -0.162]
        ])
        quat = R.from_matrix(rot_matrix).as_quat() # Convierte a [x, y, z, w]

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = "world"
        ori_c.link_name = link_name
        ori_c.orientation.x = quat[0]
        ori_c.orientation.y = quat[1]
        ori_c.orientation.z = quat[2]
        ori_c.orientation.w = quat[3]
        ori_c.absolute_x_axis_tolerance = 0.15
        ori_c.absolute_y_axis_tolerance = 0.15
        ori_c.absolute_z_axis_tolerance = 0.15
        ori_c.weight = 1.0
        goal_constraint.orientation_constraints.append(ori_c)

        req.motion_plan_request.goal_constraints.append(goal_constraint)

        self.get_logger().info(f"\n--- Evaluando {planner_id} ---")
        
        start_time = time.time()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        planning_time = time.time() - start_time

        response = future.result()
        if response.motion_plan_response.error_code.val == 1: # 1 = SUCCESS
            traj = response.motion_plan_response.trajectory.joint_trajectory.points
            
            # Calcular longitud de trayectoria (Suma de distancias en espacio articular)
            longitud = 0.0
            for i in range(1, len(traj)):
                q1 = traj[i-1].positions
                q2 = traj[i].positions
                dist = math.sqrt(sum([(a - b)**2 for a, b in zip(q1, q2)]))
                longitud += dist

            self.get_logger().info(f"ÉXITO: Ruta encontrada esquivando el obstáculo.")
            self.get_logger().info(f"> Tiempo de planeación: {planning_time:.4f} seg")
            self.get_logger().info(f"> Nodos generados (Suavidad): {len(traj)} waypoints")
            self.get_logger().info(f"> Longitud del camino: {longitud:.4f} rad")
        else:
            self.get_logger().error(f"Fallo al planear. Código de error MoveIt: {response.motion_plan_response.error_code.val}")

def main():
    rclpy.init()
    nodo = EvaluadorOMPL()
    
    # Evaluar ambos planeadores exigidos en el taller
    planeadores = ["RRTConnectkConfigDefault", "RRTstarkConfigDefault"]
    
    for planner in planeadores:
        nodo.evaluar(planner)
        time.sleep(1.0)
        
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
