#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from scipy.spatial.transform import Rotation as R

from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint, DisplayTrajectory, RobotState
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import PoseStamped

class Tramo4C(Node):
    def __init__(self):
        super().__init__('tramo_4c')
        self.plan_cli = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.display_pub = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        
        # Cliente de ejecución
        self.exec_cli = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        while not self.plan_cli.wait_for_service(timeout_sec=1.0) or not self.ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicios MoveIt2 para Tramo 4C...')
            
        while not self.exec_cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando servidor de ejecución (/execute_trajectory)...')

    def ejecutar(self):
        self.get_logger().info("--- INICIANDO TRAMO 4C (TRASLADO: PRE-PICK -> PRE-PLACE) ---")
        
        nombres_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        # 1. Definir la pose de DESTINO (Pre-Place)
        matriz_place = np.array([
            [-0.840, -0.364, -0.402,  0.358],
            [ 0.230,  0.431, -0.872, -0.549],
            [ 0.491, -0.825, -0.279,  0.383],
            [ 0.000,  0.000,  0.000,  1.000]
        ])

        rot_matrix_place = matriz_place[:3, :3]
        quat_place = R.from_matrix(rot_matrix_place).as_quat()

        pose_pre_place = PoseStamped()
        pose_pre_place.header.frame_id = "world"
        pose_pre_place.pose.position.x = matriz_place[0, 3]
        pose_pre_place.pose.position.y = matriz_place[1, 3]
        # CORRECCIÓN: Le sumamos el offset de 0.143 para que quede flotando en el Pre-Place
        pose_pre_place.pose.position.z = matriz_place[2, 3] + 0.143
        
        pose_pre_place.pose.orientation.x = quat_place[0]
        pose_pre_place.pose.orientation.y = quat_place[1]
        pose_pre_place.pose.orientation.z = quat_place[2]
        pose_pre_place.pose.orientation.w = quat_place[3]

        # Resolver IK para el destino (Pre-Place)
        ik_req_goal = GetPositionIK.Request()
        ik_req_goal.ik_request.group_name = 'kuka_arm'
        ik_req_goal.ik_request.ik_link_name = 'tool0'
        ik_req_goal.ik_request.pose_stamped = pose_pre_place
        
        # Semilla básica para ayudar al IK
        seed_goal = RobotState()
        seed_goal.joint_state.name = nombres_joints
        seed_goal.joint_state.position = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
        ik_req_goal.ik_request.robot_state = seed_goal
        
        ik_req_goal.ik_request.avoid_collisions = True
        ik_req_goal.ik_request.timeout.sec = 5

        future_goal = self.ik_cli.call_async(ik_req_goal)
        rclpy.spin_until_future_complete(self, future_goal)
        res_goal = future_goal.result()

        if res_goal.error_code.val != 1:
            self.get_logger().error(f"Fallo IK para Pre-Place destino. Código: {res_goal.error_code.val}")
            return

        self.get_logger().info("¡IK resuelto para Pre-Place con éxito!")

        # 2. Planear con RRTConnect
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'kuka_arm' 
        
        req.motion_plan_request.planner_id = ''
        req.motion_plan_request.num_planning_attempts = 15
        req.motion_plan_request.allowed_planning_time = 8.0 

        # Usar la posición física actual del robot (que ya está en Pre-Pick gracias al 4B)
        start_state = RobotState()
        start_state.is_diff = True
        req.motion_plan_request.start_state = start_state

        # Asignar restricciones de meta articulares para Pre-Place
        goal_constraint = Constraints()
        target_joints = res_goal.solution.joint_state
        
        for i, name in enumerate(target_joints.name):
            if name in nombres_joints:
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = target_joints.position[i]
                jc.tolerance_above = 0.03
                jc.tolerance_below = 0.03
                jc.weight = 1.0
                goal_constraint.joint_constraints.append(jc)

        req.motion_plan_request.goal_constraints.append(goal_constraint)
        
        self.get_logger().info("Planeando ruta desde Pre-Pick hasta Pre-Place...")
        future_plan = self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future_plan)
        res_plan = future_plan.result()
        
        if res_plan.motion_plan_response.error_code.val == 1:
            self.get_logger().info("ÉXITO: Ruta calculada. Ejecutando viaje en el robot...")
            disp_msg = DisplayTrajectory()
            disp_msg.trajectory.append(res_plan.motion_plan_response.trajectory)
            disp_msg.trajectory_start = res_plan.motion_plan_response.trajectory_start
            self.display_pub.publish(disp_msg)
            
            # Ejecutar movimiento real
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = res_plan.motion_plan_response.trajectory
            
            future_exec = self.exec_cli.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future_exec)
            goal_handle = future_exec.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Movimiento rechazado por el controlador.")
                return
                
            future_result = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, future_result)
            exec_res = future_result.result().result
            
            if exec_res.error_code.val == 1:
                self.get_logger().info("¡Movimiento completado con éxito! KUKA llegó al Pre-Place.")
            else:
                self.get_logger().error("Error ejecutando la trayectoria.")
                
        else:
            self.get_logger().error(f"Fallo en la planeación. Código: {res_plan.motion_plan_response.error_code.val}")

def main():
    rclpy.init()
    nodo = Tramo4C()
    nodo.ejecutar()
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
