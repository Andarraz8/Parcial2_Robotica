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

class Tramo4A(Node):
    def __init__(self):
        super().__init__('tramo_4a')
        self.plan_cli = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.display_pub = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        
        # Cliente para ejecutar en el robot sólido
        self.exec_cli = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        while not self.plan_cli.wait_for_service(timeout_sec=1.0) or not self.ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicios MoveIt2...')
            
        while not self.exec_cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando servidor de ejecución (/execute_trajectory)...')

    def ejecutar(self):
        self.get_logger().info("--- INICIANDO TRAMO 4A (HOME -> PRE-PICK) ---")

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = 'kuka_arm'
        ik_req.ik_request.ik_link_name = 'tool0'

        rot_matrix = np.array([
            [ 0.311, -0.947, -0.085],
            [-0.128, -0.131,  0.983],
            [-0.942, -0.295, -0.162]
        ])
        quat = R.from_matrix(rot_matrix).as_quat()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose.position.x = 0.380
        pose_stamped.pose.position.y = 0.409
        pose_stamped.pose.position.z = 0.250
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]

        ik_req.ik_request.pose_stamped = pose_stamped

        seed = RobotState()
        seed.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        seed.joint_state.position = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        ik_req.ik_request.robot_state = seed
        
        # CORRECCIÓN 1: Forzar a que la meta calculada NO choque con la mesa/pieza
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.timeout.sec = 2

        future_ik = self.ik_cli.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future_ik)
        ik_res = future_ik.result()

        if ik_res.error_code.val != 1:
            self.get_logger().error("Fallo IK para pre-pick. (El punto deseado está dentro de un obstáculo)")
            return

        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'kuka_arm' 
        
        # CORRECCIÓN 2: Dejar vacío para evitar incompatibilidad de nombres de OMPL
        req.motion_plan_request.planner_id = ''
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 5.0 

        # CORRECCIÓN 3: is_diff=True le dice que use la posición ACTUAL del robot (Magia pura)
        start_state = RobotState()
        start_state.is_diff = True 
        req.motion_plan_request.start_state = start_state

        goal_constraint = Constraints()
        target_joints = ik_res.solution.joint_state

        for i, name in enumerate(target_joints.name):
            if name in ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']:
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = target_joints.position[i]
                jc.tolerance_above = 0.05
                jc.tolerance_below = 0.05
                jc.weight = 1.0
                goal_constraint.joint_constraints.append(jc)

        req.motion_plan_request.goal_constraints.append(goal_constraint)

        future_plan = self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future_plan)
        res_plan = future_plan.result()

        if res_plan.motion_plan_response.error_code.val == 1:
            self.get_logger().info("ÉXITO: Ruta calculada. Ejecutando físicamente en el robot...")
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
                self.get_logger().info("¡Movimiento completado con éxito! KUKA en Pre-Pick.")
            else:
                self.get_logger().error("Error ejecutando la trayectoria.")

        else:
            self.get_logger().error(f"Fallo el planificador (Código de error: {res_plan.motion_plan_response.error_code.val})")

def main():
    rclpy.init()
    nodo = Tramo4A()
    nodo.ejecutar()
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
