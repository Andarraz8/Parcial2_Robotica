#!/usr/bin/env python3
import time
import csv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from scipy.spatial.transform import Rotation as R

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState, DisplayTrajectory
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import Pose
from rcl_interfaces.srv import GetParameters

import PyKDL
import kdl_parser_py.urdf

class Tramo4D(Node):
    def __init__(self):
        super().__init__('tramo_4d')
        self.cartesian_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_cli = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        while not self.cartesian_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicios cartesianos...')
            
        while not self.exec_cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando servidor de ejecución (/execute_trajectory)...')

        # Inicializar cadena KDL
        self.kdl_chain = self.obtener_cadena_kdl()
        self.kdl_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)

        # Matriz homogénea oficial del PLACE
        self.matriz_place = np.array([
            [-0.840, -0.364, -0.402,  0.358],
            [ 0.230,  0.431, -0.872, -0.549],
            [ 0.491, -0.825, -0.279,  0.383],
            [ 0.000,  0.000,  0.000,  1.000]
        ])

        rot_matrix = self.matriz_place[:3, :3]
        self.quat = R.from_matrix(rot_matrix).as_quat()

    def obtener_cadena_kdl(self):
        self.get_logger().info("Buscando el parámetro robot_description en /robot_state_publisher o /move_group...")
        
        nodes_to_try = ['/robot_state_publisher', '/move_group']
        param_client = None
        
        for node_name in nodes_to_try:
            service_name = f"{node_name}/get_parameters"
            client = self.create_client(GetParameters, service_name)
            self.get_logger().info(f"Probando servicio: {service_name}...")
            
            if client.wait_for_service(timeout_sec=2.0):
                param_client = client
                self.get_logger().info(f"¡Servicio de parámetros encontrado en {node_name}!")
                break
                
        if param_client is None:
            raise RuntimeError(
                "No se pudo encontrar el servicio get_parameters en /robot_state_publisher ni en /move_group. "
                "Verifica que tu robot esté lanzado correctamente."
            )
        
        request = GetParameters.Request()
        request.names = ['robot_description']
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response and response.values and response.values[0].string_value:
            urdf_str = response.values[0].string_value
            success, tree = kdl_parser_py.urdf.treeFromString(urdf_str)
            if success:
                self.get_logger().info("¡Árbol KDL y cadena base_link -> tool0 cargados con éxito!")
                return tree.getChain("base_link", "tool0")
                
        raise RuntimeError("El parámetro 'robot_description' está vacío o no se pudo parsear para KDL.")

    def generar_waypoints_quinticos(self, z_ini, z_fin, tf, num_puntos):
        h = z_fin - z_ini
        q_a0, q_a1, q_a2 = z_ini, 0, 0
        q_a3 = (10 * h) / (tf**3)
        q_a4 = (-15 * h) / (tf**4)
        q_a5 = (6 * h) / (tf**5)
        
        z_points = []
        tiempos = np.linspace(0, tf, num_puntos)
        for t in tiempos:
            z = q_a0 + q_a1*t + q_a2*(t**2) + q_a3*(t**3) + q_a4*(t**4) + q_a5*(t**5)
            z_points.append(z)
        return z_points

    def solicitar_trayectoria(self, z_waypoints, start_state=None, factor_vel=0.1):
        waypoints = []
        for z in z_waypoints:
            p = Pose()
            p.position.x = self.matriz_place[0, 3]
            p.position.y = self.matriz_place[1, 3]
            p.position.z = z
            p.orientation.x = self.quat[0]
            p.orientation.y = self.quat[1]
            p.orientation.z = self.quat[2]
            p.orientation.w = self.quat[3]
            waypoints.append(p)

        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.group_name = 'kuka_arm'
        req.link_name = 'tool0'
        req.waypoints = waypoints
        req.max_step = 0.01
        req.max_velocity_scaling_factor = float(factor_vel)
        req.max_acceleration_scaling_factor = float(factor_vel)
        
        if start_state is not None:
             req.start_state = start_state
        else:
             req.start_state.is_diff = True
             
        future = self.cartesian_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
        
    def calcular_y_mostrar_jacobiano_kdl(self, points, indices):
        num_joints = self.kdl_chain.getNrOfJoints()
        
        for idx in indices:
            p = points[idx]
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            q_vals = p.positions
            dq_vals = p.velocities

            jnt_array = PyKDL.JntArray(num_joints)
            for i in range(num_joints):
                jnt_array[i] = q_vals[i]

            kdl_jac = PyKDL.Jacobian(num_joints)
            self.kdl_solver.JntToJac(jnt_array, kdl_jac)

            J = np.zeros((6, num_joints))
            for i in range(6):
                for j in range(num_joints):
                    J[i, j] = kdl_jac[i, j]

            dq = np.array(dq_vals)
            V_cart = J @ dq
            v_lin = np.linalg.norm(V_cart[:3])

            self.get_logger().info(f"=== JACOBIANO KDL (4D) (t = {t:.3f} s) ===")
            self.get_logger().info(f"\n{np.array2string(J, precision=4, separator=', ')}")
            self.get_logger().info(f"Velocidad Lineal Cartesiana Resultante (|V|): {v_lin:.4f} m/s\n")

    def exportar_puntos_csv(self, trajectory, filename, num_muestras=5):
        points = trajectory.joint_trajectory.points
        total = len(points)
        indices = np.linspace(0, total - 1, num_muestras, dtype=int)
        
        self.calcular_y_mostrar_jacobiano_kdl(points, indices)

        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            header = ['time'] + [f'q{i+1}' for i in range(6)] + [f'dq{i+1}' for i in range(6)]
            writer.writerow(header)
            
            for idx in indices:
                p = points[idx]
                t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                row = [t] + list(p.positions) + list(p.velocities)
                writer.writerow(row)
                
        self.get_logger().info(f"¡Exportados {num_muestras} puntos con éxito a {filename}!")

    def ejecutar_trayectoria_fisica(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        future_exec = self.exec_cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future_exec)
        goal_handle = future_exec.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Movimiento rechazado.")
            return False
            
        future_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        exec_res = future_result.result().result
        
        return exec_res.error_code.val == 1

    def ejecutar(self):
        self.get_logger().info("--- INICIANDO TRAMO 4D (ROJO Y AZUL EN PLACE) CON KDL ---")
        
        z_place = self.matriz_place[2, 3]     
        z_pre_place = z_place + 0.143         
        
        z_wp_rojo = self.generar_waypoints_quinticos(z_pre_place, z_place, tf=2.0, num_puntos=10)
        start_state_actual = RobotState()
        start_state_actual.is_diff = True 
        
        res_rojo = self.solicitar_trayectoria(z_wp_rojo, start_state_actual, factor_vel=0.06)
        
        if res_rojo and res_rojo.fraction > 0.95:
            self.get_logger().info("Ruta roja calculada. Procesando KDL Jacobiano y exportando CSV...")
            self.exportar_puntos_csv(res_rojo.solution, "trayectoria_roja_4d.csv", num_muestras=5)
            
            if not self.ejecutar_trayectoria_fisica(res_rojo.solution):
                self.get_logger().error("Fallo la ejecución del descenso.")
                return
        else:
             self.get_logger().error("Fallo al calcular ruta roja.")
             return
             
        time.sleep(3.0) 
        
        z_wp_azul = self.generar_waypoints_quinticos(z_place, z_pre_place, tf=7.0, num_puntos=15)
        start_state_place = RobotState()
        start_state_place.is_diff = True 
        
        res_azul = self.solicitar_trayectoria(z_wp_azul, start_state_place, factor_vel=0.02)
        
        if res_azul and res_azul.fraction > 0.95:
            self.get_logger().info("Ruta azul calculada. Procesando KDL Jacobiano y exportando CSV...")
            self.exportar_puntos_csv(res_azul.solution, "trayectoria_azul_4d.csv", num_muestras=5)
            
            if self.ejecutar_trayectoria_fisica(res_azul.solution):
                self.get_logger().info("¡TRAMO 4D COMPLETADO CON ÉXITO!")
            else:
                self.get_logger().error("Fallo la ejecución del ascenso.")
        else:
             self.get_logger().error("Fallo al calcular ruta azul.")

def main():
    rclpy.init()
    nodo = Tramo4D()
    nodo.ejecutar()
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()