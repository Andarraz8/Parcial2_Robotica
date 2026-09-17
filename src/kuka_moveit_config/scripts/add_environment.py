#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class EnvironmentBuilder(Node):
    def __init__(self):
        super().__init__('environment_builder')
        self.publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.get_logger().info('Conectando con MoveIt2...')

    def create_color(self, object_id, r, g, b, a=1.0):
        color = ObjectColor()
        color.id = object_id
        color.color.r = float(r)
        color.color.g = float(g)
        color.color.b = float(b)
        color.color.a = float(a)
        return color

    def publish_scene(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        # 1. Mesa de trabajo (PLACE) - Pilar sólido hasta el suelo
        table = CollisionObject()
        table.header.frame_id = 'world'
        table.id = 'table_surface'
        
        # Dimensiones: 40x40 cm de base y 35 cm de alto (para quedar justo debajo de Z=0.383)
        altura_mesa = 0.320
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.40, 0.40, altura_mesa])
        table_pose = Pose()
        
        # NUEVAS COORDENADAS EXITOSAS DE PLACE
        table_pose.position.x = 0.358
        table_pose.position.y = -0.549
        # El centro de la caja en Z debe ser la mitad de la altura total
        table_pose.position.z = altura_mesa / 2.0  
        table_pose.orientation.w = 1.0
        
        table.primitives.append(box)
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD

        # 2. Obstáculo fijo (Punto medio entre HOME y PICK)
        obstacle = CollisionObject()
        obstacle.header.frame_id = 'world'
        obstacle.id = 'obstacle_post'
        # Dimensiones: 60 cm de alto, 4 cm de radio
        cylinder = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[0.6, 0.04])
        obs_pose = Pose()
        obs_pose.position.x = 0.430
        obs_pose.position.y = 0.205
        obs_pose.position.z = 0.300
        obs_pose.orientation.w = 1.0
        obstacle.primitives.append(cylinder)
        obstacle.primitive_poses.append(obs_pose)
        obstacle.operation = CollisionObject.ADD

        # 3. Pieza a manipular (PICK) - Esfera Roja
        piece = CollisionObject()
        piece.header.frame_id = 'world'
        piece.id = 'target_piece'
        # Esfera de 3 cm de radio
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.03])
        piece_pose = Pose()
        piece_pose.position.x = 0.420 #0.380
        piece_pose.position.y = 0.440 #0.409
        piece_pose.position.z = 0.107
        piece_pose.orientation.w = 1.0
        piece.primitives.append(sphere)
        piece.primitive_poses.append(piece_pose)
        piece.operation = CollisionObject.ADD

        # Añadir objetos a la escena
        scene_msg.world.collision_objects.extend([table, obstacle, piece])

        # Añadir colores a la escena (R, G, B)
        scene_msg.object_colors.append(self.create_color('table_surface', 0.0, 0.0, 1.0))  # Azul
        scene_msg.object_colors.append(self.create_color('obstacle_post', 0.5, 0.5, 0.5))  # Gris
        scene_msg.object_colors.append(self.create_color('target_piece', 1.0, 0.0, 0.0))   # Rojo

        self.publisher.publish(scene_msg)
        self.get_logger().info('¡Entorno ajustado a las matrices DH enviado a RViz2!')

def main():
    rclpy.init()
    node = EnvironmentBuilder()
    time.sleep(1.5)
    for _ in range(5):
        node.publish_scene()
        rclpy.spin_once(node, timeout_sec=0.2)
        time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
