Desarrollo en Moveit del robot KUKA KR6 R700 para la comprobacion de la cinematica directa, cinemcatica inversa y jacobiano


## CODIGO PARA SIMULACION EN MOVEIT##
Codigos para inicializar el paquete y abrir Moveit

cd ~/ros2_ws

colcon build

source install/setup.bash

ros2 launch kuka_moveit_config demo.launch.py


Codigo para añadir en el espacio de RViz la pelota, el obstaculo (pilar) y la mesa
python3 ~/ros2_ws/src/kuka_moveit_config/scripts/add_environment.py


Antes de hacer la simulacion de cada uno de los tramos, es muy importante hacer esta linea de codigo para el solver KDL en especifico para los tramos 4B y 4D
colcon build --packages-select kdl_parser_py urdf_parser_py


Ahora si se puede procedes con la simulacion de cada uno de los tramos, se puede desarrollar sucesivamente los siguientes codigos
python3 ~/ros2_ws/src/kuka_moveit_config/scripts/tramo_4a.py
python3 ~/ros2_ws/src/kuka_moveit_config/scripts/tramo_4b.py
python3 ~/ros2_ws/src/kuka_moveit_config/scripts/tramo_4c.py
python3 ~/ros2_ws/src/kuka_moveit_config/scripts/tramo_4d.py


## CODIGOS AUXILIARES PARA COMPROBACION CON MATLAB##
Observacion de la matriz homogenea
ros2 launch kuka_moveit_config demo.launch.py


Angulos ṕara la cinematica inversa
ros2 topic echo /joint_states --once


Evaluacion de planeadores para tramo 4A (RRTConnect y RRT*)
python3 ~/ros2_ws/src/kuka_moveit_config/scripts/evaluar_planeadores.py


Evaluacion de interpolacion cubica y quintuple
python3 ~/ros2_ws/src/kuka_moveit_config/scripts/calculo_interpolacion.py
