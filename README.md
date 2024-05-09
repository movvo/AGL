# Andresito Jr
Robot interno de investigación. Andresito Jr (AGL)
# Datos importantes para la puesta en marcha del proyecto
1.- Conectar el power bank tanto a Andresito como a la NUC, la primera conexión permitirá movimiento de ruedas mientras que la segunda todo el procesado de información por parte de ROS2.

2.- Esperar hasta que la NUC permita conexión via AnyDesk y conectarse con ella. Trabajar desde ella. 

3.- Hacer el source install/setup.bash y el colcon build --symlink-install en workspace/AGL

4.- Conectar via Bluetooth un controller cualquiera (joystick).

5.- Hacer un ros2 launch agl_bringup start_launch.py.

6.- Pulsar la key R1 y mover todos los botones para que el controller funcione.

Para lanzar el mapeado, lanzar desde el proyecto: ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/mapper_params_online_async.yaml use_sim_time:=false y cambiar los parámetros que se vieran necesarios.

Es posible que se quiera comparar la odometría alcanzada, para eso seguir https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html#check-packages para instalar turtlebot y mediante la herramienta de rqt (robot_steering) que transmite cmd_vel se pueden generar trayectorias que al ser grabadas con rosabg (ros2 bag record /topic_name) podrán ser comparadas con evo_traj (evo_traj tum odom1.tum odom2.tum -p --plot_mode xy) después de haber instalado el repositorio evo (https://github.com/MichaelGrupp/evo) y después de haber transformado los rosbag2 a tum mediante evo_traj bag2 /odom1 --save_as_tum --all_topics (si queremos guardarlos todos en tum, puede ser posible que solo queramos el de odometría por ejemplo).

# Dependencias del proyecto
Las dependencias del proyecto se encuentran instaladas en el NUC. Al trabajar en local, no hace falta instalar nada via Docker diferente de todas las dependencias necesarias para ejecutar un workspace de ROS2. Ver el tutorial de instalación de ser necesario: https://docs.ros.org/en/humble/Installation.html

sudo apt-get install:

ros-humble-geographic-msgs

ros-humble-robot-localization

ros-humble-realsense2-camera

ros-humble-imu-tools

ros-humble-slam-toolbox



# A tener en cuenta
Al clonar el repositorio y probarlo en local dará problemas por no tener la Arduino conectada, no tener el joystick conectado y no tener el Lidar conectado. Es por ello necesaria la conexión remota con el NUC que controla el robot.

Aún estando configurado en la NUC tendremos que asegurarnos que los submodulos están el la rama correcta:
![image](https://github.com/movvo/AGL/assets/146711583/c3e4a2c0-4819-4294-a639-9fadbe1bb6c0)

