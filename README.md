# Andresito Jr
Robot interno de investigación. Andresito Jr (AGL)
# Datos importantes para la puesta en marcha del proyecto
1.- Conectar el power bank tanto a Andresito como a la NUC, la primera conexión permitirá movimiento de ruedas mientras que la segunda todo el procesado de información por parte de ROS2.

2.- Esperar hasta que la NUC permita conexión via AnyDesk y conectarse con ella. Trabajar desde ella. 

3.- Hacer el source install/setup.bash y el colcon build --symlink-install.

4.- Conectar via Bluetooth un controller cualquiera (joystick).

5.- Hacer un ros2 launch agl_bringup start_launch.py.

6.- Pulsar la key R1 y mover todos los botones para que el controller funcione.

# Dependencias del proyecto
Las dependencias del proyecto se encuentran instaladas en el NUC. Al trabajar en local, no hace falta instalar nada via Docker diferente de todas las dependencias necesarias para ejecutar un workspace de ROS2. Ver el tutorial de instalación de ser necesario: https://docs.ros.org/en/humble/Installation.html

# A tener en cuenta
Al clonar el repositorio y probarlo en local dará problemas por no tener la Arduino conectada, no tener el joystick conectado y no tener el Lidar conectado. Es por ello necesaria la conexión remota con el NUC que controla el robot.
