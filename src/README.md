# ME-00

Repositorio trabajo en `ROS2` para `ME-00`.




## Arquitectura

![](../img/ME-00.png)



`DIAGNOSTICS`: Package al cual todos los demás se subscriben. Conoce el estado de todos los packages

`BRINGUP`: Package con archivos tipo *launch* para ejecutar diferentes opciones de aplicativos

`DESCRIPTION`: Package para visualizar modelo vehículo en *Gazebo* y *rViz* 

`WHEELMOTOR`: Package para el driver motor, odometria y joints (de las ruedas)

`JOYSTICK`: Package para mover el vehículo manualmente mediante joystick

`CONTROL`: Package para el control de movimiento vehículo `auto|manual`. Comunicación con `HMI`

`SAFETY`: Package para monitorizar seguridades. Comunicación con `relé seguridad`

`LASER`: Package driver laser. Driver del laser a utilizar

`IMU`: Package gestión del dispositivo de *inertial measurement unit*. Driver de la IMU a utilizar

`CAMERA`: Package driver cámara

`NAVIGATION`: Package navegación (basado en SLAM actual)

`PLANNER`: Package para gestión de *Search problems* para búsqueda de rutas óptimas

`DB-HISTORY`: Nodo conexión a base de datos local, guardar registros y obtenerlos cuando sea necesario

`POWER MANAGEMENT`: Nodo configuración y gestión batería

`ADAM`: Nodo driver del modulo de I/O Adam 6052