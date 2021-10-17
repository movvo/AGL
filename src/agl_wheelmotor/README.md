# AGL WHEELMOTOR

Repositorio trabajo `WHEELMOTOR` en `ROS2` para `AGL`.

Se usan dos controladores L298N de motores DC Brushless. Este package envia por serial el % del potencia deseado a cada motor y su dirección. Una Arduino se encarga de leer el mensaje
y enviar la señal PWM correspondiente.


## Dependencias

- `sudo apt install libserial-dev`

