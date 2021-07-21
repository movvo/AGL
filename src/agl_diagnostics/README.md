# ME-00 DIAGNOSTICS

Repositorio trabajo `DIAGNOSTICS` en `ROS2` para `ME-00`.

## Funcionalidad

Este paquete es el encargado de asegurarse del buen estado del vehiculo general, es decir, comprueba el funcionamiento general de cada nodo de todo el sistema y crea un mensaje de salida generalizado para este.
Para más información sobre los mensajes de error ir a la **Wiki** del Teams en el canal **General**, pagina de **Diagnostics of vehicles**.

## Parametros

Actualmente los parametros serian los siguientes:
+ `timeout`: Timeout to detect if a node is down. The number represents the number of cycles withour reciving status message. As the frequency of publishing the status is fixed, the final timeout in seconds is the frequency^-1 * this value. Per example, if the frequency is 10Hz and timeout value is 10 -> Timeout = 1 second
+ `nodes_list`: List of the nodes name to check the status of the node.

## TODO

- [ ] Definir como poner el estado general del vehiculo.
- [ ] Definir que valor se le asigna al `connected`.
- [ ] Protocolo de comunicación con el `FleetManager` para avisar del estado.

## Arquitectura

La arquitectura de este nodo es simple, se suscribe a todos los topicos que se le especifican y publica uno de salida. El patron de los topicos siempre es:
+ `{namespace}/{node_name}/status`: Para la lectura del estado de los `node_name`'s.
+ `{namespace}/diagnostics/status`: Para la salida del estado general del vehiculo `namespace`.