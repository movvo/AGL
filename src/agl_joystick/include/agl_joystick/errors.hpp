/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Iñaki Lorente
   Contact: support.idi@ageve.net
*/
// STATES
#define  UNCONFIGURED   0
#define  STANDBY        1
#define  RUN            2
#define  SHUTDOWN       3
#define  FAULT          4


//CRITICAL ERRORS                                       bit
#define CONNECTION_FAILED               0b1             // 1

#define CONNECTION_FAILED_DESC          "Conexion fallida, configuracion erronea"
