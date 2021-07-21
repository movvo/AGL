
#ifndef ROBOTEQ_ERRORS_MAIN_H
#define ROBOTEQ_ERRORS_MAIN_H

/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Martí Bolet Boixeda
   Contact: support.idi@ageve.net
*/

// Los bits de error del diagnostic estan representados en 32 bits.
// Los cuales estan distribuidos en segmentos de 8 bits.
// - Los 8 bits de mas peso corresponden a errores de Anybus
// - Los 8 bits del menos peso del segundo byte correponden a errores del Controlador.
// - Los 8 bits de mas peso del primer byte corresponden a errores del motor derecho.
// - Los 8 bits de menos peso corresponden a errores del motor izquierdo.

#define ANYBUS_ERROR     24  // Bits de desplazamiento
#define CONTROLLER_ERROR 16  // Bits de desplazamiento
#define MOTOR_RIGHT_ERROR 8  // Bits de desplazamiento
#define MOTOR_LEFT_ERROR  0  // Bits de desplazamiento

// Posibles errores del Anybus
// Los errores del Anybus se distinguen por su contador
// dentro de la lectura del ControlWord.
#define ERROR_COUNT     1  // First bit

// Feedback del estado del controlador
// FF: Fault Flag: Unsigned 16-bit
#define OVERHEAT        1  // First bit
#define OVERVOLTAGE     2  // Second bit
#define UDERVOLTAGE     4  // Third bit
#define SHORCIRCUIT     8  // Fourth bit
#define EMERCENCYSTOP  16  // Fifth bit
#define SETUPFAIL      32  // Sixth bit
#define MOSFETFAIL     64  // Seventh bit
#define DEFAULTCONF   128  // Eight bit

// Feedback del estado de cada motor
// FM: Motor Flag:
#define AMPSLIMIT         1  // First bit
#define MOTORSTALLED      2  // Second bit
#define LOOPERROR         4  // Third bit
#define SAFETYSTOP        8  // Fourth bit
#define FORWARDLIMIT     16  // Fifth bit
#define REVERSELIMIT     32  // Sixth bit
#define AMPPSTRIGGER     64  // Seventh bit

#endif // ROBOTEQ_ERRORS_MAIN_H