#ifndef ROBOTEQ_CONSTANTS_MAIN_H
#define ROBOTEQ_CONSTANTS_MAIN_H

#include <map>
#include <string>
#include "me00_wheelmotor/anybus_constants.h" // For VAR constants

// Channels for motors
#define MOTOR_LEFT   1 // Channel value corresponding to the left channel/motor
#define MOTOR_RIGHT  2 // Channel value corresponding to the right channel/motor

// DEFINED VARIABLES FOR CAN
// Commands CAN id
#define MOTOR_AMPS_CANID     0x2100 // Motor Amps (A)
#define MOTOR_AMPS            TDPO1
#define ENCODER_COUNT_CANID  0x2104 // Enconder Count (C)
#define ENCODER_COUNT         TDPO2
#define FEEDBACK_CANID       0x2110 // Feedback (F)
#define FEEDBACK               DPO0
#define FAULT_FLAG_CANID     0x2112 // Fault Flag (FF)
#define FAULT_FLAG            TDPO3
#define FAULT_MOTOR_CANID    0x2122 // Fault Motor (FM)
#define FAULT_MOTOR           TDPO4

#define GO_CANID             0x2000 // Serial command of GO (G)
#define CANGO_CANID          0x2000 // Command GO but for via CANopen (CG)
#define CANGO                 RDPO1
#define SPEED_CANID          0x2002 // Speed Command (S)
#define SPEED                  PDO0
#define MOTOR_STOP_CANID     0x200E // Motor Stop (MS)
#define MOTOR_STOP            RDPO2
#define EMERGENCY_STOP_CANID 0x200C // Emergency Stop (EX)
#define EMERGENCY_STOP        RDPO3

// Commands Value Range: The value of the commands has to be
// The value is relative to the mode of operation.
#define MAX_AMPS     50
#define MAX_VOLTS    20
#define MAX_RANGE  1000
#define MIN_RANGE -1000

// Client Command Specifier
#define REQUEST_HEADER 0x600  // Request Header + Node Id
#define QUERY              4
#define COMMAND            2

#define RESPONSE_HEADER 0x580 // Response Header + Node Id
#define QUERY_RESPONSE      4
#define COMMAND_RESPONSE    6
#define ERROR_RESPONSE      8


// DEFINED VARIABLES FOR CONTROLLER AND MOTOR STATUS
// Function to check if the bit in `pos` of the `var` is 1 or 0
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))


#endif // ROBOTEQ_CONSTANTS_MAIN_H