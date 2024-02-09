/*
   Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Martí Bolet Boixeda
   Contact: support.idi@ageve.net
*/

#ifndef WIT_IMU_ERRORS_MAIN_HPP
#define WIT_IMU_ERRORS_MAIN_HPP


// Los bits de error del diagnostic estan representados en 32 bits.

// El bit de mas peso de todos se utilizara para marcar un error de connexión
// con el dispositivo hardware.
#define BAD_PARAMETERS            0b1
#define CANT_OPEN_PORT            0b10
#define CANT_OPEN_PORT_NO_PERM    0b100
#define BAD_BAUDRATE              0b1000
#define CANT_SET_BAUDRATE         0b10000
#define WRONG_BAUDRATE            0b100000
#define IMU2USE_DOESNT_EXIST      0b1000000
#define NOT_CONNECTED             0b10000000
#define CALIBRATION_CRC           0b100000000
#define INIT_ERROR                0b1000000000
#define SW_DATA_ERROR             0b10000000000
#define SW_ALGORITHM_ERROR        0b100000000000
#define SW_ERROR                  0b1000000000000
#define HW_ACCELL_ERROR           0b10000000000000
#define HW_RATE_ERROR             0b100000000000000
#define HW_SENSOR_ERROR           0b1000000000000000
#define HW_MASTER_FAIL            0b10000000000000000
#define IMU_TIMEOUT               0b100000000000000000  // 131072
#define UNKNOWN_ERROR             0b10000000000000000000000000000000

// Descriptions
#define BAD_PARAMETERS_DESC         "CRITICAL Bad parameters types"
#define CANT_OPEN_PORT_DESC         "CRITICAL Can't open port, not found in filesystem"
#define CANT_OPEN_PORT_NO_PERM_DESC "CRITICAL Can't open port but exists, did you have permissions?"
#define BAD_BAUDRATE_DESC           "CRITICAL Baud rate unexistent"
#define CANT_SET_BAUDRATE_DESC      "CRITICAL Can't set baudrate to port"
#define WRONG_BAUDRATE_DESC         "CRITICAL Baudrate used to read data does not correspond to device baudrate"
#define IMU2USE_DOESNT_EXIST_DESC   "CRITICAL IMU Device to use doesn't exist"
#define NOT_CONNECTED_DESC          "CRITICAL Unable to connect to driver"
#define CALIBRATION_CRC_DESC        "CRITICAL ACEINNA Error: Calibration CRC Error"
#define INIT_ERROR_DESC             "CRITICAL ACEINNA Error: Initialization Error"
#define SW_DATA_ERROR_DESC          "CRITICAL SW Error: Data Error"
#define SW_ALGORITHM_ERROR_DESC     "CRITICAL SW Error: Algorithm Error"
#define SW_ERROR_DESC               "CRITICAL ACEINNA Software Error"
#define HW_ACCELL_ERROR_DESC       "CRITICAL HW Error: Accelleration Error"
#define HW_RATE_ERROR_DESC          "CRITICAL HW Error: Rate Error"
#define HW_SENSOR_ERROR_DESC        "CRITICAL HW Error: Sensor Error"
#define HW_MASTER_FAIL_DESC         "CRITICAL HW Error: MASTER FAIL"
#define UNKOWN_ERROR_DESC           "CRITICAL Unknown error"
#define IMU_TIMEOUT_DESC            "CRITICAL Timeout receiving imu data from cancoms"


// WARNINGS
#define TIMEOUT_READING          0b1

#define TIMEOUT_READING_DESC     "WARN Timeout when reading data from serial"

#endif // WITS_IMU_ERRORS_MAIN_H