#ifndef ANYBUS_CONSTANTS_MAIN_H
#define ANYBUS_CONSTANTS_MAIN_H

// Este fichero contiene las constantes de las 
// variables de la pasarela Anybus.


// ANYBUS Objects: Class (in hex)
#define IDENTITY_CLASS    0x01 // Identity Object: Holds general information and status about the interface.
#define ASSEMBLY_CLASS    0x04 // Assembly Object: Holds the Input and Output data buffers.
#define DIAGNOSTIC_CLASS  0xAA // Diagnostic Object: Contains diagnosticinformation about the ethernet interface.
#define TCPIP_INF_CLASS   0xF5 // TCP/IP Interface Object: Holds the IP settings of the interface.
#define ETH_LINK_CLASS    0xF6 // Ethernet Link Object: Holds the low level communication properties of teh interface.

// For the Assembly Object:
// This are the Instance Attributes:
#define GET_INSTANCE_ID  0x0064 // For reading the buffer
#define SET_INSTANCE_ID  0x0096 // For writting in the buffer

#define STATUS_CONTROL_WORD  0x00 // Address of the Status or Control Word in the buffer

// Status Word: 2bytes = 16 bits
#define GCC    61440 // General Cycle Counter: Los 4 bits de mas peso. Contador que se incrementa en cada intercambio de networks correctamente.
#define GEC     3840 // General Counter Error: Los 4 bits de menos peso del segundo byte. Contador que se incrementa cada vez 
#define MM        12 // Master Mode: Si la red es la master, indica el estado operacional actual del master a la otra red.
#define INIT       2 // Init: El segundo bit de menos peso. Indica si la otra red ha sido inicializada.
#define RUN        1 // Run: El bit de menos peso. Inidica el estado del intercambio de datos con la otra red.

// Control Word: 2bytes = 16 bits
#define RESET     128 // Reset: El bit de mas peso del primer byte.
#define MASTERMODE 12 // Master Mode: Si la red es la master, indica el estado operacional actual del master a la otra red.

// Estan separados por TDPOs y RDPOs, y a su vez
// por parejas, ya que se representan en CH1 y CH2.
#define CONTROL_STATUS_WORD 0x00 // Valor de ubicación del Control/Status Word

#define PDO0 0X00 // Variable para definir a valor nulo
// TDPOs
#define TDPO1 0x02 // Variable de "" para CH1
#define TDPO2 0x06 // Variable de "" para CH1
#define TDPO3 0x0A // Variable de "" para CH1
#define TDPO4 0x0E // Variable de "" para CH1

// RDPOs
#define RDPO1 0x02 // Variable de "" para CH1
#define RDPO2 0x06 // Variable de "" para CH1
#define RDPO3 0x0A // Variable de "" para CH1
#define RDPO4 0x0E // Variable de "" para CH1

#endif // ANYBUS_CONSTANTS_MAIN_H


