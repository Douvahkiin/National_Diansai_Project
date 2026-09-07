/*
 * Modified for driverlib-free integration (raw register access).
 * Original header had: #include "inc/hw_ints.h" and PORT_INT_YIELD = INT_FREERTOS.
 * This project has no C2000Ware driverlib, so the RTOS yield vector is declared
 * directly: PIE interrupt group 1, vector 8 (WAKE_INT, see F2837xD_pievect.h).
 */
#ifndef PORTDEFINES_H
#define PORTDEFINES_H

//-------------------------------------------------------------------------------------------------
// FreeRTOS Port Macros
//-------------------------------------------------------------------------------------------------
#define PORT_INT_YIELD          ( ( 1U << 8 ) | 8U )   /* PIE group 1, vector 8 (WAKE_INT) */
#define PORT_PIE_ACK_YIELD      ( 1U << ((( PORT_INT_YIELD & 0xFF00UL ) >> 8U ) - 1U ))
#define PORT_PIE_O_FLAG         ( 0x0CE1U + ( 2U * (( PORT_INT_YIELD & 0xFF00UL ) >> 8U )))
#define PORT_PIE_FLAG_YIELD     ( 1U << (( PORT_INT_YIELD & 0x00FFUL ) - 1U ))

#endif /* PORTDEFINES_H */
