#ifndef _RECAP_OBD_H_
#define _RECAP_OBD_H_

#include "recap_common.h"
#include <Arduino.h>
#include "wiring_private.h"
#include <OBD2UART.h>

#define OBD_RX_PIN 21
#define OBD_RX_PAD SERCOM_RX_PAD_1
#define OBD_TX_PIN 20
#define OBD_TX_PAD UART_TX_PAD_0

int setupOBD();

int readOBD(Car_t* car);

#endif
