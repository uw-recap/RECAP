#ifndef _RECAP_LORA_H_
#define _RECAP_LORA_H_

#include "recap_common.h"
#include <LoRa.h>

#define LORA_CS 8
#define LORA_RST 4
#define LORA_IRQ 3
#define LORA_FREQ 915E6

int setupLoRa();

int transmitLoRa(Car_t* car);
int receiveLoRa(Car_t* car);

#endif
