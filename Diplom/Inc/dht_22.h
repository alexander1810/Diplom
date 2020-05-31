#include "main.h"

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_12


void DHT11_Start (void);
uint8_t DHT11_Check_Response (void);
uint8_t DHT11_Read (void);



