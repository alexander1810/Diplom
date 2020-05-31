#include "main.h"


void DWT_DelayUpdate(void);
void DWT_Init(void);
uint32_t DWT_GetTick(void);
void DWT_Delay_sec(uint32_t sec);
void DWT_Delay_ms(uint32_t ms);
void DWT_Delay_us(uint32_t us);
uint8_t DWT_Test_sec(uint32_t start, uint32_t time);
uint8_t DWT_Test_ms(uint32_t start, uint32_t time);
uint8_t DWT_Test_us(uint32_t start, uint32_t time);
uint32_t DWT_Time_sec(uint32_t start, uint32_t current);
uint32_t DWT_Time_ms(uint32_t start, uint32_t current);
uint32_t DWT_Time_us(uint32_t start, uint32_t current);
