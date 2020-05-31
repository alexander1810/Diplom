#include "dwt.h"

extern uint32_t SystemCoreClock;
static uint32_t Delay_us, Delay_ms;

void DWT_DelayUpdate(void)
{
    Delay_ms = SystemCoreClock / 1000;    // Число тактов ядра за миллисекунду.
    Delay_us = SystemCoreClock / 1000000; // Число тактов ядра за микросекунду.
}

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT_DelayUpdate();
}

uint32_t DWT_GetTick(void)
{
   return DWT->CYCCNT;
}

void DWT_Delay_sec(uint32_t sec) // В секундах.
{
    uint32_t Count = DWT->CYCCNT;
    sec = sec * SystemCoreClock;
    while((DWT->CYCCNT - Count) < sec);
}

void DWT_Delay_ms(uint32_t ms) // В миллисекундах
{
    uint32_t Count = DWT->CYCCNT;
    ms = ms * Delay_ms;
    while((DWT->CYCCNT - Count) < ms);
}

void DWT_Delay_us(uint32_t us) // В микросекундах.
{
    uint32_t Count = DWT->CYCCNT;
    us = us * Delay_us;
    while((DWT->CYCCNT - Count) < us);
}

// 0 - если время меньше заданного. 1 если время больше и равно заданному.
// Параметры. start - число тиков ядра в начале отсчета. time - время.
uint8_t DWT_Test_sec(uint32_t start, uint32_t time)
{
    return (((DWT->CYCCNT - start) >= (time*SystemCoreClock)) ? 1 : 0);
}

uint8_t DWT_Test_ms(uint32_t start, uint32_t time)
{
    return (((DWT->CYCCNT - start) >= (time*Delay_ms)) ? 1 : 0);
}

uint8_t DWT_Test_us(uint32_t start, uint32_t time)
{
    return (((DWT->CYCCNT - start) >= (time*Delay_us)) ? 1 : 0);
}


// Время прошедшее с начала отсчета.
// Параметры. start - число тиков ядра в начале отсчета. current - текущее число тиков ядра.
uint32_t DWT_Time_sec(uint32_t start, uint32_t current)
{
    return (current - start) / SystemCoreClock;
}

uint32_t DWT_Time_ms(uint32_t start, uint32_t current)
{
    return (current - start) / Delay_ms;
}

uint32_t DWT_Time_us(uint32_t start, uint32_t current)
{
    return (current - start) / Delay_us;
}
