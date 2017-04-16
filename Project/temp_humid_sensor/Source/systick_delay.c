#include "systick_delay.h"

static uint32_t CounterTimer;

void _delay_ms(uint32_t t){
  CounterTimer = 0;
  while(CounterTimer < t);
}

/*
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
*/
void SysTick_Handler(void)
{
  CounterTimer++;
}