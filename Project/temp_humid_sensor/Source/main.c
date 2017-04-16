#include "stm32f4xx.h"
#include <math.h>
/* Private Include ************************************************************/
/* Private Definition *********************************************************/
/* Private Typedef ************************************************************/

/* Private Variable ***********************************************************/

/* Private Function ***********************************************************/

/* Private Prototype **********************************************************/



/*
Brief:     Main function for system exexution. This system consist of 3 peripheral
           - Temp Sensor (SPI inf)
           - Humidity Sensor (SPI inf)
           - 16bit ADC Module (ADC inf)
Version:   1
Autohr:    Goragod Pongthanisorn
*/
int main() {
  
 SysTick_Config(SystemCoreClock/10000);
 
  
 while(1);
    
}
