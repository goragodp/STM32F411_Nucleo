#include "stm32f4xx.h"
#include <math.h>
/* Private Include ************************************************************/
/* Private Definition *********************************************************/
#define TSYS01_RESET                 ((uint8_t)0x1E)
#define TSYS01_START_CONV            ((uint8_t)0x48)
#define TSYS01_READ_DATA             ((uint8_t)0x00)
#define TSYS01_READ_PROM0            ((uint8_t)0xA0)
#define TSYS01_READ_PROM1            ((uint8_t)0xA2)
#define TSYS01_READ_PROM2            ((uint8_t)0xA4)
#define TSYS01_READ_PROM3            ((uint8_t)0xA6)
#define TSYS01_READ_PROM4            ((uint8_t)0xA8)
#define TSYS01_READ_PROM5            ((uint8_t)0xAA)
#define TSYS01_READ_PROM6            ((uint8_t)0xAC)
#define TSYS01_READ_PROM7            ((uint8_t)0xAE)

#define _CS_LOW()                    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
#define _CS_HIGH()                   GPIO_SetBits(GPIOB, GPIO_Pin_12);
/* Private Typedef ************************************************************/
typedef struct {
  uint32_t tmp_data;
  uint16_t coef_k0;
  uint16_t coef_k1;
  uint16_t coef_k2;
  uint16_t coef_k3;
  uint16_t coef_k4;
} TSYS01Packet_t;
/* Private Variable ***********************************************************/
volatile uint32_t CounterTimer = 0;
uint8_t TIME_OUT = 1;
uint8_t tmp;
/* Private Function ***********************************************************/
void _delay_ms(uint32_t t){
  CounterTimer = 0;
  while(CounterTimer < t);
}
/* Private Prototype **********************************************************/
void system_init(void);
void RCC_Config(void);
void GPIO_Config(void);
void SPI_Config(void);
static uint8_t SPI_SendByte(uint8_t);



/*
Brief:     Main function for system exexution. This system consist of 3 peripheral
           - Temp Sensor (SPI inf)
           - Humidity Sensor (SPI inf)
           - 16bit ADC Module (ADC inf)
Version:   1
Autohr:    Goragod Pongthanisorn
*/
float conv_temp = 0;
float conv_temp2 = 0;

int main() {
  TSYS01Packet_t sys_peripheral_temp = {0};
  
  system_init();
  SysTick_Config(SystemCoreClock/10000);
  /*TSYS01 Reset Prorcess*/
  _delay_ms(1);
  _CS_LOW()
  SPI_SendByte(TSYS01_RESET);
  _delay_ms(200);
  _CS_HIGH()
    
  /*Read Coefficient*/
  _CS_LOW()
  SPI_SendByte(TSYS01_READ_PROM1);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k4 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k4 |= ((uint16_t)tmp);
  _CS_HIGH()
  _delay_ms(200);
    
  _CS_LOW()
  SPI_SendByte(TSYS01_READ_PROM2);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k3 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k3 |= ((uint16_t)tmp);
  _CS_HIGH()
  _delay_ms(500);
  
  _CS_LOW()
  SPI_SendByte(TSYS01_READ_PROM3);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k2 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k2 |= ((uint16_t)tmp);
  _CS_HIGH()
  _delay_ms(500);
  
  _CS_LOW()
  SPI_SendByte(TSYS01_READ_PROM4);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k1 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k1 |= ((uint16_t)tmp);
  _CS_HIGH()
  _delay_ms(500);
  
  _CS_LOW()
  SPI_SendByte(TSYS01_READ_PROM5);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k0 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  sys_peripheral_temp.coef_k0 |= ((uint16_t)tmp);
  _CS_HIGH()   
  _delay_ms(500);
  
    while(1){
      _CS_LOW()
      /*Start Conversion*/
      SPI_SendByte(TSYS01_START_CONV);
      _delay_ms(100);
      _CS_HIGH()
      
      _delay_ms(400);
     
      _CS_LOW()
      SPI_SendByte(TSYS01_READ_DATA);
      tmp = SPI_SendByte(TSYS01_READ_DATA);
      sys_peripheral_temp.tmp_data = ((uint16_t)tmp << 16);
      tmp = SPI_SendByte(TSYS01_READ_DATA);
      sys_peripheral_temp.tmp_data |= ((uint16_t)tmp << 8);
      tmp = SPI_SendByte(TSYS01_READ_DATA);
      sys_peripheral_temp.tmp_data |= ((uint16_t)tmp);
      _CS_HIGH()
      
      conv_temp = (int)( 
                    (-2)    * (sys_peripheral_temp.coef_k4) * pow(10, -21) * pow((sys_peripheral_temp.tmp_data) / 256 , 4) +
                    (4)     * (sys_peripheral_temp.coef_k3) * pow(10, -16) * pow((sys_peripheral_temp.tmp_data) / 256 , 3) +
                    (-2)    * (sys_peripheral_temp.coef_k2) * pow(10, -11) * pow((sys_peripheral_temp.tmp_data) / 256 , 2) +
                    (1)     * (sys_peripheral_temp.coef_k1) * pow(10, -6) * (sys_peripheral_temp.tmp_data) / 256 + 
                    (-1.5)  * (sys_peripheral_temp.coef_k0) * pow(10, -2) 
                   ) ;
    }

  return 0;
}

void GPIO_Config(void) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* Configure SPI2 pins: SCK */
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
 GPIO_Init(GPIOB, &GPIO_InitStructure);

 /* Configure SPI2 pins: MISO */
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
 GPIO_Init(GPIOB, &GPIO_InitStructure);

 /* Configure SPI2 pins: MOSI */
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
 GPIO_Init(GPIOB, &GPIO_InitStructure);

 /* Configure SD_SPI_CS_PIN pin: SD Card CS pin */
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOB, &GPIO_InitStructure);

 /* Connect PXx to SD_SPI_SCK */
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);

 /* Connect PXx to SD_SPI_MISO */
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2); 

 /* Connect PXx to SD_SPI_MOSI */
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);  
 
 _CS_HIGH()
}

void RCC_Config(void) {
  /* SD_SPI_CS_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_MISO_GPIO, SD_SPI_DETECT_GPIO 
       and SD_SPI_SCK_GPIO Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
 /* SPI2 Periph clock enable */
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); 
}

void SPI_Config(void) {
 SPI_InitTypeDef   SPI_InitStructure;
 /* SPI2 Config */
 SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
 SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
 SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
 SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
 SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
 SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

 SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
 SPI_InitStructure.SPI_CRCPolynomial = 7;
 SPI_Init(SPI2, &SPI_InitStructure);
 
 /* SPI2 enable */
 SPI_Cmd(SPI2, ENABLE);
}

void system_init(void) {
  RCC_Config();
  GPIO_Config();
  SPI_Config();
}

static uint8_t SPI_SendByte(uint8_t byte)
{
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
  SPI_SendData(SPI2, byte);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_ReceiveData(SPI2);
}

/* ISR and Callback ***********************************************************/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  CounterTimer++;
}