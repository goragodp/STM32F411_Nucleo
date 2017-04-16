#include "tsys01.h"
#include "systick_delay.h"
#include "math.h"

/* Private Viaralbe ***********************************************************/
extern TSYS01_Instance TempSensor;
uint8_t System_State = INIT;
/* Private Prototype **********************************************************/
void system_init(void);
void RCC_Config(void);
void GPIO_Config(void);
void SPI_Config(void);
static uint8_t SPI_SendByte(uint8_t);

/* Implementation *************************************************************/
void GPIO_Config(void) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* Configure SPI2 pins: SCK */
 GPIO_InitStructure.GPIO_Pin = TSYS01_SCLK_PIN;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
 GPIO_Init(TSYS01_SCLK_PORT, &GPIO_InitStructure);

 /* Configure SPI2 pins: MISO */
 GPIO_InitStructure.GPIO_Pin = TSYS01_MISO_PIN; 
 GPIO_Init(TSYS01_MISO_PORT, &GPIO_InitStructure);

 /* Configure SPI2 pins: MOSI */
 GPIO_InitStructure.GPIO_Pin = TSYS01_MOS_PIN;
 GPIO_Init(TSYS01_MOSI_PORT, &GPIO_InitStructure);

 /* Configure SD_SPI_CS_PIN pin: SD Card CS pin */
 GPIO_InitStructure.GPIO_Pin = TSYS01_CS_PIN;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(TSYS01_CS_PORT, &GPIO_InitStructure);

 /* Connect PXx to SD_SPI_SCK */
 GPIO_PinAFConfig(TSYS01_SCLK_PORT, TSYS01_SCLK_PIN_SORUCE, TSYS01_SCLK_PIN_AF);

 /* Connect PXx to SD_SPI_MISO */
 GPIO_PinAFConfig(TSYS01_MISO_PORT, TSYS01_MISO_PIN_SORUCE, TSYS01_MISO_PIN_AF); 

 /* Connect PXx to SD_SPI_MOSI */
 GPIO_PinAFConfig(TSYS01_MOSI_PORT, TSYS01_MOSI_PIN_SORUCE, TSYS01_MOSI_PIN_AF);  
 _CS_HIGH();
 
}

void RCC_Config(void) {
  /* SD_SPI_CS_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_MISO_GPIO, SD_SPI_DETECT_GPIO 
       and SD_SPI_SCK_GPIO Periph clock enable */
 TSYS01_RCC_GPIO_CLOCK_CMD(TSYS01_RCC_GPIO_CLOCK_PORT, ENABLE);
 /* SPI2 Periph clock enable */
 TSYS01_RCC_SPI_CLOCK_CMD(TSYS01_RCC_GPIO_CLOCK_PORT, ENABLE); 
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
 SPI_Init(TSYS01_SPI, &SPI_InitStructure);
 
 /* SPI2 enable */
 SPI_Cmd(TSYS01_SPI, ENABLE);
}

void system_init(void) {
  RCC_Config();
  GPIO_Config();
  SPI_Config();
}


static uint8_t SPI_SendByte(uint8_t byte)
{
  while (SPI_I2S_GetFlagStatus(TSYS01_SPI, SPI_I2S_FLAG_TXE) == RESET); 
  SPI_SendData(TSYS01_SPI, byte);
  while (SPI_I2S_GetFlagStatus(TSYS01_SPI, SPI_I2S_FLAG_RXNE) == RESET);
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_ReceiveData(TSYS01_SPI);
}


void TSYS01_CoeffCalibrate(TSYS01Packet_t* data) {
  uint8_t tmp;
   /*TSYS01 Reset Prorcess*/
  _delay_ms(1);
  _CS_LOW();
  SPI_SendByte(TSYS01_RESET);
  _delay_ms(200);
  _CS_HIGH();
    
  /*Read Coefficient*/
  _CS_LOW();
  SPI_SendByte(TSYS01_READ_PROM1);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k4 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k4 |= ((uint16_t)tmp);
  _CS_HIGH();
  _delay_ms(200);
    
  _CS_LOW();
  SPI_SendByte(TSYS01_READ_PROM2);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k3 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k3 |= ((uint16_t)tmp);
  _CS_HIGH();
  _delay_ms(500);
  
  _CS_LOW();
  SPI_SendByte(TSYS01_READ_PROM3);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k2 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k2 |= ((uint16_t)tmp);
  _CS_HIGH();
  _delay_ms(500);
  
  _CS_LOW();
  SPI_SendByte(TSYS01_READ_PROM4);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k1 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k1 |= ((uint16_t)tmp);
  _CS_HIGH();
  _delay_ms(500);
  
  _CS_LOW();
  SPI_SendByte(TSYS01_READ_PROM5);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k0 |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->coef_k0 |= ((uint16_t)tmp);
  _CS_HIGH()   
  _delay_ms(500);
}

float TSYS01_GetTemp(TSYS01Packet_t* data) {
  uint8_t tmp;
  _CS_LOW();
  
   /*Start Conversion*/
  SPI_SendByte(TSYS01_START_CONV);
  _delay_ms(100);
  _CS_HIGH() 
  _delay_ms(400);
  
  _CS_LOW()
  SPI_SendByte(TSYS01_READ_DATA);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->raw = ((uint16_t)tmp << 16);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->raw |= ((uint16_t)tmp << 8);
  tmp = SPI_SendByte(TSYS01_READ_DATA);
  data->raw |= ((uint16_t)tmp);
  _CS_HIGH()
   
  return       (int)( 
                    (-2)    * (data->coef_k4) * pow(10, -21) * pow((data->raw) / 256 , 4) +
                    (4)     * (data->coef_k3) * pow(10, -16) * pow((data->raw) / 256 , 3) +
                    (-2)    * (data->coef_k2) * pow(10, -11) * pow((data->raw) / 256 , 2) +
                    (1)     * (data->coef_k1) * pow(10, -6) * (data->raw) / 256 + 
                    (-1.5)  * (data->coef_k0) * pow(10, -2) 
                    );
}

void TSYS01_Init(TSYS01_Instance* system_instance){
  system_init();
  system_instance->temp = 0;
  system_instance->sensor_val = {0};
}

void TSYS01_ServiceRoutine(TSYS01_Instance* instance) {
  switch(System_State) {
    case INIT:
       TSYS01_Init(instance);
       System_State = CALB;
    break;
    case CALB:
      TSYS01_CoeffCalibrate(&instance->sensor_val);
      System_State = MEAS;
    break;
    case MEAS:
      instance->temp =  TSYS01_GetTemp((instance->sensor_val);
      /*Sanity Calibrate*/
      if(instance->temp > 1 && instance < 95 )
        System_State = CALB;
      else
        System_State = MEAS;
    break;
    
    default:
      while(1);
    break;
  }
  
}