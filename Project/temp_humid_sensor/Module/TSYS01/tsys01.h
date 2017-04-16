#ifndef __TSYS01_H__
#define __TSYS01_H__

#include "stm32f4xx.h"
/* Private Include ************************************************************/

/* Private Definition *********************************************************/
#define TSYS01_RESET                      ((uint8_t)0x1E)
#define TSYS01_START_CONV                 ((uint8_t)0x48)
#define TSYS01_READ_DATA                  ((uint8_t)0x00)
#define TSYS01_READ_PROM0                 ((uint8_t)0xA0)
#define TSYS01_READ_PROM1                 ((uint8_t)0xA2)
#define TSYS01_READ_PROM2                 ((uint8_t)0xA4)
#define TSYS01_READ_PROM3                 ((uint8_t)0xA6)
#define TSYS01_READ_PROM4                 ((uint8_t)0xA8)
#define TSYS01_READ_PROM5                 ((uint8_t)0xAA)
#define TSYS01_READ_PROM6                 ((uint8_t)0xAC)
#define TSYS01_READ_PROM7                 ((uint8_t)0xAE)

#define _CS_LOW()                         GPIO_ResetBits(GPIOB, GPIO_Pin_12);
#define _CS_HIGH()                        GPIO_SetBits(GPIOB, GPIO_Pin_12);

/*RCC clock control definition*/
#define TSYS01_RCC_SPI_CLOCK_CMD          RCC_APB1PeriphClockCmd
#define TSYS01_RCC_SPI_CLOCK_PORT        RCC_APB1Periph_SPI2
#define TSYS01_RCC_GPIO_CLOCK_CMD         RCC_AHB1PeriphClockCmd
#define TSYS01_RCC_GPIO_CLOCK_PORT        RCC_AHB1Periph_GPIOB   

#define TSYS01_SPI                        SPI2

/*Pin & Port definition*/
//CS
#define TSYS01_CS_PIN                     GPIO_Pin_12
#define TSYS01_CS_PORT                    GPIOB
//SCLK
#define TSYS01_SCLK_PIN                   GPIO_Pin_13
#define TSYS01_SCLK_PORT                  GPIOB
#define TSYS01_SCLK_PIN_SORUCE            GPIO_PinSource13
#define TSYS01_SCLK_PIN_AF                GPIO_AF_SPI2
//MISO
#define TSYS01_MISO_PIN                   GPIO_Pin_14
#define TSYS01_MISO_PORT                  GPIOB
#define TSYS01_MISO_PIN_SORUCE            GPIO_PinSource14
#define TSYS01_MISO_PIN_AF                GPIO_AF_SPI2
//MOSI
#define TSYS01_MOS_PIN                    GPIO_Pin_15
#define TSYS01_MOSI_PORT                  GPIOB
#define TSYS01_MOSI_PIN_SORUCE            GPIO_PinSource15
#define TSYS01_MOSI_PIN_AF                GPIO_AF_SPI2

/* Private Typedef ************************************************************/
typedef struct {
  uint32_t raw;
  uint16_t coef_k0;
  uint16_t coef_k1;
  uint16_t coef_k2;
  uint16_t coef_k3;
  uint16_t coef_k4;
} TSYS01Packet_t;

typedef struct {
  TSYS01Packet_t sensor_val;
  float temp;
}TSYS01_Instance;

enum TSYS01_State{
  INIT,
  CALB,
  MEAS
};

/* Function Prototype *********************************************************/
void TSYS01_Init(TSYS01_Instance*);
float TSYS01_GetTemp(TSYS01Packet_t*);
void TSYS01_CoeffCalibrate(TSYS01Packet_t*);
void TSYS01_ServiceRoutine(TSYS01_Instance*);
#endif