#include "stm32f4xx.h"
#include "core_cm4.h"


/**
  * @brief  SHT11 I2C Interface pins
  */
#define SHT11_I2C                       I2C1
#define SHT11_I2C_CLK                   RCC_APB1Periph_I2C1

#define SHT11_I2C_SCK_PIN               GPIO_Pin_8                 /* PB.10 */
#define SHT11_I2C_SCK_GPIO_PORT         GPIOB                        /* GPIOB */
#define SHT11_I2C_SCK_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define SHT11_I2C_SCK_SOURCE            GPIO_PinSource8
#define SHT11_I2C_SCK_AF                GPIO_AF_I2C1

#define SHT11_I2C_SDA_PIN               GPIO_Pin_9                  /* PB.7 */
#define SHT11_I2C_SDA_GPIO_PORT         GPIOB                        /* GPIOB */
#define SHT11_I2C_SDA_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define SHT11_I2C_SDA_SOURCE            GPIO_PinSource9
#define SHT11_I2C_SDA_AF                GPIO_AF_I2C1

#define SHT11_FLAG_TIMEOUT             ((uint32_t)0x1000)
#define SHT11_LONG_TIMEOUT             ((uint32_t)(10 * SHT11_FLAG_TIMEOUT))  

#define SHT11_ADDR                    0
#define SHT11_MEASURE_CMD             0x03


__IO uint32_t  time_delay = 0;
uint16_t temp_data = 0;
uint8_t check_sum = 0;
float convert_temp = 0;

void Delay(__IO uint32_t t, char unit) {
    if  (unit == 'u')
      time_delay = t;
    else if (unit == 'm')
      time_delay = t * 1000;
    else
      return;
}

/* read write funtions */
void I2C_start(I2C_TypeDef*, uint8_t, uint8_t);
void I2C_write(I2C_TypeDef*, uint8_t);
uint8_t I2C_read_ack(I2C_TypeDef*);
uint8_t I2C_read_nack(I2C_TypeDef*);
void I2C_stop(I2C_TypeDef*);
void init_I2C(void);


volatile uint32_t cycle = 0;
int main()
{
    //Enable cycle counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
   //SysTick_Config(SystemCoreClock / 168);
   init_I2C();
   

   while(1){
     cycle = DWT->CYCCNT;
     I2C_start(SHT11_I2C, SHT11_ADDR << 1, I2C_Direction_Transmitter);
     I2C_write(SHT11_I2C, SHT11_MEASURE_CMD);
     I2C_stop(SHT11_I2C);
     
     /*Read Data*/
     I2C_start(SHT11_I2C, SHT11_ADDR << 1, I2C_Direction_Receiver);
     temp_data |= (uint16_t)(I2C_read_ack(SHT11_I2C) << 8);
     temp_data |= I2C_read_ack(SHT11_I2C);
     /*Read checksum*/
     check_sum = I2C_read_nack(SHT11_I2C);
     /*Stop*/
     I2C_stop(SHT11_I2C);
     
     convert_temp = -40.1 - (0.04 * temp_data);
   
   }
}

void init_I2C(void){
  
  GPIO_InitTypeDef GPIO_InitStruct;
  I2C_InitTypeDef I2C_InitStruct;
  
  // enable APB1 peripheral clock for I2C1
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  // enable clock for SCL and SDA pins
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // we are going to use PB6 and PB7
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;     // set pins to alternate function
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    // set GPIO speed
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;     // set output to open drain --> the line has to be only pulled low, not driven high
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;     // enable pull up resistors
  GPIO_Init(GPIOB, &GPIO_InitStruct);         // init GPIOB
  
  // Connect I2C1 pins to AF  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); // SCL
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA
  
  // configure I2C1 
  I2C_InitStruct.I2C_ClockSpeed = 100000;     // 100kHz
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;     // I2C mode
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
  I2C_InitStruct.I2C_OwnAddress1 = 0x10;      // own address, not relevant in master mode
  I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;   // disable acknowledge when reading (can be changed later on)
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
  I2C_Init(I2C1, &I2C_InitStruct);        // init I2C1
  
  
  // enable I2C1
  I2C_Cmd(I2C1, ENABLE);
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
  // wait until I2C1 is not busy anymore
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
  // Send I2C1 START condition 
  I2C_GenerateSTART(I2Cx, ENABLE);
    
  // wait for I2C1 EV5 --> Slave has acknowledged start condition
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    
  // Send slave Address for write 
  I2C_Send7bitAddress(I2Cx, address, direction);
    
  /* wait for I2C1 EV6, check if 
   * either Slave has acknowledged Master transmitter or
   * Master receiver mode, depending on the transmission
   * direction
   */ 
  if(direction == I2C_Direction_Transmitter){
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  }
  else if(direction == I2C_Direction_Receiver){
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  }
}

/* This function transmits one byte to the slave device
 * Parameters:
 *    I2Cx --> the I2C peripheral e.g. I2C1 
 *    data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
  I2C_SendData(I2Cx, data);
  // wait for I2C1 EV8_2 --> byte has been transmitted
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
  // enable acknowledge of recieved data
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  // wait until one byte has been received
  while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
  // read data from I2C data register and return data byte
  uint8_t data = I2C_ReceiveData(I2Cx);
  return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
  // disabe acknowledge of received data
  // nack also generates stop condition after last byte received
  // see reference manual for more info
  I2C_AcknowledgeConfig(I2Cx, DISABLE);
  I2C_GenerateSTOP(I2Cx, ENABLE);
  // wait until one byte has been received
  while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
  // read data from I2C data register and return data byte
  uint8_t data = I2C_ReceiveData(I2Cx);
  return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
  // Send I2C1 STOP Condition 
  I2C_GenerateSTOP(I2Cx, ENABLE);
}

/*
*Callback and IRQ Handler
*/

void Systick_Hanlder(void){
  
  (time_delay != 0)?(time_delay--):(time_delay);
}
