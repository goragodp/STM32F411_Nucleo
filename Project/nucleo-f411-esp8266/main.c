/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "esp8266.h"
#include "string.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
enum main_state
{
  WAIT_READY = 0,
  SEND_ATE0,
  SEND_CWMODE,
  SEND_CWJAP,
  SEND_CIFSR,
  SEND_CIPMUX,
  SEND_CIPSERVER,
  WAIT_CONN,
};

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
enum main_state main_state = WAIT_READY;
struct esp_cifsr_buf ipbuffer;
const char echoback[] = "Test Echo\r\n";
char cmd_buf[256];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void SysTick_Configuration(void);

void esp8266_main_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  switch(res)
  {
  case ESP_WIFI_CONNECTED:
    break;
    
  case ESP_WIFI_GOTIP:
    break;
    
  case ESP_WIFI_DISCONNECT:
    break;
    
  case ESP_CONNECT:
    break;
    
  case ESP_CONNECT_FAIL:
    break;
    
  case ESP_CLOSED:
    break;
  }
}

void esp8266_cipsend_payload_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  if(res == ESP_SEND_OK)
    printf("SEND PAYLOAD - SEND OK\r\n");
  else
    printf("SEND PAYLOAD - ERROR %d\r\n", res);
}
          
void esp8266_cipsend_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  if(res == ESP_OK)
  {
    printf("CIPSEND - OK\r\nSend payload\r\n");
    esp8266_send_payload(echoback, strlen(echoback), esp8266_cipsend_payload_callback);
  }
  else
    printf("CIPSEND - ERROR %d\r\n", res);
}

void esp8266_ipd_callback(struct esp_ipd_info *ipdinfo, uint8_t *packet, uint32_t packet_len)
{
  printf("CONID - %d\r\n", ipdinfo->conid);
  
  packet[packet_len] = 0x00;
  
  printf("%s\r\n", packet);
  
  sprintf(cmd_buf, "AT+CIPSEND=%d,%d\r\n", ipdinfo->conid, strlen(echoback));
  esp8266_send_command(cmd_buf, strlen(cmd_buf), esp8266_cipsend_callback);
}

          
void esp8266_ate0_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  if(res == ESP_OK)
    printf("ATE0 - OK\r\n");
  else
    printf("ATE0 - ERROR %d\r\n", res);
}

void esp8266_cwmode_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  if(res == ESP_OK)
    printf("CWMODE - OK\r\n");
  else
    printf("CWMODE - ERROR %d\r\n", res);
}

void esp8266_cwjap_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  if(res == ESP_OK)
    printf("CWJAP - OK\r\n");
  else
    printf("CWJAP - ERROR %d\r\n", res);
}

void esp8266_cifsr_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  if(res == ESP_OK)
  {
    printf("CIFSR - OK\r\n");
    esp8266_cisfr_parse((char*)msg, len, &ipbuffer);
    
    printf("%u.%u.%u.%u\r\n", ipbuffer.sta_ip[0], ipbuffer.sta_ip[1], ipbuffer.sta_ip[2], ipbuffer.sta_ip[3]);
  }
  else
    printf("CIFSR - ERROR %d\r\n", res);
}

void esp8266_cipmux_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  if(res == ESP_OK)
    printf("CIPMUX - OK\r\n");
  else
    printf("CIPMUX - ERROR %d\r\n", res);
}

void esp8266_cipserver_callback(enum esp_res res, uint8_t *msg, uint32_t len)
{
  if(res == ESP_OK)
    printf("CIPSERVER - OK\r\n");
  else
    printf("CIPSERVER - ERROR %d\r\n", res);
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  SysTick_Configuration();
  
  esp8266_init();
  
  while(1)
  {
    esp8266_process(esp8266_main_callback, esp8266_ipd_callback);
    
    switch(main_state)
    {
    case WAIT_READY:
      if(esp8266_is_ready())
        main_state = SEND_ATE0;
      
      break;
      
    case SEND_ATE0:
      if(esp8266_is_send_command_ready())
      {
        esp8266_send_command("ATE0\r\n", strlen("ATE0\r\n"), esp8266_ate0_callback);
        main_state = SEND_CWMODE;
      }
      break;
      
    case SEND_CWMODE:
      if(esp8266_is_send_command_ready())
      {
        esp8266_send_command("AT+CWMODE=1\r\n", strlen("AT+CWMODE=1\r\n"), esp8266_cwmode_callback);
        main_state = SEND_CWJAP;
      }
      break;
      
    case SEND_CWJAP:
      if(esp8266_is_send_command_ready())
      {
        esp8266_send_command("AT+CWJAP=\"WIFI_SSID\",\"WIFI_PASSWORD\"\r\n", strlen("AT+CWJAP=\"WIFI_SSID\",\"WIFI_PASSWORD\"\r\n"), esp8266_cwjap_callback);
        main_state = SEND_CIFSR;
      }
      break;
      
    case SEND_CIFSR:
      if(esp8266_is_send_command_ready())
      {
        esp8266_send_command("AT+CIFSR\r\n", strlen("AT+CIFSR\r\n"), esp8266_cifsr_callback);
        main_state = SEND_CIPMUX;
      }
      break;
      
    case SEND_CIPMUX:
      if(esp8266_is_send_command_ready())
      {
        esp8266_send_command("AT+CIPMUX=1\r\n", strlen("AT+CIPMUX=1\r\n"), esp8266_cipmux_callback);
        main_state = SEND_CIPSERVER;
      }
      break;
      
    case SEND_CIPSERVER:
      if(esp8266_is_send_command_ready())
      {
        esp8266_send_command("AT+CIPSERVER=1,9999\r\n", strlen("AT+CIPSERVER=1,9999\r\n"), esp8266_cipserver_callback);
        main_state = WAIT_CONN;
      }
      break;
      
    case WAIT_CONN:
      break;
    }
  }
}

/**
  * @brief  SysTick_Configuration
  * @param  None
  * @retval None
  */
static void SysTick_Configuration(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  
  /* Configure Systick clock source as HCLK */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

  /* SystTick configuration: an interrupt every 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config( ( RCC_Clocks.HCLK_Frequency / ( 1000 ) ) );
}

void SysTick_Handler(void)
{
  
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
