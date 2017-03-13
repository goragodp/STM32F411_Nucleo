/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ESP8266_H
#define __ESP8266_H

#include "stm32f4xx.h"

#ifdef __cplusplus
 extern "C" {
#endif 

#define ESP_USART                       USART1
#define ESP_USART_RCC                   RCC_APB2Periph_USART1
#define ESP_USART_RCC_CMD               RCC_APB2PeriphClockCmd

#define ESP_USART_TX_GPIO_Pin           GPIO_Pin_9
#define ESP_USART_TX_GPIO_Port          GPIOA
#define ESP_USART_TX_GPIO_PinSource     GPIO_PinSource9
#define ESP_USART_TX_GPIO_AF            GPIO_AF_USART1
#define ESP_USART_TX_GPIO_RCC           RCC_AHB1Periph_GPIOA
#define ESP_USART_TX_GPIO_RCC_CMD       RCC_AHB1PeriphClockCmd

#define ESP_USART_RX_GPIO_Pin           GPIO_Pin_10
#define ESP_USART_RX_GPIO_Port          GPIOA
#define ESP_USART_RX_GPIO_PinSource     GPIO_PinSource10
#define ESP_USART_RX_GPIO_AF            GPIO_AF_USART1
#define ESP_USART_RX_GPIO_RCC           RCC_AHB1Periph_GPIOA
#define ESP_USART_RX_GPIO_RCC_CMD       RCC_AHB1PeriphClockCmd

#define ESP_PD_GPIO_Pin           GPIO_Pin_0
#define ESP_PD_GPIO_Port          GPIOB
#define ESP_PD_GPIO_RCC           RCC_AHB1Periph_GPIOB
#define ESP_PD_GPIO_RCC_CMD       RCC_AHB1PeriphClockCmd

#define ESP_RST_GPIO_Pin          GPIO_Pin_0
#define ESP_RST_GPIO_Port         GPIOB
#define ESP_RST_GPIO_RCC          RCC_AHB1Periph_GPIOB
#define ESP_RST_GPIO_RCC_CMD      RCC_AHB1PeriphClockCmd

#define ESP_CS_GPIO_Pin           GPIO_Pin_12
#define ESP_CS_GPIO_Port          GPIOB
#define ESP_CS_GPIO_RCC           RCC_AHB1Periph_GPIOB
#define ESP_CS_GPIO_RCC_CMD       RCC_AHB1PeriphClockCmd

#define ESP_USART_IRQn                  USART1_IRQn
#define ESP_USART_IRQHandler            USART1_IRQHandler

/* esp response def */
typedef enum esp_res
{
  ESP_NULL = 0x00,
  ESP_OK,
  ESP_SEND_READY,
  ESP_SEND_OK,
  ESP_SENDBUF_OK,
  ESP_SENDBUF_FAIL,
  ESP_SENDBUF_FULL,
  ESP_READY,
  ESP_ERROR,
  ESP_FAIL,
  ESP_TIMEOUT,
  ESP_UNLINK,
  
  ESP_CONTINUE = 0x20,
  ESP_INQUEUE,
  ESP_ENQUEUE_FAIL,
  ESP_INPROCESS,
  ESP_ECHO,
  ESP_BLANK,
  ESP_SEND_PAYLOAD_FINISH,
  ESP_IPD,
  
  ESP_WIFI_CONNECTED = 0x40,
  ESP_WIFI_GOTIP,
  ESP_WIFI_DISCONNECT,
    
  ESP_CLOSED = 0x50,
  ESP_CONNECT,
  ESP_CONNECT_FAIL,
}esp_res_t;

/* IPD state def */
typedef enum esp_ipd_state
{
  ESPIPD_STATE_IDLE = 0,
  ESPIPD_STATE_GET_PLUS,
  ESPIPD_STATE_GET_I,
  ESPIPD_STATE_GET_P,
  ESPIPD_STATE_GET_D,
  ESPIPD_STATE_PARSE_HDR,
  ESPIPD_STATE_RECV,
  ESPIPD_STATE_FINISH
}esp_ipd_state_t;

/* IPD infomation structure def */
struct esp_ipd_info
{
  uint8_t mode;                         /* IPD Mode */
  enum esp_ipd_state state;             /* IPD parsing state */
  long conid;                           /* Connection ID */
  uint16_t len;                         /* Payload length */
  uint16_t remote_port;                 /* Remote port */
  uint8_t remote_ipv4[4];               /* Remote IP address */
  uint16_t headerlen;                   /* IPD header length */
  uint16_t remainlen;                   /* IPD remain length */
};

struct esp_cifsr_buf
{
  uint8_t sta_ip[4];
  uint8_t sta_mac[6];
  uint8_t ap_ip[4];
  uint8_t ap_mac[6];
};

typedef void (*callback_t)(enum esp_res,uint8_t*,uint32_t);
typedef void (*ipd_callback_t)(struct esp_ipd_info*,uint8_t*,uint32_t);


void esp8266_init(void);
uint8_t esp8266_is_ready(void);
uint8_t esp8266_is_send_command_ready(void);
void esp8266_process(callback_t main_callback, ipd_callback_t ipd_callback);
int esp8266_send_command(const char *buffer, uint32_t len, callback_t callback);
int esp8266_send_payload(const char *buffer, uint32_t len, callback_t callback);
int esp8266_cisfr_parse(const char *buffer, int len, struct esp_cifsr_buf *parse_buf);

#ifdef __cplusplus
}
#endif

#endif /* __ESP8266_H */