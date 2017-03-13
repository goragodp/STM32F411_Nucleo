#include "esp8266.h"
#include "string.h"

#define FIFO_SIZE               2048
#define RX_PROCESS_BUF_SIZE     1024
#define RX_PACKET_BUF_SIZE      1500    /* Ethernet v2 MTU size */

struct esp_txmode_info
{
  uint8_t       mode;
  uint8_t       sent;
  const char    *buf;
  uint32_t      len;
  const char    *payload;
  uint32_t      payloadlen;
  callback_t    callback;
  enum esp_res  last_res;
};

typedef struct esp_fifo
{
  unsigned char buf[FIFO_SIZE];
  unsigned char *bufend;
  unsigned char *head;
  unsigned char *tail;
  unsigned long cnt;
}esp_fifo_t;

static struct esp_fifo esp_rxfifo;
static struct esp_fifo esp_txfifo;

static uint8_t esp_ready = 0;
static uint8_t esp_rxpacketbuf[RX_PACKET_BUF_SIZE];
static uint8_t esp_rxprocbuf[RX_PROCESS_BUF_SIZE];
static uint32_t esp_rxprocinx = 0;
static uint32_t esp_rxprocread = 0;
static uint32_t esp_rxpacketinx = 0;
static struct esp_ipd_info ipdinfo;
static uint16_t ipdinfo_tmp[8];
static enum esp_res parse_res;

struct esp_txmode_info txmode_info;

// Private function ------------------------------------------------------------
static void esp_gpio_init(void);
static void esp_usart_init(void);
static void esp_nvic_init(void);
static int esp_lowlevel_write(const char *buf, int bufsize);

static uint32_t esp_ipaddr_u32(const char *ipaddr, uint8_t *iparray);
static uint8_t esp_mac_parse(const char *buf, uint8_t *mac);

static unsigned long esp_fifo_put(struct esp_fifo *fifo, const char *data, unsigned long len);
static unsigned long esp_fifo_get(struct esp_fifo *fifo, char *buf, unsigned long size);
static unsigned long esp_fifo_get_count(struct esp_fifo *fifo);

// *****************************************************************************
// esp_LowlevelInit
// 
// Initialize esp Lowlevel.
// *****************************************************************************
void esp8266_init(void)
{
  int reset_delay = 0xFFFF;
  
  esp_gpio_init();
  esp_usart_init();
  esp_nvic_init();
  
  esp_rxfifo.bufend = esp_rxfifo.buf + FIFO_SIZE;
  esp_rxfifo.head = esp_rxfifo.buf;
  esp_rxfifo.tail = esp_rxfifo.buf;
  esp_rxfifo.cnt = 0;
  
  esp_txfifo.bufend = esp_txfifo.buf + FIFO_SIZE;
  esp_txfifo.head = esp_txfifo.buf;
  esp_txfifo.tail = esp_txfifo.buf;
  esp_txfifo.cnt = 0;
  
  ipdinfo.state = ESPIPD_STATE_IDLE;
  
  GPIO_ResetBits(ESP_CS_GPIO_Port, ESP_CS_GPIO_Pin);
  
  /* Drive CH_PD Pin to low. */
  GPIO_ResetBits(ESP_PD_GPIO_Port, ESP_PD_GPIO_Pin);
  
  while(reset_delay--);
  
  /* Drive CH_PD Pin to high. */
  GPIO_SetBits(ESP_PD_GPIO_Port, ESP_PD_GPIO_Pin);
}

static int esp_lowlevel_write(const char *buf, int bufsize)
{
  int cnt;
  
  cnt = esp_fifo_put(&esp_txfifo, buf, bufsize);
  
  USART_ITConfig(ESP_USART, USART_IT_TXE, ENABLE);
  
  return cnt;
}

// *****************************************************************************
// esp_gpio_init
//
// esp GPIO Initialize.
// *****************************************************************************
static void esp_gpio_init(void)
{
  GPIO_InitTypeDef gpio_init_struct;
  
  ESP_USART_TX_GPIO_RCC_CMD(ESP_USART_TX_GPIO_RCC, ENABLE);
  ESP_USART_RX_GPIO_RCC_CMD(ESP_USART_RX_GPIO_RCC, ENABLE);
  ESP_RST_GPIO_RCC_CMD(ESP_RST_GPIO_RCC, ENABLE);
  ESP_PD_GPIO_RCC_CMD(ESP_PD_GPIO_RCC, ENABLE);
  ESP_CS_GPIO_RCC_CMD(ESP_CS_GPIO_RCC, ENABLE);
  
  gpio_init_struct.GPIO_Mode = GPIO_Mode_AF;
  gpio_init_struct.GPIO_OType = GPIO_OType_PP;
  gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init_struct.GPIO_PuPd = GPIO_PuPd_UP;
  
  gpio_init_struct.GPIO_Pin = ESP_USART_TX_GPIO_Pin;
  GPIO_Init(ESP_USART_TX_GPIO_Port, &gpio_init_struct);
  
  gpio_init_struct.GPIO_Pin = ESP_USART_RX_GPIO_Pin;
  GPIO_Init(ESP_USART_RX_GPIO_Port, &gpio_init_struct);
  
  GPIO_PinAFConfig(ESP_USART_TX_GPIO_Port, ESP_USART_TX_GPIO_PinSource, ESP_USART_TX_GPIO_AF);
  GPIO_PinAFConfig(ESP_USART_RX_GPIO_Port, ESP_USART_RX_GPIO_PinSource, ESP_USART_RX_GPIO_AF);
  
  gpio_init_struct.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init_struct.GPIO_OType = GPIO_OType_PP;
  gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init_struct.GPIO_PuPd = GPIO_PuPd_UP;
  
  gpio_init_struct.GPIO_Pin = ESP_PD_GPIO_Pin;
  GPIO_Init(ESP_PD_GPIO_Port, &gpio_init_struct);
  
  gpio_init_struct.GPIO_Pin = ESP_RST_GPIO_Pin;
  GPIO_Init(ESP_RST_GPIO_Port, &gpio_init_struct);
  
  gpio_init_struct.GPIO_Pin = ESP_CS_GPIO_Pin;
  gpio_init_struct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(ESP_CS_GPIO_Port, &gpio_init_struct);
}

// *****************************************************************************
// esp_usart_init
//
// esp USART Initialize.
// *****************************************************************************
static void esp_usart_init(void)
{
  USART_InitTypeDef usart_init_struct;
  
  ESP_USART_RCC_CMD(ESP_USART_RCC, ENABLE);
  
  usart_init_struct.USART_BaudRate = 115200;   
  usart_init_struct.USART_WordLength = USART_WordLength_8b;  
  usart_init_struct.USART_StopBits = USART_StopBits_1;   
  usart_init_struct.USART_Parity = USART_Parity_No ;
  usart_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(ESP_USART, &usart_init_struct);
  
  USART_ITConfig(ESP_USART, USART_IT_RXNE, ENABLE);
  
  USART_Cmd(ESP_USART, ENABLE);
}

// *****************************************************************************
// esp_nvic_init
//
// esp NVIC Initialize.
// *****************************************************************************
static void esp_nvic_init(void)
{
  NVIC_InitTypeDef nvic_init_struct;
  
  nvic_init_struct.NVIC_IRQChannel = ESP_USART_IRQn;
  nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 0x7;
  nvic_init_struct.NVIC_IRQChannelSubPriority = 0x00;
  nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init_struct);
}

// *****************************************************************************
// ESP8266_USART_IRQHandler
//
// esp8266 USART Interrupt IRQ.
// *****************************************************************************
void ESP_USART_IRQHandler(void)
{
  uint8_t TmpData;
  
  /* USART RX Not Empty Interrupt */
  if(USART_GetITStatus(ESP_USART, USART_IT_RXNE) != RESET)
  {
    USART_ClearITPendingBit(ESP_USART, USART_IT_RXNE);
    
    /* Receive data from RX Data register. */
    TmpData = USART_ReceiveData(ESP_USART);
    
    esp_fifo_put(&esp_rxfifo, (const char*)&TmpData, 1);
  }
  
  /* USART TX Empty Interrupt */
  else if(USART_GetITStatus(ESP_USART, USART_IT_TXE) != RESET)
  {
    USART_ClearITPendingBit(ESP_USART, USART_IT_TXE);
    
    if(esp_fifo_get_count(&esp_txfifo))
    {
      esp_fifo_get(&esp_txfifo, (char*)&TmpData, 1);
      USART_SendData(ESP_USART, TmpData);
    }
    else
    {
      USART_ITConfig(ESP_USART, USART_IT_TXE, DISABLE);
    }
    
  }
  else
  {
    USART_Cmd(ESP_USART, DISABLE);
    USART_Cmd(ESP_USART, ENABLE);
  }
}

/*
 * @brief  Put a data into circular buffer.
 * @param  cbuf: pointer to cbuf handle.
 * @param  data: pointer to a data.
 * @param  len: size of data in bytes.
 * @retval size of data in bytes that put into buffer.
 */
static unsigned long esp_fifo_put(struct esp_fifo *fifo, const char *data, unsigned long len)
{
  unsigned long n = 0;
  
  if(fifo)
  {
    while(fifo->cnt < FIFO_SIZE && len--)
    {
      *fifo->tail++ = *data++;
      fifo->cnt++;
      n++;
      
      if(fifo->tail >= fifo->bufend)
        fifo->tail = fifo->buf;
    }
  }
  
  return n;
}

/*
 * @brief  Get a data from circular buffer.
 * @param  cbuf: pointer to cbuf handle.
 * @param  buf: pointer to a buffer to store a data.
 * @param  size: size of buffer in bytes.
 * @retval size of data in bytes that get from buffer.
 */
static unsigned long esp_fifo_get(struct esp_fifo *fifo, char *buf, unsigned long size)
{
  unsigned long n = 0;
  
  if(fifo)
  {
    while(fifo->cnt && size--)
    {
      *buf++ = *fifo->head++;
      fifo->cnt--;
      n++;
      
      if(fifo->head >= fifo->bufend)
        fifo->head = fifo->buf;
    }
  }
  
  return n;
}

/*
 * @brief  Get a data from circular buffer.
 * @param  cbuf: pointer to cbuf handle.
 * @param  buf: pointer to a buffer to store a data.
 * @param  size: size of buffer in bytes.
 * @retval size of data in bytes that get from buffer.
 */
static unsigned long esp_fifo_get_count(struct esp_fifo *fifo)
{
  return fifo->cnt;
}

uint8_t esp8266_is_ready(void)
{
  return esp_ready;
}

uint8_t esp8266_is_send_command_ready(void)
{
  if(txmode_info.mode)
    return 0;
  else
    return 1;
}

void esp8266_process(callback_t main_callback, ipd_callback_t ipd_callback)
{
  char byte;
  const char *inbuf;
  
  if(esp_fifo_get_count(&esp_rxfifo))
  {
    esp_fifo_get(&esp_rxfifo, &byte, 1);
    
    esp_rxprocbuf[esp_rxprocinx++] = byte;
    
    if(byte == '\n' && ipdinfo.state != ESPIPD_STATE_RECV)
    {
      inbuf = (const char*)&esp_rxprocbuf[esp_rxprocread];
      parse_res = ESP_CONTINUE;
      
      /*
       * Standard Response
       */
      if(strncmp(inbuf,"OK\r\n",4) == 0)
      {
        parse_res = ESP_OK;
      }
      else if(strncmp(inbuf,"ERROR\r\n",7) == 0)
      {
        parse_res = ESP_ERROR;
      }
      else if(strncmp(inbuf,"no change\r\n",11) == 0)
      {
        parse_res = ESP_OK;
      }
      else if(strncmp(inbuf,"ready\r\n",7) == 0)
      {
        parse_res = ESP_READY;
        esp_ready = 1;
      }
      else if(strncmp(inbuf,"UNLINK\r\n",8) == 0)
      {
        parse_res = ESP_UNLINK;
      }
      else if(strncmp(inbuf,"FAIL\r\n",6) == 0)
      {
        parse_res = ESP_FAIL;
      }
      
      /*
       * WIFI Response
       */
      else if(strncmp(inbuf,"WIFI CONNECTED\r\n",16) == 0)
      {
        parse_res = ESP_WIFI_CONNECTED;
        main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      else if(strncmp(inbuf,"WIFI GOT IP\r\n",13) == 0)
      {
        parse_res = ESP_WIFI_GOTIP;
        main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      else if(strncmp(inbuf,"WIFI DISCONNECT\r\n",17) == 0)
      {
        parse_res = ESP_WIFI_DISCONNECT;
        main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      
      /*
       * TCP/IP Response
       */
      else if(strncmp(inbuf,"CONNECT\r\n",9) == 0)
      {
        parse_res = ESP_CONNECT;
        
        if(txmode_info.mode == 3)
          parse_res = ESP_CONTINUE;
        else
          main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      else if(strncmp(inbuf,"CONNECT FAIL\r\n",14) == 0)
      {
        parse_res = ESP_CONNECT_FAIL;
        
        if(txmode_info.mode == 3)
          parse_res = ESP_CONTINUE;
        else
          main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      else if(strncmp(&inbuf[1],",CONNECT\r\n",10) == 0)
      {
        parse_res = ESP_CONNECT;
        
        if(txmode_info.mode == 3)
          parse_res = ESP_CONTINUE;
        else
          main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      else if(strncmp(&inbuf[1],",CONNECT FAIL\r\n",15) == 0)
      {
        parse_res = ESP_CONNECT_FAIL;
        
        if(txmode_info.mode == 3)
          parse_res = ESP_CONTINUE;
        else
          main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      else if(strncmp(inbuf,"CLOSED\r\n",8) == 0)
      {
        parse_res = ESP_CLOSED;
        
        if(txmode_info.mode == 3)
          parse_res = ESP_CONTINUE;
        else
          main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      else if(strncmp(&inbuf[1],",CLOSED\r\n",9) == 0)
      {
        parse_res = ESP_CLOSED;
        
        if(txmode_info.mode == 3)
          parse_res = ESP_CONTINUE;
        else
          main_callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
      }
      else if(strncmp(inbuf,"SEND OK\r\n",9) == 0)
      {
        parse_res = ESP_SEND_OK;
      }
      /*
       * WIFI Response
       */
      else if((strncmp(inbuf,"AT+",3) == 0) || (strncmp(inbuf,"ATE",3) == 0))
      {
        parse_res = ESP_ECHO;
      }
      /*
       * Unknown response
       */
      else
      {
        parse_res = ESP_CONTINUE;
      }
      
      if(txmode_info.mode)
      {
        txmode_info.last_res = parse_res;
        
        if(parse_res == ESP_OK || parse_res == ESP_SEND_OK ||parse_res == ESP_ERROR || parse_res == ESP_FAIL)
        {
          txmode_info.mode = 0;
          txmode_info.buf = 0;
          txmode_info.len = 0;
          txmode_info.sent = 0;
          
          if(txmode_info.callback)
            txmode_info.callback(parse_res, esp_rxprocbuf, esp_rxprocinx);
        }
      }
      
      if(parse_res == ESP_CONTINUE)
      {
        esp_rxprocread = esp_rxprocinx;
      }
      else
      {
        esp_rxprocinx = 0;
        esp_rxprocread = 0;
      }
    }
    else
    {
      switch(ipdinfo.state)
      {
      case ESPIPD_STATE_IDLE:
        if( byte == '+' )
          ipdinfo.state = ESPIPD_STATE_GET_PLUS;
        else
          ipdinfo.state = ESPIPD_STATE_IDLE;
        break;
        
      case ESPIPD_STATE_GET_PLUS:
        if( byte == 'I' )
          ipdinfo.state = ESPIPD_STATE_GET_I;
        else if( byte == '+' )
          ipdinfo.state = ESPIPD_STATE_GET_PLUS;
        else
          ipdinfo.state = ESPIPD_STATE_IDLE;
        break;
        
      case ESPIPD_STATE_GET_I:
        if( byte == 'P' )
          ipdinfo.state = ESPIPD_STATE_GET_P;
        else if( byte == '+' )
          ipdinfo.state = ESPIPD_STATE_GET_PLUS;
        else
          ipdinfo.state = ESPIPD_STATE_IDLE;
        break;
        
      case ESPIPD_STATE_GET_P:
        if( byte == 'D' )
          ipdinfo.state = ESPIPD_STATE_GET_D;
        else if( byte == '+' )
          ipdinfo.state = ESPIPD_STATE_GET_PLUS;
        else
          ipdinfo.state = ESPIPD_STATE_IDLE;
        break;
        
      case ESPIPD_STATE_GET_D:
        if( byte == ',' )
        {
          ipdinfo.state = ESPIPD_STATE_PARSE_HDR;
          memset(ipdinfo_tmp, 0, sizeof(u16)*8);
        }
        else if( byte == '+' )
          ipdinfo.state = ESPIPD_STATE_GET_PLUS;
        else
          ipdinfo.state = ESPIPD_STATE_IDLE;
        break;
        
      case ESPIPD_STATE_PARSE_HDR:
        if( byte >= '0' && byte <= '9' )
          ipdinfo_tmp[ipdinfo_tmp[7]] = ipdinfo_tmp[ipdinfo_tmp[7]] * 10 + (byte - '0');
        else if(byte == ',' || byte == '.')
          ipdinfo_tmp[7]++;
        else if( byte == ':' )
        {
          ipdinfo_tmp[7]++;
          
          switch(ipdinfo_tmp[7])
          {
          case 1:
            ipdinfo.conid = 0;
            ipdinfo.len = ipdinfo_tmp[0];
            ipdinfo.remote_ipv4[0] = 0;
            ipdinfo.remote_ipv4[1] = 0;
            ipdinfo.remote_ipv4[2] = 0;
            ipdinfo.remote_ipv4[3] = 0;
            ipdinfo.remote_port = 0;
            ipdinfo.mode = 0;
            break;
            
          case 2:
            ipdinfo.conid = ipdinfo_tmp[0];
            ipdinfo.len = ipdinfo_tmp[1];
            ipdinfo.remote_ipv4[0] = 0;
            ipdinfo.remote_ipv4[1] = 0;
            ipdinfo.remote_ipv4[2] = 0;
            ipdinfo.remote_ipv4[3] = 0;
            ipdinfo.remote_port = 0;
            ipdinfo.mode = 0;
            break;
            
          case 6:
            ipdinfo.conid = 0;
            ipdinfo.len = ipdinfo_tmp[0];
            ipdinfo.remote_ipv4[0] = ipdinfo_tmp[1];
            ipdinfo.remote_ipv4[1] = ipdinfo_tmp[2];
            ipdinfo.remote_ipv4[2] = ipdinfo_tmp[3];
            ipdinfo.remote_ipv4[3] = ipdinfo_tmp[4];
            ipdinfo.remote_port = ipdinfo_tmp[5];
            ipdinfo.mode = 1;
            break;
            
          case 7:
            ipdinfo.conid = ipdinfo_tmp[0];
            ipdinfo.len = ipdinfo_tmp[1];
            ipdinfo.remote_ipv4[0] = ipdinfo_tmp[2];
            ipdinfo.remote_ipv4[1] = ipdinfo_tmp[3];
            ipdinfo.remote_ipv4[2] = ipdinfo_tmp[4];
            ipdinfo.remote_ipv4[3] = ipdinfo_tmp[5];
            ipdinfo.remote_port = ipdinfo_tmp[6];
            ipdinfo.mode = 1;
            break;
          }
          
          if(ipdinfo.len)
          {
            ipdinfo.state = ESPIPD_STATE_RECV;
            ipdinfo.remainlen = ipdinfo.len;
            esp_rxpacketinx = 0;
          }
          else
          {
            ipdinfo.state = ESPIPD_STATE_IDLE;
            ipdinfo.remainlen = 0;
          }
        }
        else if( byte == '+' )
          ipdinfo.state = ESPIPD_STATE_GET_PLUS;
        else
          ipdinfo.state = ESPIPD_STATE_IDLE;
        break;
        
      case ESPIPD_STATE_RECV:
        esp_rxpacketbuf[esp_rxpacketinx++] = byte;
        
        if(!(--ipdinfo.remainlen))
        {
          ipdinfo.state = ESPIPD_STATE_FINISH;
          ipd_callback(&ipdinfo, esp_rxpacketbuf, esp_rxpacketinx);
          ipdinfo.state = ESPIPD_STATE_IDLE;
        }
        break;
      }
    }
  }
  
  if(esp_ready)
  {
    if((txmode_info.mode >= 1 && txmode_info.mode <= 3) && !txmode_info.sent)
    {
      txmode_info.sent = 1;
      esp_lowlevel_write(txmode_info.buf, txmode_info.len);
    }
    else if(txmode_info.mode == 4 && !txmode_info.sent)
    {
      if(byte == '>')
      {
        txmode_info.sent = 1;
        esp_lowlevel_write(txmode_info.buf, txmode_info.len);
      }
    }
  }
}

int esp8266_send_command(const char *buffer, uint32_t len, callback_t callback)
{
  if(txmode_info.mode)
    return 0;
  
  if(!buffer || !len)
    return 0;
  
  if(strncmp(buffer, "AT+CIPSEND", 10) == 0)
    txmode_info.mode = 2;
  else if(strncmp(buffer, "AT+CIPSTART", 11) == 0 || strncmp(buffer, "AT+CIPCLOSE", 11) == 0)
    txmode_info.mode = 3;
  else
    txmode_info.mode = 1;
  
  txmode_info.sent = 0;
  txmode_info.buf = buffer;
  txmode_info.len = len;
  txmode_info.callback = callback;
  txmode_info.last_res = ESP_NULL;
  
  return 1;
}

int esp8266_send_payload(const char *buffer, uint32_t len, callback_t callback)
{
  if(txmode_info.mode)
    return 0;
  
  if(!buffer || !len)
    return 0;
  
  txmode_info.mode = 4;
  txmode_info.sent = 0;
  txmode_info.buf = buffer;
  txmode_info.len = len;
  txmode_info.callback = callback;
  txmode_info.last_res = ESP_NULL;
  
  return 1;
}

int esp8266_cisfr_parse(const char *buffer, int len, struct esp_cifsr_buf *parse_buf)
{
  memset(parse_buf, 0, sizeof(struct esp_cifsr_buf));
  
  do
  {
    /*
     * CIFSR Response
     */
    while(*buffer != '+' && len)
    {
      buffer++;
      len--;
    }
    
    if(strncmp(buffer,"+CIFSR:APIP,",12) == 0)
    {
      buffer += 13;
      len -= 13;
      esp_ipaddr_u32(buffer,parse_buf->ap_ip);
    }
    else if(strncmp(buffer,"+CIFSR:APMAC,",13) == 0)
    {
      buffer += 14;
      len -= 14;
      esp_mac_parse(buffer,parse_buf->ap_mac);
    }
    else if(strncmp(buffer,"+CIFSR:STAIP,",13) == 0)
    {
      buffer += 14;
      len -= 14;
      esp_ipaddr_u32(buffer,parse_buf->sta_ip);
    }
    else if(strncmp(buffer,"+CIFSR:STAMAC,",14) == 0)
    {
      buffer += 15;
      len -= 15;
      esp_mac_parse(buffer,parse_buf->sta_mac);
    }
  }while(len < 12);
  
  return 0;
}

/*
 * @brief  Convert an ascii IP address to an uint32_t.
 * @param  ipaddr: IP address in ascii (e.g. "127.0.0.1")
 * @retval IP address in big endian (network) byte order.
 */
static uint32_t esp_ipaddr_u32(const char *ipaddr, uint8_t *iparray)
{
  uint32_t index;
  uint32_t tmp;
  uint32_t tmp2;
  uint32_t tmp3;
  uint8_t ip[4];
  
  /* IP parse */
  index = 0;
  tmp = 0;
  tmp2 = 4;
  
  while(tmp2--)
  {
    ip[tmp] = 0;
    tmp3 = 3;
    while(tmp3--)
    {
      if(ipaddr[index] >= '0' && ipaddr[index] <= '9')
        ip[tmp] = ip[tmp]*10 + (ipaddr[index++] - 0x30);
      else
        break;
    }
    
    if(ipaddr[index] == '.' && tmp2)
    {
      index++;
      tmp++;
    }
  }
  
  if(tmp == 3)
  {
    iparray[0] = ip[0];
    iparray[1] = ip[1];
    iparray[2] = ip[2];
    iparray[3] = ip[3];
    
    return (ip[0] << 24) | (ip[1] << 16) | (ip[2] << 8) | ip[3];
  }
  else
    return 0;
}

/*
 * @brief  Convert an ascii MAC address to a 48-bits unsigned integer (array of uint8_t).
 * @param  buf: String buffer.
 * @param  mac: MAC address buffer.
 * @retval If success, 1 is returned.
 *         Error, 0 is returned.
 */
static uint8_t esp_mac_parse(const char* buf, uint8_t *mac)
{
  uint32_t index;
  uint32_t tmp;
  uint32_t tmp2;
  
  /* MAC parse */
  index = 0;
  tmp = 0;
  for(tmp = 0; tmp < 6; tmp++)
  {
    mac[tmp] = 0;
    for(tmp2 = 0; tmp2 < 2; tmp2++)
    {
      if(buf[index] >= 'a' && buf[index] <= 'f')
        mac[tmp] = mac[tmp]*16 + (buf[index++] - 'a' + 10);
      else if(buf[index] >= 'A' && buf[index] <= 'F')
        mac[tmp] = mac[tmp]*16 + (buf[index++] - 'A' + 10);
      else if(buf[index] >= '0' && buf[index] <= '9')
        mac[tmp] = mac[tmp]*16 + (buf[index++] - 0x30);
      else
        break;
    }
    
    if(tmp2 < 2)
      break;
    
    if(buf[index] == ':')
      index++;
    else
      break;
  }
  
  if(tmp < 6 || (tmp == 6 && tmp2 < 2))
  {
    for(tmp = 0; tmp < 6; tmp++)
      mac[tmp] = 0;
    
    return 0;
  }
  else
    return 1;
}