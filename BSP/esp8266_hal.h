#ifndef __ESP8266_HAL_H__
#define __ESP8266_HAL_H__

//添加hal库头文件
#include "main.h"
//添加串口头文件
#include "usart.h"

//串口移植


#define  RX_LEN  100

typedef struct
{
    uint8_t  RX_flag;        //IDLE receive flag
    uint16_t RX_Size;          //receive length
	  uint16_t count;
    uint16_t tem_RX_Size;          //receive length
    uint8_t  RX_pData[RX_LEN];
    uint8_t  tem_RX_pData[RX_LEN];
    uint8_t  TX_pData[100];
} USART_TYPE;



void  WIFI_DMA_START(void);
uint8_t atk_8266_send_cmd(uint8_t *cmd,uint8_t *ack,uint32_t waittime);
void wifi_send_string(uint8_t *p);

#endif
