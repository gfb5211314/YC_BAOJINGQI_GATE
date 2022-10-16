#ifndef __ETHER_HAL_H__
#define __ETHER_HAL_H__


#include "main.h"


#define  data_len  200

typedef struct
{
    uint8_t  RX_flag;        //IDLE receive flag
    uint16_t RX_Size;          //receive length
	  uint16_t count;
    uint16_t tem_RX_Size;          //receive length
    uint8_t  RX_pData[data_len];
    uint8_t  tem_RX_pData[data_len];
    uint8_t  TX_pData[data_len];
}ETH_TYPE;

void send_string_to_eth(uint8_t *p,uint16_t plen);
uint8_t eth_init(void);
void ETH_Rst ( void );
 void  ETH_DMA_START(void);
void eth_at_open(void); 
#endif
