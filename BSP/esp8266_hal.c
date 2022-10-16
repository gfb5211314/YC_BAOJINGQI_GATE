#include "esp8266_hal.h"
#include "string.h"
#include "stdio.h"
#define  wifi_usart     hlpuart1
#define  RST_L        HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, GPIO_PIN_RESET);
#define  RST_H        HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, GPIO_PIN_SET);
//UART_HandleTypeDef hlpuart1;
//UART_HandleTypeDef huart1;
//DMA_HandleTypeDef hdma_lpuart1_rx;
USART_TYPE  ESP8266;
USART_TYPE  ESP8266_temp;

/***********************************************************
                        *发送HAL*
************************************************************/

//发送1字节到WIFI模块
void send_byte_to_wifi(uint8_t p)
{

    HAL_UART_Transmit(&wifi_usart, (uint8_t*)&p,1, 0xffff);

}
//发送一串字符到wifi模块,不能发送数据0
void wifi_send_string(uint8_t *p) {

    while(*p!= '\0') {

        send_byte_to_wifi(*p);
        p++;
    }
//	length=sizeof(data)/sizeof(data[0])
}
void send_string_to_wifi(uint8_t *p,uint8_t plen)
{
	 HAL_UART_Transmit(&wifi_usart, p,plen, 0xffff);
	
	
}
/*****************************************************************
                       *接收HAL*
******************************************************************/
//开启DMA接收空闲中断
void  WIFI_DMA_START()
{
//    HAL_UART_Receive_DMA(&wifi_usart,(uint8_t *)ESP8266.RX_pData, RX_LEN);  //不能启动打开
    __HAL_UART_ENABLE_IT(&wifi_usart, UART_IT_IDLE);
//		HAL_UART_Receive_IT(&wifi_usart,ESP8266_temp.RX_pData,1);		// 重新使能串口2接收中断
}


void ESP8266_tim_sucess()
{
//if( rec_sta_flag==1)
//				 {
//					   rec_sta_count++;
//             if(rec_sta_count>300)
//						 {							 
//                ESP8266.RX_flag=1;
//							 rec_sta_count=0;
//							 rec_sta_flag=0;
//							
//						 }
//			    }
}
//开启接收空闲中断
void  ESP8266_UsartReceive_IDLE()
{
    uint32_t temp=0;

    if((__HAL_UART_GET_FLAG(&wifi_usart,UART_FLAG_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&wifi_usart);
        HAL_UART_DMAStop(&wifi_usart);
        temp = wifi_usart.hdmarx->Instance->CNDTR;//	
        ESP8266.RX_Size = RX_LEN - temp;
        ESP8266.RX_flag=1;
       HAL_UART_Receive_DMA(&wifi_usart,(uint8_t *)ESP8266.RX_pData, RX_LEN);
    }
}
//
void LPUART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */
     ESP8266_UsartReceive_IDLE();

    /* USER CODE END USART3_IRQn 0 */
      HAL_UART_IRQHandler(&wifi_usart);
    /* USER CODE BEGIN USART3_IRQn 1 */
    /* USER CODE END USART3_IRQn 1 */
}
/********************************************************
                        *drive*
********************************************************/

void ESP8266_Rst ( void )
{

	 RST_L;
	 HAL_Delay( 500 ); 
	 RST_H;
	HAL_Delay(2000);
	HAL_Delay(2000);
  HAL_Delay(2000);
}
//
//只可以用于AT命令发送，字符为0，没有问题! 数据里面有零都不行
uint8_t atk_8266_send_cmd(uint8_t *cmd,uint8_t *ack,uint32_t waittime)
{
	     uint8_t sucess_state=0;
	    static  uint8_t comumtate=0; 
	    static  uint32_t wait_tim_sta=0; 
	    static  uint32_t wait_tim_end=0; 
	   switch(comumtate)
		 {
			 //进入发送状态
			 case 0 : 
				            wifi_send_string(cmd);
			 
			 HAL_UART_Receive_DMA(&wifi_usart,(uint8_t *)ESP8266.RX_pData, RX_LEN);//放在这里最好
			                    comumtate=1; 
			                    wait_tim_sta=HAL_GetTick();
			              printf("wait_tim_sta=%d\r\n",wait_tim_sta);
			               break;
			 //进入ack状态
			 case 1 :  if(ESP8266.RX_flag==1)
			         {
				 
				           printf("ack:%s\r\n",(uint8_t*)ESP8266.RX_pData);
				           if(strstr((const char*)ESP8266.RX_pData,(const char*)ack))
			            {
								          ESP8266.RX_flag=0;
								        	ESP8266.RX_Size=0;
								           sucess_state=1;
								             comumtate=0;
                      memset(ESP8266.RX_pData, 0, sizeof(ESP8266.RX_pData));					        
				           }
									    ESP8266.RX_flag=0;
								     
                      memset(ESP8266.RX_pData, 0, sizeof(ESP8266.RX_pData));	
			         }
							 else
							 {
								         wait_tim_end=HAL_GetTick();
//								       printf("wait_tim_end=%d\r\n",wait_tim_end);
								         if((wait_tim_end-wait_tim_sta)>waittime)
												 {
													 
													    comumtate=0;
													   
												 }								 
							 }
							 break;			 
		 }
              return sucess_state;
} 
void wifi_rec_data_process()
{
	 if(ESP8266.RX_flag==1)
	 {
		  
		 
		 
		 
		 
	 }
	
	
	
}
//启动模块进入透传
uint8_t wifi_Passthrough_mode()
{
	  uint8_t sucess_stetc=0;
	  static uint8_t wifi_state_num=0;
	   switch(wifi_state_num)
		 {
	
			 case 0 : 
				                if(atk_8266_send_cmd((uint8_t *)"ATE0\r\n",(uint8_t *)"OK",5000)==1)
											  {
												  wifi_state_num=1;											 
											  }
			   break;
			 case 1 :  
				                    printf("设置模式\r\n");
				 if(atk_8266_send_cmd((uint8_t *)"AT+CWMODE=1\r\n",(uint8_t *)"OK",5000)==1)
											 {
												  wifi_state_num=2;											 
											 }
			   break; 
											 
			 	 case 2 :       printf("链接路由\r\n");
					 if(atk_8266_send_cmd((uint8_t *)"AT+CWJAP=\"TP-LINK_WEIHOU\",\"wh-sfzbz\"\r\n\r\n",(uint8_t *)"OK",5000)==1)
											 {
							  wifi_state_num=3;											 
											 }
			   break; 
			  	 case 3 :        printf("123\r\n");
						 if(atk_8266_send_cmd((uint8_t *)"AT+CIPMUX=0\r\n",(uint8_t *)"OK",5000)==1)
											 {
							  wifi_state_num=4;											 
											 }
			   break; 
			  	 case 4 :      printf("链接IP地址\r\n");
						 if(atk_8266_send_cmd((uint8_t *)"AT+CIPSTART=\"TCP\",\"192.168.0.128\",6962\r\n",(uint8_t *)"OK",5000)==1)
											 {
							     wifi_state_num=5;											 
											 }
			   break; 
			  	 case 5 :         if(atk_8266_send_cmd((uint8_t *)"AT+CIPMODE=1\r\n",(uint8_t *)"OK",5000)==1)
											    {
							               wifi_state_num=6;											 
											    }
			   break; 
			 	  	 case 6 :         if(atk_8266_send_cmd((uint8_t *)"AT+CIPSEND\r\n",(uint8_t *)">",5000)==1)
											    {
														sucess_stetc=1;
							               wifi_state_num=7;											 
											    }
			   break; 		 
		 }
	     return  sucess_stetc;
}

