#include "main_app.h"
#include "SX127X_Driver.h"
#include "SX127X_Hal.h"
#include "stdio.h"
#include "bsp_eeprom.h"
#include "string.h"
#include "ether_hal.h"
#include "stm_flash.h"


#include <stdlib.h>
//业务层

#define   Manufacturer_ID_1        0xfe
#define   Manufacturer_ID_2        0xef

#define   Manufacturer_ID_3        0xef 
#define   Manufacturer_ID_4        0xfe

#define join_network   0x01    //入网请求
#define join_network_state  0x02 //入网状态发送
#define Power_report    0x03    //设备电池电量上报
#define Equipment_tamper    0x04   //设备防拆上报
#define device_number     0x05     //设备号上报
#define xiaojing_pass_num      0x06     //消警密码上报
#define user_password_fun     0x07    //用户密码上报
#define dev_run_state     0x08    //正常状态上报
#define set_xiaojing_password  0x09   //设备消警密码
#define set_userpassword         0x10  //后台设置用户密码
#define rang_fun              0x12 //设备报警上报
#define xiaojing_num          0x13   //消警指令下发
//extern USART_TYPE  ESP8266;

extern ETH_TYPE  ether_st;
uint32_t Fre[6] = {433800000,470800000, 494600000, 510000000, 868000000, 915000000};   //收发频率
uint8_t communication_states=0; //业务状态
uint8_t TXbuffer[50] = {0};
uint8_t RXbuffer[50] = {0};
uint16_t T_Cnt = 0;
uint32_t factory_parameter_flag=0;    //出厂是否设置
uint8_t user_password[6]={0};        //用户密码
uint8_t user_temp_password[6]={0};        //用户密码
uint8_t xiaojing_password[6]={1,2,3,2,2,2};    //消警密码
uint8_t dev_num[4]={0,0};     //设备号
uint8_t pack_len = 0;
uint8_t data_wifi_pa=0;
uint8_t wifiTXbuffer[50] = {0};
uint32_t xintiao_count=0;
uint32_t xintiao_flag=0;
uint8_t  ruwang_flag=0;  //入网标志
//uint8_t gate_way_flag=0;
extern	    uint32_t chuchang_flag;
extern uint8_t  local_eth_ip[30];
//uint8_t   local_eth_port[30]="at+NRPort0=6962\r\n";
extern uint8_t  Remote_eth_ip[30];
extern uint8_t  Remote_eth_port[30];
extern uint32_t  u32_local_eth_ip[1];
extern uint32_t  u32_Remote_eth_ip[1];
extern uint32_t  u32_Remote_eth_port[1];


uint8_t product_key[30]={0,0,0,0,0,0,0,1};//一定要初始化，要不写读会发生问题
uint8_t baojingqi_product_key[30]={0,0,0,0,0,0,0,1};//一定要初始化，要不写读会发生问题

extern uint32_t u32_product_key[2];
extern uint32_t  u32_dev_num[1]; 
void set_txrx_datalen(uint8_t datalen);
void data_encrypt(int length,unsigned char *input,
                   unsigned char *output);
void data_decrypt(int length,unsigned char *input,
                   unsigned char *output);
uint8_t pin_pack(uint8_t *dev,uint8_t dev_type,uint8_t function_num,uint8_t *data_pack);
void u8_ip_to_u32_ip_more(uint8_t *ipbuf,uint32_t *ipuf,uint16_t u8_len);
void u8_ip_to_u32_ip(uint8_t *ipbuf,uint32_t *ipuf);
void u32_ip_to_u8_ip(uint8_t *ipbuf,uint32_t *ipuf,uint16_t u32_len);



#define  LORA_GATE_PRODUCTY             "gS7P5ubq"
#define  LORA_BAOJINGQI_PRODUCTY        "LBJ32RWK"

void Param_Init()
{
		 memcpy(product_key, LORA_GATE_PRODUCTY,   sizeof(LORA_GATE_PRODUCTY) );
	  memcpy(baojingqi_product_key, LORA_BAOJINGQI_PRODUCTY,   sizeof(LORA_BAOJINGQI_PRODUCTY) );
	
}
enum DemoInternalStates
{
    APP_RNG = 0, // nothing to do (or wait a radio interrupt)
    RX_DONE,
    TX_DONE,
    TX_ING,
    APP_IDLE,
};
//LORA 初始化配置
void app_lora_config_init()
{
	   uint8_t astate=0;
    G_LoRaConfig.LoRa_Freq = Fre[0];   //中心频率470MHz
    G_LoRaConfig.BandWidth = BW125KHZ;    //BW = 125KHz  BW125KHZ
    G_LoRaConfig.SpreadingFactor = SF09;  //SF = 9
    G_LoRaConfig.CodingRate = CR_4_6;     //CR = 4/6
    G_LoRaConfig.PowerCfig = 15;          //19±1dBm
    G_LoRaConfig.MaxPowerOn = true;       //最大功率开启
    G_LoRaConfig.CRCON = true;            //CRC校验开启?
    G_LoRaConfig.ExplicitHeaderOn = true; //Header开启
    G_LoRaConfig.PayloadLength = 64;      //数据包长度
     astate= SX127X_Lora_init();
	   printf("astate=%d\r\n",astate);
    if(astate!= NORMAL)	 //无线模块初始化
    {
        while(1)
        {
           printf("lora init fail\r\n");
					HAL_Delay(500);
        }
    }
	   	//设置LORA为接收
		 
		  	SX127X_StartRx();
		  communication_states=APP_IDLE;
}
uint32_t rand_data=0;
void lora_send_data(uint8_t *lora_data,uint8_t lora_datalen)
{
	       rand_data=rand()%40+1;
	      rand_data=rand_data*100;
	      HAL_Delay(rand_data);
	      SX127X_StandbyMode();    
     set_txrx_datalen(lora_datalen);
	/***********************加密*************************/
	   data_encrypt(lora_datalen,lora_data,TXbuffer);	        
	/****************************************************/
	   SX127X_TxPacket(TXbuffer);
	   communication_states = APP_IDLE;
}

extern uint8_t sn_code[12];
//   uint8_t flag;
void lora_process()
{
	    if(ruwang_flag==1)
			{
//	    lora_send_data("123",5);   
	      uint16_t  send_eth_len=0;
	 switch(communication_states)
        {
               case APP_IDLE:
            if(DIO0_GetState() == GPIO_PIN_SET)
        {
          uint8_t flag;
        SX127X_Read(REG_LR_IRQFLAGS, &flag);
        SX127X_Write(REG_LR_IRQFLAGS, 0xff); //clear flags
        if(flag & RFLR_IRQFLAGS_TXDONE)
        {
            communication_states = TX_DONE;
					//  printf("send sucess\r\n");
				//	  send_string_to_eth((uint8_t*)"txsucees",8);
        }
        else if(flag & RFLR_IRQFLAGS_RXDONE)
        {
				
            communication_states = RX_DONE;
				//	printf("rx sucess\r\n");
				//	 send_string_to_eth((uint8_t*)"rxsuceess",8);
        }
       }     
            break;

        case TX_DONE:
      //    SX127X_StandbyMode();   //待机模式   打开接收模式
              SX127X_StartRx();
            communication_states = APP_IDLE;
            break;
					
				case RX_DONE :
					 SX127X_Read(REG_LR_NBRXBYTES, &G_LoRaConfig.PayloadLength); //获取数据长度
//					 set_txrx_datalen(G_LoRaConfig.PayloadLength);
				     SX127X_RxPacket(RXbuffer);		
				   //  SX127X_StandbyMode();  //切换状态清空FIFO，要不收到250个字节，会出错
			   //   SX127X_SleepMode(); //睡眠模式	
//						 for(uint16_t i=0;i<G_LoRaConfig.PayloadLength;i++)
//			 	 {
//					printf("RXbuffer[%d]=%02x\r\n",i,RXbuffer[i]);
//									
//				 }
					/*******************解密**********************************************/
						data_decrypt(G_LoRaConfig.PayloadLength,RXbuffer, RXbuffer);
				/******************************************************************/
//             printf("网关收到来自与报警器的数据\r\n");				
//				 for(uint16_t i=0;i<G_LoRaConfig.PayloadLength;i++)
//			 	 {
//					printf("RXbuffer[%d]=%02x\r\n",i,RXbuffer[i]);
//									
//				 }
       //   send_string_to_eth(RXbuffer,G_LoRaConfig.PayloadLength);
				 //添加productkey
				 for(uint8_t i=0;i<8;i++)
				 {
					 ether_st.RX_pData[i]=baojingqi_product_key[i];
				 }
		//添加sn码
				 	 for(uint8_t i=0;i<12;i++)
				 {
					 ether_st.RX_pData[i+8]=RXbuffer[i];
				 }
				 //添加包头
				
					 ether_st.RX_pData[8+12]=0xfe;  //+12添加SN码//20
					 ether_st.RX_pData[9+12]=0xef;  //+12//21
				 //设备号
				 for(uint8_t i=0;i<G_LoRaConfig.PayloadLength-2-12;i++)//19-12-2
			 	 {
				 	ether_st.RX_pData[i+10+12]=RXbuffer[i+2+12];//去掉0x5a ,a5包头+sncode+12
									
				 }//5
				 //
//				 printf("G_LoRaConfig.PayloadLength=%d\r\n",G_LoRaConfig.PayloadLength); 
				 //包尾
				  ether_st.RX_pData[10+G_LoRaConfig.PayloadLength-2]=0xef;  //8+12+17=
				  ether_st.RX_pData[11+G_LoRaConfig.PayloadLength-2]=0xfe;
				 
				   send_eth_len=G_LoRaConfig.PayloadLength+10;//+12 是sn
//				    for(uint16_t i=0;i<send_eth_len;i++)
//				 {
//					 printf("ether_st.RX_pData[%d]=%02x\r\n",i,ether_st.RX_pData[i]);
//					 
//					 
//				 }
				//   send_string_to_eth(RXbuffer,G_LoRaConfig.PayloadLength);
				  	 for(uint8_t i=0;i<G_LoRaConfig.PayloadLength-12;i++)
				 {
					   RXbuffer[i]=RXbuffer[i+12];
				 }
				 
		   
				 
			//	 send_string_to_eth(RXbuffer,G_LoRaConfig.PayloadLength-12);
				 //接收数据处理和网关入网  重要说明 RXbuffer[1]=a5是网关lora发出，过滤掉多个网关3
				   if((RXbuffer[0]==0x5a&&RXbuffer[1]!=0xa5&&RXbuffer[4]==0x01))
					 {   
						 
						    
						   switch(RXbuffer[5])
							 {
								 //接收到设备入网求，通过wifi转发到后台
								 case  0x01 :
									 printf("设备入网请求收到,转发到Web\r\n");
									   send_string_to_eth(ether_st.RX_pData,send_eth_len);  break;
								 //设备入网状态，转发到后台
								 case  0x02 : 
                    printf("设备入网状态收到,转发到Web\r\n");									 
									  send_string_to_eth(ether_st.RX_pData,send_eth_len);  break;
								 //电池电量上报
								 case 0x03 : 
									 	 send_string_to_eth(ether_st.RX_pData,send_eth_len); break;
								 //设备被拆掉
								 case 0x04 : 
									  send_string_to_eth(ether_st.RX_pData,send_eth_len); break;
								 //设备号上报
								 case 0x05 : 
									    send_string_to_eth(ether_st.RX_pData,send_eth_len);  break;   
								 //消警密码上报
								  case 0x06 : 
									     send_string_to_eth(ether_st.RX_pData,send_eth_len); break;  
//						   //用户密码上报			
						      case 0x07 : 
									    send_string_to_eth(ether_st.RX_pData,send_eth_len);  break;  
									//设备状态上报
									case 0x08 : 
									    send_string_to_eth(ether_st.RX_pData,send_eth_len); break;  
									//有人按下报警上报
								  case 0x12 : 
									     send_string_to_eth(ether_st.RX_pData,send_eth_len);  break;  
									 case 0x13 : 
									     send_string_to_eth(ether_st.RX_pData,send_eth_len);  break;
									 //开始布防
									 case 0x20: 
									     send_string_to_eth(ether_st.RX_pData,send_eth_len);  break;
									 //设备撤防
									 	 case 0x21: 
									     send_string_to_eth(ether_st.RX_pData,send_eth_len);  break;
								}
							}	
	         memset(RXbuffer, 0, sizeof(RXbuffer));	
					memset(ether_st.RX_pData, 0, sizeof(ether_st.RX_pData));
            SX127X_StartRx();
					  communication_states = APP_IDLE;
					 break;
						}
				}
}
//检查出厂参数
uint8_t check_factory_parameter()
{
	       uint8_t value=0;
	      STMFLASH_Read (  0x800f400, (uint32_t* )&factory_parameter_flag, 1);
					   printf("factory_parameter_flag=%d",factory_parameter_flag);
					           if(factory_parameter_flag ==3)
										 {
											     
											 value=1; //出厂
										 }
										 else
										 {
											value=2; //没有出厂
										 }
		
				 return value;
}
void set_txrx_datalen(uint8_t datalen)
{
  G_LoRaConfig.PayloadLength = datalen;      //数据包长度
}
//组包
uint8_t pin_pack(uint8_t *dev,uint8_t dev_type,uint8_t function_num,uint8_t *data_pack)
{
	uint8_t index=0;
	  TXbuffer[index++]=0x5a;
	  TXbuffer[index++]=0xa5;  //厂家识别码
	  TXbuffer[index++]=dev[0];  //设备码
	  TXbuffer[index++]=dev[1];  //设备码
	  TXbuffer[index++]=dev_type;  //设备类型
  	TXbuffer[index++]=function_num; //功能码
	   for(uint8_t i=0;i<strlen((const char *)data_pack);i++)
	{
	  TXbuffer[index++]=data_pack[i]; //功能码
	}
	  return index;
}
//uint8_t wifi_connect_pin_pack(uint8_t *dev,uint8_t dev_type,uint8_t function_num,uint8_t *data_pack,uint8_t data_pack_len)
//{
//	    uint8_t index=0;
//	  wifiTXbuffer[index++]=dev[0];  //设备码
//	  wifiTXbuffer[index++]=dev[1];  //设备码
//	  wifiTXbuffer[index++]=0x02;  //设备类型
//  	wifiTXbuffer[index++]=function_num; //功能码
//	   for(uint8_t i=0;i<data_pack_len;i++)
//	{
//	  wifiTXbuffer[index++]=data_pack[i]; //
//	}
//	  return index;
//}
extern uint8_t sn_code[12];
uint8_t wifi_connect_pin_pack(uint8_t *dev,uint8_t dev_type,uint8_t function_num,uint8_t *data_pack,uint8_t data_pack_len,uint8_t *proctukey)
{
	    uint16_t index=0;
		//产品密钥
	     for(uint16_t i=0;i<8;i++)
	{
	  wifiTXbuffer[index++]=proctukey[i]; //productkey
	}
		     for(uint16_t i=0;i<12;i++)
	{
	  wifiTXbuffer[index++]=sn_code[i]; //productkey
	}
	  wifiTXbuffer[index++]=Manufacturer_ID_1;
	  wifiTXbuffer[index++]=Manufacturer_ID_2;
	  wifiTXbuffer[index++]=dev[0];  //设备码
	  wifiTXbuffer[index++]=dev[1];  //设备码
	  wifiTXbuffer[index++]=0x02;  //设备类型
  	wifiTXbuffer[index++]=function_num; //功能码
	   for(uint16_t i=0;i<data_pack_len;i++)
	{
	  wifiTXbuffer[index++]=data_pack[i]; //数据
	}
    wifiTXbuffer[index++]=Manufacturer_ID_3;
	  wifiTXbuffer[index++]=Manufacturer_ID_4;
	  return index;
}
uint8_t eth_ack_flag=0;
//处理任务
void wifi_process()
{
	
	   static uint8_t wifi_comum=0; //  15 先配置设备地址     0直接设备入网
	  
	   switch(wifi_comum)
		 {
			 //检查设备是否有设备号
			 case 0 : 
				 //没有出厂
			         if(check_factory_parameter()==2)
							 {
								   send_string_to_eth((uint8_t *)"请求出厂设置123\r\n",17); 
								      wifi_comum=1;
								  printf("请求出厂设置\r\n");
							 }
							 //已经出厂
							 else  if(check_factory_parameter()==1)
							 {
								  STMFLASH_Read (  0x800f488, (uint32_t* )&u32_dev_num, 1);
						     	dev_num[0]=*(uint8_t *)&u32_dev_num;
							  	dev_num[1]=*(((uint8_t *)&u32_dev_num[0])+1);
	                printf("dev_num0=%x",dev_num[0]);
								  printf("dev_num1=%x",dev_num[1]);
								       wifi_comum=3;
							 }
							 break;
						//配置网关IP地址
			 case 15 :
				  
					    if(chuchang_flag!=3)
							{
								
//							   send_string_to_eth("!=3",3);	
//							printf("配置网络\r\n");
				         if(ether_st.RX_flag==1)
							  {
//								 //减去序列号
//									for(uint8_t i=0;i<ether_st.RX_Size-8;i++)
//									{
//										
//										ether_st.RX_pData[i]=ether_st.RX_pData[i+8];
//									}
//									
//									  send_string_to_eth( (uint8_t *)ether_st.RX_pData,ether_st.RX_Size);
								   if(ether_st.RX_pData[2]==0x02)
									 {
										     switch(ether_st.RX_pData[3])
												 {												
																 //设置本机IP地址
														case 0x03 :
															printf("设置IP地址\r\n");
															     memset(local_eth_ip, 0, sizeof(local_eth_ip));	
																	  for(uint8_t i=0;i<4;i++)
																	 {
																		  
																		 local_eth_ip[i]= ether_st.RX_pData[4+i];
																	 }
																	 u8_ip_to_u32_ip(local_eth_ip,u32_local_eth_ip);
                             STMFLASH_Write (  0x800f428, (uint32_t* )u32_local_eth_ip, 1)	; //写本机地址																	 
																      data_wifi_pa=1;
				                      wifi_connect_pin_pack(dev_num,0x02,0x03,&data_wifi_pa,1,product_key);
												 send_string_to_eth(wifiTXbuffer,wifi_connect_pin_pack(dev_num,0x02,0x03,&data_wifi_pa,1,product_key));				
														break;
																	 //设置远程IP地址
													case 0x04 :
																printf("设置远程IP地址\r\n");
															     memset(Remote_eth_ip, 0, sizeof(Remote_eth_ip));	
																	  for(uint8_t i=0;i<4;i++)
																	 {
																		  
																		 Remote_eth_ip[i]= ether_st.RX_pData[4+i];
																	 }
																	 u8_ip_to_u32_ip(Remote_eth_ip,u32_Remote_eth_ip);
                             STMFLASH_Write (  0x800f448, (uint32_t* )u32_Remote_eth_ip, 1)	; //写远程IP地址																	 
                                        		      data_wifi_pa=1;
				                      wifi_connect_pin_pack(dev_num,0x02,0x04,&data_wifi_pa,1,product_key);
												 send_string_to_eth(wifiTXbuffer,wifi_connect_pin_pack(dev_num,0x02,0x04,&data_wifi_pa,1,product_key));		
														break;
																	 //设置远程端口
					                	case 0x05 :
																printf("设置远程IP端口\r\n");
															     memset(Remote_eth_port, 0, sizeof(Remote_eth_port));	
																	  for(uint8_t i=0;i<4;i++)
																	 {
																		  
																		 Remote_eth_port[i]= ether_st.RX_pData[4+i];
																	 }
																	 u8_ip_to_u32_ip(Remote_eth_port,u32_Remote_eth_port);
                             STMFLASH_Write (  0x800f468, (uint32_t* )u32_Remote_eth_port, 1)	; //写远程端口地址																	 
														                       	chuchang_flag=3; //设置成功
														         STMFLASH_Write (  0x800f510, (uint32_t* )&chuchang_flag, 1)	;
																	 		      data_wifi_pa=1;
				                      wifi_connect_pin_pack(dev_num,0x02,0x05,&data_wifi_pa,1,product_key);
												 send_string_to_eth(wifiTXbuffer,wifi_connect_pin_pack(dev_num,0x02,0x05,&data_wifi_pa,1,product_key));		
																	        HAL_Delay(100);
														             HAL_NVIC_SystemReset(); //复位重启
																	 																	 
																	 //设置产品密钥
																break;	 
																	  case 0x06 :
															    	printf("设置Productkey\r\n");
															     memset(product_key, 0, sizeof(product_key));	
																	  for(uint8_t i=0;i<8;i++)
																	 {
																		  
																		 product_key[i]= ether_st.RX_pData[4+i];
																		 printf("product_key[%d]=%d\r\n",i,product_key[i]);
						         						    }
																	 
//																	 u8_ip_to_u32_ip(product_key,u32_product_key);
																	 u8_ip_to_u32_ip_more(product_key,u32_product_key,2);
                                  STMFLASH_Write ( 0x800f500, (uint32_t* )u32_product_key, 2)	; //写序列号		
//   																STMFLASH_Read (  0x800f500, (uint32_t* )u32_product_key, 2)	; //读
                               	u32_ip_to_u8_ip(product_key,u32_product_key,2);
																		
                                        for(uint8_t i=0;i<8;i++)
                                     	{
		                                 printf("%c",product_key[i]);
		
	                                    }			
																	 		      data_wifi_pa=1;
				                      wifi_connect_pin_pack(dev_num,0x02,0x06,&data_wifi_pa,1,product_key);
												 send_string_to_eth(wifiTXbuffer,wifi_connect_pin_pack(dev_num,0x02,0x06,&data_wifi_pa,1,product_key));		
				
														break;
															 
																							
												}
									}
									    memset(ether_st.tem_RX_pData, 0, sizeof(ether_st.tem_RX_pData));	
											  memset(ether_st.RX_pData, 0, sizeof(ether_st.RX_pData));	
									 ether_st.RX_flag=0;
								}		
							}
                       else
											 {

                             wifi_comum=0;
											 }												 
                   break;
							 //请求后台派发设备号
			 case 1 :
				                   
				              data_wifi_pa=1;
				        wifi_connect_pin_pack(dev_num,0x02,0x01,&data_wifi_pa,1,product_key);
			       printf("wifilen1=%d\r\n",wifi_connect_pin_pack(dev_num,0x02,0x01,&data_wifi_pa,1,product_key));
			          for(uint8_t i=0;i<wifi_connect_pin_pack(dev_num,0x02,0x01,&data_wifi_pa,1,product_key);i++)
			           {
									 
								   printf("	TXbuffer[i]=%02x",wifiTXbuffer[i]);
								 }
								 //后面处理2条发送数据，解决网关入网问题
                     send_string_to_eth(wifiTXbuffer,wifi_connect_pin_pack(dev_num,0x02,0x01,&data_wifi_pa,1,product_key));
								      
							  	     eth_ack_flag=1;
			                  wifi_comum=16;
		                 ether_st.RX_flag=0;//开机，清空数据buf
										 memset(ether_st.RX_pData, 0, sizeof(ether_st.RX_pData));	
			                   break;
			 case 16:
			       if(eth_ack_flag==2)
						 {
							  wifi_comum=1;
							 
						 }
				       if(ether_st.RX_flag==1)
							    {									

										 	for(uint8_t i=0;i<ether_st.RX_Size-20;i++)
									{
										
										ether_st.RX_pData[i]=ether_st.RX_pData[i+20];   //fe ef ...数据  把产品ductkey去掉和sn   8+12=20
//											 printf("ether_st.RX_pData[%d]=%02x",i,ether_st.RX_pData[i]);
									}
									//第一次收到数据，发生数据错位，前面第一个位置是00 
								send_string_to_eth(ether_st.RX_pData,ether_st.RX_Size-20);
								 //网关的信息
								     if(ether_st.RX_pData[4]==0x02)  //fe ef 01 02 05 01
									   {
										     switch(ether_st.RX_pData[5])
												 {												
													  case 0x01 :  
															   if(factory_parameter_flag!=3)
																 {
																	 
																	 printf("get dev id\r\n");
														   dev_num[0]=ether_st.RX_pData[6]; 
													     dev_num[1]=ether_st.RX_pData[7];
//													    EEPROM_WriteBytes(dev_num, 10, 2);
														    printf(" dev_num[0]=%d", dev_num[0]);
														    printf(" dev_num[1]=%d", dev_num[1]);
												             data_wifi_pa=1;
				                 printf("wifilen=%d\r\n",wifi_connect_pin_pack(dev_num,0x02,0x02,&data_wifi_pa,1,product_key));
			       for(uint8_t i=0;i<wifi_connect_pin_pack(dev_num,0x02,0x02,&data_wifi_pa,1,product_key);i++)
			           {
									 
								   printf("	 wifiTXbuffer[i]=%02x",wifiTXbuffer[i]);
								 }
							        	 factory_parameter_flag=3;
//								   STMFLASH_Write (  0x800f400, (uint32_t* )&factory_parameter_flag, 1);
//								  //把设备号写进去
//								 		           	 u8_ip_to_u32_ip(dev_num,u32_dev_num);
//                             STMFLASH_Write (0x800f488, (uint32_t* )u32_dev_num, 1)	; 		
                                  eth_ack_flag=0;
													         wifi_comum=4;
								               
							                   }
													         break;
								 	            case 0x02 :  
                                 if((factory_parameter_flag==3)&&(dev_num[0]==ether_st.RX_pData[2])&&(dev_num[1]==ether_st.RX_pData[3]))
																 {
																	 
																	    
																	       wifi_comum=3;
																	       eth_ack_flag=0;
																 printf("设备绑定成功\r\n");
																 }
													        break;
												      }
								 //增加一个WIFI出厂设置密码
										 }
										 else
										 {
											 
											 		  memset(ether_st.RX_pData, 0, sizeof(ether_st.RX_pData));	
										 }
										   memset(ether_st.tem_RX_pData, 0, sizeof(ether_st.tem_RX_pData));	
											  memset(ether_st.RX_pData, 0, sizeof(ether_st.RX_pData));	
									 ether_st.RX_flag=0;
									}
		                      	 break;

		 	case 3 :
				// 发送心跳包
//			        gate_way_flag=1;
			      if( xintiao_flag==1)
					   	{
								ruwang_flag=1;//入网成功
						   	 printf("发送心跳\r\n");
			                 data_wifi_pa=1;
				         wifi_connect_pin_pack(dev_num,0x02,0x08,&data_wifi_pa,1,product_key);
								 send_string_to_eth(wifiTXbuffer,wifi_connect_pin_pack(dev_num,0x02,0x08,&data_wifi_pa,1,product_key));	
							   xintiao_flag=0;							
						  }
			 //业务处理
			           if(ether_st.RX_flag==1)
							 {
								 //如果收到888，重新设置设备参数
								    if((ether_st.RX_pData[0]==8)&&(ether_st.RX_pData[1]==8)&&(ether_st.RX_pData[2]==8))
										{
											
											
											                 	chuchang_flag=1; //设置成功
														         STMFLASH_Write (  0x800f510, (uint32_t* )&chuchang_flag, 1)	;
											          HAL_Delay(100);
											               STMFLASH_Read (  0x800f510, (uint32_t* )&chuchang_flag, 1)	;
										       	 HAL_Delay(100);
											   send_string_to_eth( (uint8_t *)&chuchang_flag,1);
																	 	    factory_parameter_flag=0;//重新分配设备地址
								            STMFLASH_Write (  0x800f400, (uint32_t* )&factory_parameter_flag, 1);
																						          HAL_Delay(100);
											               STMFLASH_Read (  0x800f400, (uint32_t* )&factory_parameter_flag, 1)	;
											 send_string_to_eth( (uint8_t *)&factory_parameter_flag,1);
				                      wifi_connect_pin_pack(dev_num,0x02,0x05,&data_wifi_pa,1,product_key);
												 send_string_to_eth(ether_st.RX_pData,3);		
																	        HAL_Delay(100);
														             HAL_NVIC_SystemReset(); //复位重启 
											
											
										}
								   
								 			

					       //去掉prodcut和包头，包尾+sncode
										for(uint8_t i=0;i<ether_st.RX_Size-12-12;i++)//8+12+2+2
									{
										
										   ether_st.RX_pData[i]=ether_st.RX_pData[i+10+12];//从设备号开启存放  8+12+2  
									}
									
								 //网关的信息
								   if((ether_st.RX_pData[2]==0x02)&&(ether_st.RX_pData[0]==dev_num[0])&&(ether_st.RX_pData[1]==dev_num[1]))
									 {
										 
										   printf("收到网关消息\r\n");
						 
									 }
									 //lora数据
									 else if(ether_st.RX_pData[2]==0x01)  //00 00 01 01 01
											{
												       printf("收到lora消息\r\n");
												        ether_st.tem_RX_pData[0]=0x5a;
														    ether_st.tem_RX_pData[1]=0xa5;
												    ether_st.RX_Size=ether_st.RX_Size-8-12-2;//sncode 8+12+2 
												   switch(ether_st.RX_pData[3])
													 {	 
														 case 0x01 :
															 
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																																				
																			 }
//                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
//                                       {
//	                                 printf(" ESP8266.tem_RX_pData[%d]=%02x",i,ether_st.tem_RX_pData[i]);
//																			 }
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);   
														 break;
																			  case 0x02 :
															 
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																	
																				
																			 }
//                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
//                                       {
//	                           printf(" ESP8266.tem_RX_pData[%d]=%02x",i,ether_st.tem_RX_pData[i]);
//																			 }
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);   
														 break;
														 //lora 上传电量信息 ，WIFI转发
														 case 0x03 :
															
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																	
																				
																			 }
//                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
//                                       {
//	                           printf(" ESP8266.tem_RX_pData[%d]=%02x",i,ether_st.tem_RX_pData[i]);
//																			 }
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
																			 	 //lora 防拆应答 ，WIFI转发
														 case 0x04 :
															 
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }

	
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
																	 case 0x05 :
													
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }

	
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
																			 		 case 0x06 :
															
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }

	
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
																			 						 		 case 0x07 :
														
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }

	
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
																			  						 		 case 0x08 :
														
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }

	
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
																			 	  						 		 case 0x09 :
														
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }

	
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
		               	 case 0x10 :
															
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }

	
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
								 		 case 0x12 :
											  printf("收到后台报警指令应答\r\n");
														
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
												case 0x13 :
														  printf("收到后台报警指令下发\r\n");
														
                                      for(uint8_t i=0;i<ether_st.RX_Size;i++)
                                       {
																				 
																				 ether_st.tem_RX_pData[i+2]=ether_st.RX_pData[i];
																				 
																			 }

	
                                       lora_send_data(ether_st.tem_RX_pData,ether_st.RX_Size);     
														   break;
																			 
												  case 0x20 :  
													    
												    break;
												
													case 0x21 :  
													    
												    break;
													 }
															 
                           															 
											}
											//设置IP地址
							else
							{
								
								
								
								
							}
										
									  memset(ether_st.tem_RX_pData, 0, sizeof(ether_st.tem_RX_pData));	
											  memset(ether_st.RX_pData, 0, sizeof(ether_st.RX_pData));	
									 ether_st.RX_flag=0;
								 } 
							 break;
								 //网关入网状态发送
            case 4 :
									       data_wifi_pa=1;
				              wifi_connect_pin_pack(dev_num,0x02,0x02,&data_wifi_pa,1,product_key);
												 send_string_to_eth(wifiTXbuffer,   wifi_connect_pin_pack(dev_num,0x02,0x02,&data_wifi_pa,1,product_key));	
			                   wifi_comum=20;  
                       eth_ack_flag=1;//开启应答
                   break;	
						case 20 : 	
							   if(eth_ack_flag==2)
						 {
							  wifi_comum=4;
							 
						 }
				        if(ether_st.RX_flag==1)
							    {									
											 //如果收到888，重新设置设备参数
								    if((ether_st.RX_pData[0]==8)&&(ether_st.RX_pData[1]==8)&&(ether_st.RX_pData[2]==8))
										{
											
											
											                 	chuchang_flag=1; //设置成功
														         STMFLASH_Write (  0x800f510, (uint32_t* )&chuchang_flag, 1)	;
											          HAL_Delay(100);
											               STMFLASH_Read (  0x800f510, (uint32_t* )&chuchang_flag, 1)	;
											           HAL_Delay(100);
											      send_string_to_eth( (uint8_t *)&chuchang_flag,1);
																	 	    factory_parameter_flag=0;//重新分配设备地址
								            STMFLASH_Write (  0x800f400, (uint32_t* )&factory_parameter_flag, 1);
																						          HAL_Delay(100);
											               STMFLASH_Read (  0x800f400, (uint32_t* )&factory_parameter_flag, 1)	;
											 send_string_to_eth( (uint8_t *)&factory_parameter_flag,1);
				                      wifi_connect_pin_pack(dev_num,0x02,0x05,&data_wifi_pa,1,product_key);
												 send_string_to_eth(ether_st.RX_pData,3);		
																	        HAL_Delay(100);
														             HAL_NVIC_SystemReset(); //复位重启 
											
											
										}
										 	for(uint8_t i=0;i<ether_st.RX_Size-20;i++)
									{
										
										ether_st.tem_RX_pData[i]=ether_st.RX_pData[i+20];
									}
											 	for(uint8_t i=0;i<ether_st.RX_Size-20;i++)
									{
										
										ether_st.RX_pData[i]=ether_st.tem_RX_pData[i];
									}
								 //网关的信息
								     if(ether_st.RX_pData[4]==0x02)
									   {
										     switch(ether_st.RX_pData[5])
												 {												
													  case 0x01 :  
															   if(factory_parameter_flag!=3)
																 {
																	 
																	 printf("get dev id\r\n");
														   dev_num[0]=ether_st.RX_pData[6]; 
													     dev_num[1]=ether_st.RX_pData[7];
//													    EEPROM_WriteBytes(dev_num, 10, 2);
														    printf(" dev_num[0]=%d", dev_num[0]);
														    printf(" dev_num[1]=%d", dev_num[1]);
												             data_wifi_pa=1;
//				                 printf("wifilen=%d\r\n",wifi_connect_pin_pack(dev_num,0x02,0x02,&data_wifi_pa,1,product_key));
//			       for(uint8_t i=0;i<wifi_connect_pin_pack(dev_num,0x02,0x02,&data_wifi_pa,1,product_key);i++)
//			           {
//									 
//								   printf("	 wifiTXbuffer[i]=%02x",wifiTXbuffer[i]);
//								 }
							        	 factory_parameter_flag=3;
//								   STMFLASH_Write (  0x800f400, (uint32_t* )&factory_parameter_flag, 1);
//								  //把设备号写进去
//								 		           	 u8_ip_to_u32_ip(dev_num,u32_dev_num);
//                             STMFLASH_Write (0x800f488, (uint32_t* )u32_dev_num, 1)	; 		
                                eth_ack_flag=0;
													         wifi_comum=4;
								               
							                   }
													         break;
								 	            case 0x02 :  
                                 if((factory_parameter_flag==3)&&(dev_num[0]==ether_st.RX_pData[2])&&(dev_num[1]==ether_st.RX_pData[3]))
																 {
																	 
																	    
																	       wifi_comum=3;
																	       eth_ack_flag=0;
																 printf("设备绑定成功\r\n");
																 }
													        break;
												      }
								 //增加一个WIFI出厂设置密码
										 }
										 else
										 {
											 
											 		  memset(ether_st.RX_pData, 0, sizeof(ether_st.RX_pData));	
										 }
										   memset(ether_st.tem_RX_pData, 0, sizeof(ether_st.tem_RX_pData));	
											  memset(ether_st.RX_pData, 0, sizeof(ether_st.RX_pData));	
									 ether_st.RX_flag=0;
									}
		                      	 break;


 
						
								   
								 
							 }
		  memset(wifiTXbuffer, 0, sizeof(wifiTXbuffer));	
							 		
		 }
