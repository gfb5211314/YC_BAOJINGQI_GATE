/**
  ******************************************************************************

  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm_flash.h"
#include "stdio.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
  #define STM_SECTOR_SIZE  128 //字节



/* 私有变量 ------------------------------------------------------------------*/
#if STM32_FLASH_WREN	//如果使能了写 
static uint32_t STMFLASH_BUF [128/4];//最多是2K字节
static FLASH_EraseInitTypeDef EraseInitStruct;
#endif
	void FLASH_If_Init(void)
{
  /* Unlock the Program memory */
  HAL_FLASH_Unlock();

  /* Clear all FLASH flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP  | FLASH_FLAG_WRPERR);
  /* Unlock the Program memory */
  HAL_FLASH_Lock();
}
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 读取指定地址的半字(16位数据)
  * 输入参数: faddr:读地址(此地址必须为2的倍数!!)
  * 返 回 值: 返回值:对应数据.
  * 说    明：无
  */
uint16_t STMFLASH_ReadHalfWord ( uint32_t faddr )
{
	return *(__IO uint16_t*)faddr; 
}


/**
  * 函数功能: 读取指定地址的半字(16位数据)
  * 输入参数: faddr:读地址(此地址必须为2的倍数!!)
  * 返 回 值: 返回值:对应数据.
  * 说    明：无
  */
uint32_t STMFLASH_ReadWord ( uint32_t faddr )
{
	return *(__IO uint32_t*)faddr; 
}

#if STM32_FLASH_WREN	//如果使能了写   
/**
  * 函数功能: 不检查的写入
  * 输入参数: WriteAddr:起始地址
  *           pBuffer:数据指针
  *           NumToWrite:字(32位)数
  * 返 回 值: 无
  * 说    明：无
  */
void STMFLASH_Write_NoCheck ( uint32_t WriteAddr, uint32_t * pBuffer, uint32_t NumToWrite )   
{ 			 		 
	uint16_t i;	
	
	for(i=0;i<NumToWrite;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,pBuffer[i]);
	  WriteAddr+=4;                                    // 一个地址一个字节,地址增加4.
	}  
} 

/**
  * 函数功能: 从指定地址开始写入指定长度的数据
  * 输入参数: WriteAddr:起始地址(此地址必须为2的倍数!!)
  *           pBuffer:数据指针
  *           NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
  * 返 回 值: 无
  * 说    明：无
  */
void STMFLASH_Write ( uint32_t WriteAddr, uint32_t * pBuffer, uint32_t NumToWrite )	
{
	uint8_t sate=0;
		uint8_t sate1=0;
  uint32_t SECTORError = 0;
	uint32_t secoff;	   //扇区内偏移地址(32位字计算)
	uint32_t secremain; //扇区内剩余地址(32位字计算)	   
 	uint16_t i;    
	uint32_t secpos;	   //扇区地址
	uint32_t offaddr;   //去掉0X08000000后的地址
	
	if(WriteAddr<FLASH_BASE||(WriteAddr>=(FLASH_BASE+1024*STM32_FLASH_SIZE)))
	{	
		printf("非法地址\r\n");
	  return;//非法地址
	}
	
	HAL_FLASH_Unlock();						//解锁
	
	offaddr=WriteAddr-FLASH_BASE;		//实际偏移地址.
		printf("实际偏移地址offaddr=%d\r\n",offaddr);
	secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
	printf("扇区地址secpos=%d\r\n",secpos);
	secoff=(offaddr%STM_SECTOR_SIZE)/4;		//在扇区内的偏移(4个字节为基本单位.)
		printf("在扇区内的偏移secoff=%d\r\n",secoff);
	secremain=STM_SECTOR_SIZE/4-secoff;		//扇区剩余空间大小   
	printf("扇区剩余空间大小secremain=%d\r\n",secremain);
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
	
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/4);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
//			printf("chachu12\r\n");
			if(STMFLASH_BUF[secoff+i]!=0XFFFFFFFF)break;//需要擦除  	  
		}

		if(i<secremain)//需要擦除
		{
       __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP  | FLASH_FLAG_WRPERR);   //擦除之前先清标志，（一定要加这个）
//			printf("chachu\r\n");
      //擦除这个扇区
      /* Fill EraseInit structure*/
      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.PageAddress   = secpos*STM_SECTOR_SIZE+FLASH_BASE;
      EraseInitStruct.NbPages       = 1;
      sate=HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);


  if (sate != HAL_OK)
  {
		   sate1=HAL_FLASH_GetError();
    /* Error occurred while page erase */
  printf("sate1=%d\n",sate1);
  }
			for(i=0;i<secremain;i++)//复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/4);//写入整个扇区  
		}
    else
    {
      STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		}
    if(NumToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0 	 
		   	pBuffer+=secremain;  	//指针偏移
			WriteAddr+=secremain;	//写地址偏移	   
		   	NumToWrite-=secremain;	//字节(32位)数递减
			if(NumToWrite>(STM_SECTOR_SIZE/4))secremain=STM_SECTOR_SIZE/4;//下一个扇区还是写不完
			else secremain=NumToWrite;//下一个扇区可以写完了
		}	 
	};	
	HAL_FLASH_Lock();//上锁
}
#endif

/**
  * 函数功能: 从指定地址开始读出指定长度的数据
  * 输入参数: ReadAddr:起始地址
  *           pBuffer:数据指针
  *           NumToRead:半字(16位)数
  * 返 回 值: 无
  * 说    明：无
  */
void STMFLASH_Read ( uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead )   	
{
	uint16_t i;
	
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
