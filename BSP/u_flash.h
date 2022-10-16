#ifndef  __U_FLASH_H_
#define  __U_FLASH_H_
#include "main.h"

#define PAGE_SIZE                128
#define PAGE_NUM                 512
#define ADDR_FLASH_PAGE_505      0X08000000 + 128*504   //
#define ADDR_FLASH_PAGE_506      0X08000000 + 128*505   //
#define ADDR_FLASH_PAGE_507      0X08000000 + 128*506   //
#define ADDR_FLASH_PAGE_508      0X08000000 + 128*507   //
#define ADDR_FLASH_PAGE_509      0X08000000 + 128*508   //
#define ADDR_FLASH_PAGE_510      0X08000000 + 128*509   //
#define ADDR_FLASH_PAGE_511      0X08000000 + 128*510   //
#define ADDR_FLASH_PAGE_512      0X08000000 + 128*511   //最后一页



extern void Flash_Write_Num_Word(uint32_t WriteAddr, uint32_t * pBuffer, uint32_t NumToWrite );

/**
  * 函数功能: 读取指定地址的半字(16位数据)
  * 输入参数: faddr:读地址(此地址必须为2的倍数!!)
  * 返 回 值: 返回值:对应数据.
  * 说    明：无
  */
extern uint32_t U_STMFLASH_ReadWord ( uint32_t faddr );

/**
  * 函数功能: 从指定地址开始读出指定长度的数据
  * 输入参数: ReadAddr:起始地址
  *           pBuffer:数据指针
  *           NumToRead:半字(16位)数
  * 返 回 值: 无
  * 说    明：无
  */
extern void Flash_Read_Word( uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead )  ; 	

#endif