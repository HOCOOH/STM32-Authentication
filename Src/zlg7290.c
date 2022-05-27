#include "zlg7290.h"

/* Maximum Timeout values for flags and events waiting loops. These timeouts are
not based on accurate values, they just guarantee that the application will 
not remain stuck if the I2C communication is corrupted.
You may modify these timeout values depending on CPU frequency and application
conditions (interrupts routines ...). */   
#define I2C_Open_FLAG_TIMEOUT         ((uint32_t)0x1000)

#define I2C_Open_LONG_TIMEOUT         ((uint32_t)0xffff)

#define HAL_I2C_RW_THRESHOLD 4
__IO uint32_t  I2CTimeout = I2C_Open_LONG_TIMEOUT;


/*******************************************************************************
* Function Name  : I2C_24C64_Read
* Description    : 
* Input          : 
* Output         : 
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_ZLG7290_Read(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num)
{
	uint8_t memread_cnt = 0;
    while(HAL_I2C_Mem_Read (I2Cx ,I2C_Addr,addr,I2C_MEMADD_SIZE_8BIT,buf,num,I2CTimeout) != HAL_OK ) {
		memread_cnt++;
		if (memread_cnt > HAL_I2C_RW_THRESHOLD) {
			MX_I2C1_Init();
			memread_cnt = 0;
		}
	}
}

/*******************************************************************************
* Function Name  : I2C_24C64_WriteOneByte
* Description    : 
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_ZLG7290_WriteOneByte(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value)
{   
	uint8_t memwrite_cnt = 0;
	while( HAL_I2C_Mem_Write(I2Cx, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &value, 0x01, I2CTimeout) != HAL_OK ) {
		memwrite_cnt++;
		if (memwrite_cnt > HAL_I2C_RW_THRESHOLD) {
			MX_I2C1_Init();
			memwrite_cnt = 0;
		}
	}
}

/*******************************************************************************
* Function Name  : I2C_24C64_Write
* Description    : 
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_ZLG7290_Write(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num)
{
	while(num--)
	{
    	I2C_ZLG7290_WriteOneByte(I2Cx, I2C_Addr,addr++,*buf++);
		HAL_Delay(5);
	}
}

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
