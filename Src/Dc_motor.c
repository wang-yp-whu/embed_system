#include "Dc_motor.h"

#define I2C_Open_LONG_TIMEOUT         ((uint32_t)0xffff)

 __IO uint32_t  I2CTimeout1 = I2C_Open_LONG_TIMEOUT;


/*******************************************************************************
* Function Name  : I2C_WriteOneByte
* Description    : 
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_DC_Motor_WriteOneByte(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value)
{   
    HAL_StatusTypeDef result;
    int retry = 0;
    
    do {
        //result = HAL_I2C_Mem_Write(I2Cx, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &value, 0x01, 1);
        retry++;
        if(retry > 1) {
            printf("I2C Write Failed after 3 retries\r\n");
            break;
        }
    } while(result != HAL_OK);
}

/*******************************************************************************
* Function Name  : I2C_Write
* Description    : 
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_DC_Motor_Write(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num)
{
	while(num--)
	{
		
    I2C_DC_Motor_WriteOneByte(I2Cx, I2C_Addr,addr++,*buf++);
		HAL_Delay(5);
		
	}
}
/*******************************************************************************
* Function Name  : Dc_Motor
* Description    : 
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/
uint8_t fs_flag = 0;
void DC_Task(uint8_t iKey)
{
    
    uint8_t Buffer_DC[1]={0Xff};
    uint8_t Buffer_DC_Zero[1]={0x00};
    
    switch(iKey)
    {
        case 0x1B:
            DC_Motor_Pin_Low();
            fs_flag = 1;
            I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x03,Buffer_DC,1);
            break;
        case 0x13:
            DC_MOtor_Pin_High();
            fs_flag = 0;
            I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x05,Buffer_DC,1);
            break;
        default:
            DC_Motor_Pin_Low();
            fs_flag = 2;
            //I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x00,Buffer_DC_Zero,1);
            break;
    }
}




