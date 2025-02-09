#include "main.h"
#include "i2c_slave_u.h"

extern I2C_HandleTypeDef hi2c1;
#define RxSIZE 4
uint8_t RxData[RxSIZE];
uint8_t count;
uint16_t last;



void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{

    uint16_t res = 0 ;
    res |= (uint16_t)RxData[0] << 8;
    res |= (uint16_t)RxData[1];
    
    if (res != last){
        
        last = res;
        printf("val: %d\n\t",  res);
    }
    




        HAL_I2C_EnableListen_IT(hi2c);



}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData, RxSIZE, I2C_FIRST_AND_LAST_FRAME);
    }
    else {
        Error_Handler();
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    count ++;

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave doesn't acknowledge its address, Master restarts communication.
    * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
    */

      HAL_I2C_EnableListen_IT(hi2c);  

}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}
