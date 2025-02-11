#include "main.h"
#include "i2c_slave_u.h"
#include <stdio.h>
#include <string.h>

#define BufferSIZE 5

extern uint8_t i2c_received;

extern uint8_t not_i2c_buffer[BufferSIZE];

uint8_t RxData[BufferSIZE];
uint8_t TxData[BufferSIZE];

extern I2C_HandleTypeDef hi2c1;
uint8_t firstData = 1;
uint8_t count;
uint16_t last;

/* void process_data(void){ */
    /* {} */

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{

    /* uint16_t res = 0 ; */
    /* res |= (uint16_t)RxData[0] << 8; */
    /* res |= (uint16_t)RxData[1]; */
    
    /* if (res != last){ */
        
        /* last = res; */
        /* printf("val: %d\n\t",  res); */
    /* } */
    




    // это необходимо. Почему работало у индуса без этого - не знаю. 
        HAL_I2C_EnableListen_IT(hi2c);



}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // Master wants to write data

        


        if (firstData == 1){
            count = 0;
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+count, 1,  I2C_FIRST_FRAME);
        }
    }
    else {
        //Master wants to read data
        Error_Handler();
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (firstData == 1){
        firstData = 0;
        count ++;

        HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+count, 4,  I2C_LAST_FRAME);


    }
    else{
        HAL_GPIO_TogglePin(BlueLed_GPIO_Port, BlueLed_Pin);
        memcpy(&not_i2c_buffer,RxData, BufferSIZE );

        i2c_received = 1;
        firstData = 1;
        /* process_data(); */

    }


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
