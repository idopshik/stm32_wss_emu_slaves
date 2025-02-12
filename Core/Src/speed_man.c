
#include "main.h"
#include "speed_man.h"
#include <stdio.h>
#include <string.h>



extern whl_chnl *whl_arr[2];

uint32_t calculete_period_only(int val)
{
    // For 32bit timer

    float factor = 0.02;
    return (APB1_CLK / (val * factor * MinutTeethFactor * (12 + 1))) - 1;  // значение регистра ARR
}

void set_new_speeds(int vXLrpm, int vXRrpm)
{
    //__HAL_TIM_SET_PRESCALER(&htim3, val );
    //__HAL_TIM_SET_AUTORELOAD(&htim3, val );
    /* permutations! */
    /* 1) changing prescaler up or down  */
    /* 2) the same prescaler */
    /* ----- */
    /* 1) going up (speed up) */
    /* 2) going down. */

    // 32-bit timers first

    ////////////    TIMER2 -  XL  ///////////////
    if (vXLrpm == 0) {
        TIM2->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM2->CR1 |= TIM_CR1_CEN;  // enable
                                   //
        uint32_t current_32bit_period = calculete_period_only(vXLrpm);

        if (vXLrpm > whl_arr[numXL]->prev_speed) {
            TIM2->CR1 |= TIM_CR1_ARPE;
            TIM2->ARR = current_32bit_period;

            if (TIM2->CNT > current_32bit_period) {
                TIM2->EGR |= TIM_EGR_UG;
            }

            /* HAL_GPIO_TogglePin(LED_RX2_GPIO_Port, LED_RX2_Pin); */
        }
        else {
            /* my_printf(" ARR: %d\n\r", arr_with_calculations[0]); */

            /* HAL_GPIO_TogglePin(LED_TX2_GPIO_Port, LED_TX2_Pin); */

            if (TIM2->CNT < current_32bit_period) {
                TIM2->CR1 |= TIM_CR1_ARPE;
                TIM2->ARR = current_32bit_period;
            }
            else {
                // Не должна никогда выполняться.
                TIM2->CR1 &= ~TIM_CR1_ARPE;
                TIM2->ARR = current_32bit_period;
            }
        }

        //Костыльный предохранитель от убегания.
        if (TIM2->CNT > current_32bit_period) {
            TIM2->EGR |= TIM_EGR_UG;
        }
    }
    whl_arr[numXL]->prev_speed = vXLrpm;







    ////////////    TIMER5 -  XR  ///////////////
    if (vXRrpm == 0) {
        TIM5->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM5->CR1 |= TIM_CR1_CEN;  // enable
                                   //
        uint32_t current_32bit_period = calculete_period_only(vXRrpm);

        if (vXRrpm > whl_arr[numXR]->prev_speed) {
            TIM5->CR1 |= TIM_CR1_ARPE;
            TIM5->ARR = current_32bit_period;

            if (TIM5->CNT > current_32bit_period) {
                TIM5->EGR |= TIM_EGR_UG;
            }

            /* HAL_GPIO_TogglePin(LED_RX2_GPIO_Port, LED_RX2_Pin); */
        }
        else {
            /* my_printf(" ARR: %d\n\r", arr_with_calculations[0]); */

            /* HAL_GPIO_TogglePin(LED_TX2_GPIO_Port, LED_TX2_Pin); */

            if (TIM5->CNT < current_32bit_period) {
                TIM5->CR1 |= TIM_CR1_ARPE;
                TIM5->ARR = current_32bit_period;
            }
            else {
                TIM5->CR1 &= ~TIM_CR1_ARPE;
                TIM5->ARR = current_32bit_period;
            }
        }

        //Костыльный предохранитель от убегания.
        if (TIM5->CNT > current_32bit_period) {
            TIM5->EGR |= TIM_EGR_UG;
            /* HAL_GPIO_TogglePin(LED_RX3_GPIO_Port, LED_RX3_Pin); */
        }
    }
    whl_arr[numXR]->prev_speed = vXRrpm;
}


uint8_t TWI_deal(uint8_t *receive_arr )
{

    uint16_t newval_0 = 0;
    uint16_t newval_1 = 0;

    newval_0 |= ((int)receive_arr[0]) << 8;
    newval_0 |= (int)receive_arr[1];

    newval_1 |= ((int)receive_arr[2]) << 8;
    newval_1 |= (int)receive_arr[3];

    set_new_speeds(newval_0, newval_1);

    return 0;
}

