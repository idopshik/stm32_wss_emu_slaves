
#ifndef INC_SPEED_MAN_H
#define  INC_SPEED_MAN_H


#define MinutTeethFactor 1.6
#define APB1_CLK 84000000
#define numXL 0
#define numXR 1

typedef struct {
    uint8_t wheel_num;
    TIM_HandleTypeDef *htim;
    int prev_speed;
    int prev_psc;
    int cur_psc;
    int initial_tmp_flag;
    uint8_t psc_change_flag;
} whl_chnl;


uint8_t TWI_deal(uint8_t *receive_arr );

#endif /* INC_SPEED_MAN_H */
