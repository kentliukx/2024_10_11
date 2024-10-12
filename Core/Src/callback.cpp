//
// Created by kentl on 24-10-12.
//
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"

extern uint8_t rx_message[8];
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_message[8];
uint8_t send[4];
uint32_t buffer;

class M3508_Motor {
    private:
        float ratio_;// 电机减速比
        float angle_;//存在误差的储存方式，输出端累计转动角度
        struct {
            int r;
            float angle;
        } angle_new;//用结构将输出端角度分开储存减少误差
        float delta_angle_;// deg 输出端新转动的角度
        float ecd_angle_;// deg 当前电机编码器角度
        float last_ecd_angle_;  // deg 上次电机编码器角度
        float delta_ecd_angle_; // deg 编码器端新转动的角度
        float rotate_speed_;
        // dps 反馈转子转速
        float current_;
        float temp;
    public:
        M3508_Motor();
        void canRxMsgCallback_v4(uint8_t rx_data[8]);//task6
        void store_angle_new(float delta_angle_);//储存输出端累计角度
        int show_angle_new();//返回输出端累计角度,都几百万度了就别在意那零点几度了
        void show(uint8_t send[4]);

};

M3508_Motor::M3508_Motor(){
    ratio_=3591/187;
    angle_=0;
    angle_new.r=0;
    angle_new.angle=0;
    last_ecd_angle_=0;
    ecd_angle_=0;
}

float linearMapping(int in, int in_min, int in_max, float out_min, float out_max){
    return ((out_max-out_min)/(in_max-in_min)*in+(in_max*out_min-in_min*out_max)/(in_max-in_min));
}

void M3508_Motor::store_angle_new(float delta_angle_){
    angle_new.angle+=delta_angle_;
    if(angle_new.angle>=360){
        angle_new.angle-=360;
        angle_new.r+=1;
    }
    if(angle_new.angle<0){
        angle_new.angle+=360;
        angle_new.r-=1;
    }
}
int M3508_Motor::show_angle_new(){
    return angle_new.angle+360*angle_new.r;
}

void M3508_Motor::canRxMsgCallback_v4(uint8_t rx_data[8]){
    last_ecd_angle_=ecd_angle_;//更新上次编码器角度

    ecd_angle_=((float)rx_data[0]*256.0+(float)rx_data[1])*(360.0/8191.0);
    rotate_speed_=rx_data[2]*256+rx_data[3];
    current_=rx_data[4]*256+rx_data[5];
    temp=rx_data[6];//get数据

    if(ecd_angle_>last_ecd_angle_-180) delta_ecd_angle_=ecd_angle_-last_ecd_angle_;
    else delta_ecd_angle_=ecd_angle_-last_ecd_angle_+360;//获取角度增量or减量,此时已经转过一整圈
    delta_angle_=delta_ecd_angle_*ratio_;
    store_angle_new(delta_angle_);//保存电机累计转过角度
}

void M3508_Motor::show(uint8_t send[4])
{
    send[0]=show_angle_new();
    send[1]=rotate_speed_;
    send[2]=current_;
    send[3]=temp;
}

M3508_Motor motor1;



void setmotor1(int current)
{
    tx_header.StdId=0x200;
    tx_header.IDE=CAN_ID_STD;
    tx_header.RTR=CAN_RTR_DATA;
    tx_header.DLC=8;
    tx_message[2]=0;
    tx_message[3]=current;

}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rx_header,rx_message);
    if(rx_header.StdId==0x202){
        motor1.canRxMsgCallback_v4(rx_message);
        motor1.show(send);
    }
    if(rx_header.StdId==0x100)
    {
        setmotor1(rx_message[3]);
    }

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6) {
        HAL_UART_Transmit_IT(&huart6,send, 4);
        HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_message, &buffer);

    }
}



