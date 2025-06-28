#ifndef _NV_Params_c
#define NV_Params_C

#include <stdint.h>

typedef struct
{
    //PID controller gains
    float I_Control_Kp; //0,1
    float I_Control_Ki; //2,3
    float I_Filter_Fc; //4,5
    float Controller_Sampling_Time; //6,7

    float Control_Max_Out;//8,9


    //Current sensor filter constant

    //current sensor params for phase A and B
    float I_Sense_Gain; //10,11
    float I_Sense_Offset; //12,13
    float Vmotor_Sense_Gain; //14,15
    float Vmotor_Sense_Offset; //16,17

    //differential angle torque sensor
    float Angle_Sensor_1_Gain; //18,19
    float Angle_Sensor_1_Offset; //20,21
    float Angle_Sensor_2_Gain; //22,23
    float Angle_Sensor_2_Offset; //24,25
    float Torque_Signal_Gain; //26,27
    float Torque_Signal_offset; //28,29
    float Angle_Sensor_Fc; //LPF cutoff frequency 30,31
    float Amplifier_offset;//amplifier offset 32,33
    float Linear_Disp_Gain; //linear displacment sensor gain 34,35
    float I_Control_Kc;

    uint16_t Checksum; //38,39

}SystemParams_t;

#define SystemParams_Size_Words (sizeof(SystemParams_t)/2)





#endif
