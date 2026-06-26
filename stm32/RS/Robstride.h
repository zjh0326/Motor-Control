#ifndef __RobStride_H__
#define __RobStride_H__

#include "main.h"
#include "fdcan.h"      // ????? can.h

#define Set_mode            'j'         //??????
#define Set_parameter       'p'         //????
//??????
#define move_control_mode   0
#define Pos_control_mode    1
#define Speed_control_mode  2
#define Elect_control_mode  3
#define Set_Zero_mode       4
#define CSP_control_mode    5
//????
#define Communication_Type_Get_ID              0x00
#define Communication_Type_MotionControl       0x01
#define Communication_Type_MotorRequest        0x02
#define Communication_Type_MotorEnable         0x03
#define Communication_Type_MotorStop           0x04
#define Communication_Type_SetPosZero          0x06
#define Communication_Type_Can_ID              0x07
// Control_Mode is set via SetSingleParameter(0x12) with index 0x7005
#define Communication_Type_GetSingleParameter  0x11
#define Communication_Type_SetSingleParameter  0x12
#define Communication_Type_ErrorFeedback       0x15
#define Communication_Type_MotorDataSave       0x16
#define Communication_Type_BaudRateChange      0x17
#define Communication_Type_ProactiveEscalationSet 0x18
#define Communication_Type_MotorModeSet        0x19

class data_read_write_one
{
public:
    uint16_t index;
    float data;
};

static const uint16_t Index_List[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014,
    0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D
};

class data_read_write
{
public:
    data_read_write_one run_mode;
    data_read_write_one iq_ref;
    data_read_write_one spd_ref;
    data_read_write_one imit_torque;
    data_read_write_one cur_kp;
    data_read_write_one cur_ki;
    data_read_write_one cur_filt_gain;
    data_read_write_one loc_ref;
    data_read_write_one limit_spd;
    data_read_write_one limit_cur;
    data_read_write_one mechPos;
    data_read_write_one iqf;
    data_read_write_one mechVel;
    data_read_write_one VBUS;
    data_read_write_one rotation;
    data_read_write(const uint16_t *index_list = Index_List);
};

typedef struct
{
    float Angle;
    float Speed;
    float Torque;
    float Temp;
    int pattern;
} Motor_Pos_RobStride_Info;

typedef struct
{
    int set_motor_mode;
    float set_current;
    float set_speed;
    float set_acceleration;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_limit_speed;
    float set_Kp;
    float set_Ki;
    float set_Kd;
} Motor_Set;

enum MIT_TYPE
{
    operationControl = 0,
    positionControl = 1,
    speedControl = 2
};

class RobStride_Motor
{
private:
    uint8_t CAN_ID;
    uint64_t Unique_ID;
    uint16_t Master_CAN_ID;
    float (*Motor_Offset_MotoFunc)(float Motor_Tar);
    Motor_Set Motor_Set_All;
    uint8_t error_code;
    bool MIT_Mode;
    MIT_TYPE MIT_Type;
    void Set_MIT_Mode(bool MIT_Mode);
    void Set_MIT_Type(MIT_TYPE MIT_Type);

public:
    float output;
    int Can_Motor;
    Motor_Pos_RobStride_Info Pos_Info;
    data_read_write drw;

    RobStride_Motor(uint8_t CAN_Id, bool MIT_Mode);
    RobStride_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id, bool MIT_mode);
    void RobStride_Get_CAN_ID();
    void Set_RobStride_Motor_parameter(uint16_t Index, float Value, char Value_mode);
    void Get_RobStride_Motor_parameter(uint16_t Index);
    void RobStride_Motor_Analysis(uint8_t *DataFrame, uint32_t ID_ExtId);
    void RobStride_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd);
    void RobStride_Motor_Pos_control(float Speed, float Angle);
    void RobStride_Motor_CSP_control(float Angle, float limit_spd);
    void RobStride_Motor_Speed_control(float Speed, float limit_cur);
    void RobStride_Motor_current_control(float current);
    void RobStride_Motor_Set_Zero_control();
    void RobStride_Motor_MotorModeSet(uint8_t F_CMD);
    void Enable_Motor();
    void Disenable_Motor(uint8_t clear_error);
    void Set_CAN_ID(uint8_t Set_CAN_ID);
    void Set_ZeroPos();

    bool Get_MIT_Mode();
    MIT_TYPE get_MIT_Type();
    void RobStride_Motor_MIT_Control(float Angle, float Speed, float Kp, float Kd, float Torque);
    void RobStride_Motor_MIT_PositionControl(float position_rad, float speed_rad_per_s);
    void RobStride_Motor_MIT_SpeedControl(float speed_rad_per_s, float current_limit);
    void RobStride_Motor_MIT_Enable();
    void RobStride_Motor_MIT_Disable();
    void RobStride_Motor_MIT_SetZeroPos();
    void RobStride_Motor_MIT_ClearOrCheckError(uint8_t F_CMD);
    void RobStride_Motor_MIT_SetMotorType(uint8_t F_CMD);
    void RobStride_Motor_MIT_SetMotorId(uint8_t F_CMD);
    void RobStride_Motor_MotorDataSave();
    void RobStride_Motor_BaudRateChange(uint8_t F_CMD);
    void RobStride_Motor_ProactiveEscalationSet(uint8_t F_CMD);
    void RobStride_Motor_MIT_MotorModeSet(uint8_t F_CMD);
};

#endif