#include "RobStride.h"
#include "string.h"
#include "fdcan.h"
 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -44.0f 
 #define V_MAX 44.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -17.0f
 #define T_MAX 17.0f

extern FDCAN_HandleTypeDef hfdcan1;


/*******************************************************************************
* @����     		: RobStride���ʵ�����Ĺ��캯��
* @����         : CAN ID
* @����ֵ 			: void
* @����  				: ��ʼ�����ID��
*******************************************************************************/
RobStride_Motor::RobStride_Motor(uint8_t CAN_Id, bool MIT_mode)
{
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0xFD;	
	Motor_Set_All.set_motor_mode = move_control_mode;
	MIT_Mode = MIT_mode;
	MIT_Type = operationControl;
}
RobStride_Motor::RobStride_Motor(float (*Offset_MotoFunc)(float Motor_Tar) , uint8_t CAN_Id, bool MIT_mode)
{
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0xFD;	
	Motor_Set_All.set_motor_mode = move_control_mode;
	Motor_Offset_MotoFunc = Offset_MotoFunc;
	MIT_Mode = MIT_mode;
	MIT_Type = operationControl;
}
/*******************************************************************************
* @����     		: uint16_t��תfloat�͸�����
* @����1        : ��Ҫת����ֵ
* @����2        : x����Сֵ
* @����3        : x�����ֵ
* @����4        : ��Ҫת���Ľ�����
* @����ֵ 			: ʮ���Ƶ�float�͸�����
* @����  				: None
*******************************************************************************/
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
		x &= span; 
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}
/*******************************************************************************
* @����     		: float������תint��
* @����1        : ��Ҫת����ֵ
* @����2        : x����Сֵ
* @����3        : x�����ֵ
* @����4        : ��Ҫת���Ľ�����
* @����ֵ 			: ʮ���Ƶ�int������
* @����  				: None
*******************************************************************************/
int float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x = x_max;
	else if(x < x_min) x = x_min;
	return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}
/*******************************************************************************
* @����     		: uint8_t����תfloat������
* @����        	: ��Ҫת��������
* @����ֵ 			: ʮ���Ƶ�float�͸�����
* @����  				: None
*******************************************************************************/
float Byte_to_float(uint8_t* bytedata)  
{  
	uint32_t data = bytedata[7]<<24|bytedata[6]<<16|bytedata[5]<<8|bytedata[4];
	float data_float = *(float*)(&data);
  return data_float;  
}  
/*******************************************************************************
* @����     	: MIT��λ�� ת���� ˽��ģʽ��λ��
* @����        	: MIT��λ��
* @����ֵ 		: uint8_t
* @����  		: None
*******************************************************************************/
uint8_t mapFaults(uint16_t fault16) {
    uint8_t fault8 = 0;

    if (fault16 & (1 << 14)) fault8 |= (1 << 4); // ���ع���
    if (fault16 & (1 << 7))  fault8 |= (1 << 5); // δ�궨
    if (fault16 & (1 << 3))  fault8 |= (1 << 3); // �ű������
    if (fault16 & (1 << 2))  fault8 |= (1 << 0); // Ƿѹ����
    if (fault16 & (1 << 1))  fault8 |= (1 << 1); // ��������
    if (fault16 & (1 << 0))  fault8 |= (1 << 2); // ����

    return fault8;
}
//int count_num = 0 ;
/*******************************************************************************
* @����     	: ���մ�������		��ͨ������2 17Ӧ��֡ 0Ӧ��֡��
* @����1        : ���յ�������
* @����2        : ���յ���CANID
* @����ֵ 		: None
* @����  		: drwֻ��ͨ��ͨ��17�����Ժ����ֵ
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId)
{
	if(MIT_Mode)
	{
		if((ID_ExtId & 0xFF) == 0XFD)
		{
			if(DataFrame[3] == 0x00 && DataFrame[4] == 0x00 && DataFrame[5] == 0x00 && DataFrame[6] == 0x00 && DataFrame[7] == 0x00)
			{
				uint16_t fault16 = 0;
				memcpy(&fault16, &DataFrame[1], 2);
				error_code = mapFaults(fault16);
			}
			else
			{
				Pos_Info.Angle =  uint16_to_float((DataFrame[1]<<8) | (DataFrame[2]),P_MIN,P_MAX,16);
				Pos_Info.Speed =  uint16_to_float((DataFrame[3]<<4) | (DataFrame[4]>>4),V_MIN,V_MAX,12);
				Pos_Info.Torque = uint16_to_float((DataFrame[4]<<8) | (DataFrame[5]),T_MIN,T_MAX,12);
				Pos_Info.Temp = ((DataFrame[6]<<8) | DataFrame[7])*0.1;
			}
		}
		else
		{
		    memcpy(&Unique_ID, DataFrame, 8);
		}
	}
	else 
	{
		if (uint8_t((ID_ExtId&0xFF00)>>8) == CAN_ID)
		{		
			if (int((ID_ExtId&0x3F000000)>>24) == 2)
			{
				Pos_Info.Angle =  uint16_to_float(DataFrame[0]<<8|DataFrame[1],P_MIN,P_MAX,16);
				Pos_Info.Speed =  uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);			
				Pos_Info.Torque = uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);				
				Pos_Info.Temp = (DataFrame[6]<<8|DataFrame[7])*0.1;
				error_code = uint8_t((ID_ExtId&0x3F0000)>>16);
				Pos_Info.pattern = uint8_t((ID_ExtId&0xC00000)>>22);
			}
			else if (int((ID_ExtId&0x3F000000)>>24) == 17)
			{
				for (int index_num = 0; index_num <= 13; index_num++)
				{
					if ((DataFrame[1]<<8|DataFrame[0]) == Index_List[index_num])
						switch(index_num)
						{
							case 0:
								drw.run_mode.data = uint8_t(DataFrame[4]);
								break;
							case 1:
								drw.iq_ref.data = Byte_to_float(DataFrame);
								break;
							case 2:
								drw.spd_ref.data = Byte_to_float(DataFrame);
								break;
							case 3:
								drw.imit_torque.data = Byte_to_float(DataFrame);
								break;
							case 4:
								drw.cur_kp.data = Byte_to_float(DataFrame);
								break;
							case 5:
								drw.cur_ki.data = Byte_to_float(DataFrame);
								break;
							case 6:
								drw.cur_filt_gain.data = Byte_to_float(DataFrame);
								break;
							case 7:
								drw.loc_ref.data = Byte_to_float(DataFrame);
								break;
							case 8:
								drw.limit_spd.data = Byte_to_float(DataFrame);
								break;
							case 9:
								drw.limit_cur.data = Byte_to_float(DataFrame);
								break;	
							case 10:
								drw.mechPos.data = Byte_to_float(DataFrame);
								break;	
							case 11:
								drw.iqf.data = Byte_to_float(DataFrame);
								break;	
							case 12:
								drw.mechVel.data =Byte_to_float(DataFrame);
								break;	
							case 13:
								drw.VBUS.data = Byte_to_float(DataFrame);
								break;	
						}
				}
			}
			else if ((uint8_t)((ID_ExtId & 0xFF)) == 0xFE)
			{
				CAN_ID = uint8_t((ID_ExtId & 0xFF00)>>8);	
				memcpy(&Unique_ID, DataFrame, 8);
			}
		}
	}
}
/*******************************************************************************
* @����     		: RobStride�����ȡ�豸ID��MCU��ͨ������0��
* @����         : None
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Get_CAN_ID()
{
		uint8_t txdata[8] = {0};						   	//��������
		FDCAN_TxHeaderTypeDef TxMessage={0}; 	//��������

TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_Get_ID<<24|Master_CAN_ID <<8|CAN_ID; 
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
}
/*******************************************************************************
* @����     		: RobStride����˿�ģʽ  ��ͨ������1��
* @����1        : ���أ�-4Nm~4Nm��
* @����2        : Ŀ��Ƕ�(-4��~4��)
* @����3        : Ŀ����ٶ�(-30rad/s~30rad/s)
* @����4        : Kp(0.0~500.0)
* @����5        : Kp(0.0~5.0)
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd)
{
	uint8_t txdata[8] = {0};						   	//��������
	FDCAN_TxHeaderTypeDef TxMessage = {0}; 					//��������
	Motor_Set_All.set_Torque = Torque;
	Motor_Set_All.set_angle = Angle;	
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_Kp = Kp;
	Motor_Set_All.set_Kd = Kd;
	if (drw.run_mode.data != 0)
	{
		Set_RobStride_Motor_parameter(0X7005, move_control_mode, Set_mode);		//���õ��ģʽ
		Get_RobStride_Motor_parameter(0x7005);
		Enable_Motor();
		Motor_Set_All.set_motor_mode = move_control_mode;
	}
	if(Pos_Info.pattern != 2)
	{
		Enable_Motor();
	}
	TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_MotionControl<<24|float_to_uint(Motor_Set_All.set_Torque,T_MIN,T_MAX,16)<<8|CAN_ID; 
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;
	txdata[0] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16)>>8; 
	txdata[1] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16); 
	txdata[2] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16)>>8; 
	txdata[3] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16); 
	txdata[4] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16)>>8; 
	txdata[5] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16); 
	txdata[6] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16)>>8; 
	txdata[7] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16); 
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
}
//MITģʽʹ��
void RobStride_Motor::RobStride_Motor_MIT_Enable()
{
	uint8_t txdata[8] = {0}; 	//���巢����������
FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier = CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;

	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFC;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata);
}

//MITģʽʧ��
void RobStride_Motor::RobStride_Motor_MIT_Disable()
{
	uint8_t txdata[8] = {0}; 	//���巢����������
FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier = CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;

	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFD;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata);
}

//MITģʽ����������
void RobStride_Motor::RobStride_Motor_MIT_ClearOrCheckError(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0}; 	//���巢����������
FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier = CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;

	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFB;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata); 	//����CAN��Ϣ
}

//MIT���õ������ģʽ
void RobStride_Motor::RobStride_Motor_MIT_SetMotorType(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0}; 	//���巢����������
FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier = CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFC;
HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata); 	//����CAN��Ϣ
}

//MIT���õ��ID
void RobStride_Motor::RobStride_Motor_MIT_SetMotorId(uint8_t F_CMD)
{
    uint8_t txdata[8] = {0}; 	//���巢����������
FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier = CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;

	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0x01;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata); 	//����CAN��Ϣ
}



//MIT����ģʽ
void RobStride_Motor::RobStride_Motor_MIT_Control(float Angle, float Speed, float Kp, float Kd, float Torque)
{
	uint8_t txdata[8] = {0}; 	//���巢����������
FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier = CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;
	txdata[0] = float_to_uint(Angle, P_MIN,P_MAX, 16)>>8;
	txdata[1] = float_to_uint(Angle, P_MIN,P_MAX, 16);
	txdata[2] = float_to_uint(Speed, V_MIN,V_MAX, 12)>>4;
	txdata[3] = float_to_uint(Speed, V_MIN,V_MAX, 12)<<4 | float_to_uint(Kp, KP_MIN, KP_MAX, 12)>>8;
	txdata[4] = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
	txdata[5] = float_to_uint(Kd, KD_MIN, KD_MAX, 12)>>4;
	txdata[6] = float_to_uint(Kd, KD_MIN, KD_MAX, 12)<<4 | float_to_uint(Torque, T_MIN, T_MAX, 12)>>8;
	txdata[7] = float_to_uint(Torque, T_MIN, T_MAX, 12);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata); 	//����CAN��Ϣ
}

//MITλ��ģʽ
void RobStride_Motor::RobStride_Motor_MIT_PositionControl(float position_rad, float speed_rad_per_s)
{
	uint8_t txdata[8] = {0}; 	//���巢����������
	
	FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier =  (1 << 8) | CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;
	memcpy(&txdata[0], &position_rad, 4); 	//��λ�����ݸ��Ƶ���������������
	memcpy(&txdata[4], &speed_rad_per_s, 4); 	//���ٶ����ݸ��Ƶ���������������
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata); 	//����CAN��Ϣ
}
// MITģʽ�ٶȿ���ʵ��
void RobStride_Motor::RobStride_Motor_MIT_SpeedControl(float speed_rad_per_s, float current_limit)
{
	uint8_t txdata[8] = {0}; 	//���巢����������
	
	FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier =  (2 << 8) | CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;

	memcpy(&txdata[0], &speed_rad_per_s, 4);
	memcpy(&txdata[4], &current_limit, 4);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata);
}

//MIT�������ģʽ
void RobStride_Motor::RobStride_Motor_MIT_SetZeroPos()
{
	uint8_t txdata[8] = {0}; 	//���巢����������
	
	
	FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier = CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;
	
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFE;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata); 	//����CAN��Ϣ
}

/*******************************************************************************
* @����     		: RobStride���λ��ģʽ(PP�岹λ��ģʽ����)
* @����1        : Ŀ����ٶ�(-30rad/s~30rad/s)
* @����2        : Ŀ��Ƕ�(-4��~4��)
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_Pos_control(float Speed, float Angle)
{
		Motor_Set_All.set_speed = Speed;
		Motor_Set_All.set_angle = Angle;
//		Motor_Set_All.set_limit_speed = vel_max;
//		Motor_Set_All.set_acceleration = acc_set;
//		if (drw.run_mode.data != 1 && Pos_Info.pattern == 2)
		if (drw.run_mode.data != 1)
		{
			Set_RobStride_Motor_parameter(0X7005, Pos_control_mode, Set_mode);		//���õ��ģʽ
			Get_RobStride_Motor_parameter(0x7005);
			Motor_Set_All.set_motor_mode = Pos_control_mode;
			Enable_Motor();
			Set_RobStride_Motor_parameter(0X7024, Motor_Set_All.set_limit_speed, Set_parameter);
			Set_RobStride_Motor_parameter(0X7025, Motor_Set_All.set_acceleration, Set_parameter);
		}	
		HAL_Delay(1);
		Set_RobStride_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
}
/*******************************************************************************
* @����     		: RobStride���λ��ģʽ(CSPλ��ģʽ����)
* @����1        : Ŀ��Ƕ�(-4��~4��)
* @����2        : Ŀ����ٶ�(0rad/s~44rad/s)
* @����ֵ 				: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_CSP_control(float Angle, float limit_spd)
{
	if(MIT_Mode){
		RobStride_Motor_MIT_PositionControl(Angle, limit_spd);
	}
	else{
		Motor_Set_All.set_angle = Angle;
		Motor_Set_All.set_limit_speed = limit_spd;
		if (drw.run_mode.data != 1)
		{
			Set_RobStride_Motor_parameter(0X7005, CSP_control_mode, Set_mode);
			Get_RobStride_Motor_parameter(0x7005);
			Enable_Motor();
			Set_RobStride_Motor_parameter(0X7017, Motor_Set_All.set_limit_speed, Set_parameter);
		}
		HAL_Delay(1);
		Set_RobStride_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
	}
}

/*******************************************************************************
* @����     		: RobStride����ٶ�ģʽ 
* @����1        : Ŀ����ٶ�(-30rad/s~30rad/s)
* @����2        : Ŀ���������(0~23A)
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
uint8_t count_set_motor_mode_Speed = 0;
void RobStride_Motor::RobStride_Motor_Speed_control(float Speed, float limit_cur)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_limit_cur = limit_cur;
	if (drw.run_mode.data != 2)
	{
		Set_RobStride_Motor_parameter(0X7005, Speed_control_mode, Set_mode);		//���õ��ģʽ
		Get_RobStride_Motor_parameter(0x7005);
		Enable_Motor();
		Motor_Set_All.set_motor_mode = Speed_control_mode;
		Set_RobStride_Motor_parameter(0X7018, Motor_Set_All.set_limit_cur, Set_parameter);
		Set_RobStride_Motor_parameter(0X7022, 10, Set_parameter);	
//		Set_RobStride_Motor_parameter(0X7022, Motor_Set_All.set_acceleration, Set_parameter);	
	}
	Set_RobStride_Motor_parameter(0X700A, Motor_Set_All.set_speed, Set_parameter);
}
/*******************************************************************************
* @����     		: RobStride�������ģʽ
* @����         : Ŀ�����(-23~23A)
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
uint8_t count_set_motor_mode = 0;
void RobStride_Motor::RobStride_Motor_current_control(float current)
{
	Motor_Set_All.set_current = current;
	output = Motor_Set_All.set_current;
	if (Motor_Set_All.set_motor_mode != 3)
	{
		Set_RobStride_Motor_parameter(0X7005, Elect_control_mode, Set_mode);		//���õ��ģʽ
		Get_RobStride_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = Elect_control_mode;
		Enable_Motor();
	}
	Set_RobStride_Motor_parameter(0X7006, Motor_Set_All.set_current, Set_parameter);
}
/*******************************************************************************
* @����     	: RobStride������ģʽ(����ص���е���)
* @����         : None
* @����ֵ 		: void
* @����  		: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_Set_Zero_control()
{
	Set_RobStride_Motor_parameter(0X7005, Set_Zero_mode, Set_mode);					//���õ��ģʽ
}
/*******************************************************************************
* @����     		: RobStride���ʹ�� ��ͨ������3��
* @����         : None
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::Enable_Motor()
{
	if (MIT_Mode)
	{
		RobStride_Motor_MIT_Enable();
	}
	else
	{
		uint8_t txdata[8] = {0};				//��������

		FDCAN_TxHeaderTypeDef TxMessage = {0};
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
	}
}
/*******************************************************************************
* @����     		: RobStride���ʧ�� ��ͨ������4��
* @����         : �Ƿ��������λ��0����� 1�����
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::Disenable_Motor(uint8_t clear_error)
{
	if (MIT_Mode)
	{
		RobStride_Motor_MIT_Disable();
	}
	else
	{
		uint8_t txdata[8] = {0};					   	//��������
			FDCAN_TxHeaderTypeDef TxMessage = {0};; 	//��������

		txdata[0] = clear_error;
		
	
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
		Set_RobStride_Motor_parameter(0X7005, move_control_mode, Set_mode);
	}

}
/*******************************************************************************
* @����     		: RobStride���д����� ��ͨ������18��
* @����1        : ������ַ
* @����2        : ������ֵ
* @����3        : ѡ���Ǵ������ģʽ ������������ ��Set_mode���ÿ���ģʽ Set_parameter���ò�����
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::Set_RobStride_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{
	uint8_t txdata[8] = {0};						   	//��������
	//��������
	
	FDCAN_TxHeaderTypeDef TxMessage = {0};
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier =Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;

	
	txdata[0] = Index;
	txdata[1] = Index>>8;
	txdata[2] = 0x00;
	txdata[3] = 0x00;	
	if (Value_mode == 'p')
	{
		memcpy(&txdata[4],&Value,4);
	}
	else if (Value_mode == 'j')
	{
		Motor_Set_All.set_motor_mode = int(Value);
		txdata[4] = (uint8_t)Value;
		txdata[5] = 0x00;	
		txdata[6] = 0x00;	
		txdata[7] = 0x00;	
	}
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
}
/*******************************************************************************
* @����     		: RobStride�������������ȡ ��ͨ������17��
* @����         : ������ַ
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::Get_RobStride_Motor_parameter(uint16_t Index)
{
	uint8_t txdata[8] = {0};						   	//��������
  FDCAN_TxHeaderTypeDef TxMessage = {0};	//��������
	txdata[0] = Index;
	txdata[1] = Index>>8;

TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_GetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;
	
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
}
/*******************************************************************************
* @����     		: RobStride�������CAN_ID ��ͨ������7��
* @����         : �޸ĺ�Ԥ�裩CANID
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::Set_CAN_ID(uint8_t Set_CAN_ID)
{
	Disenable_Motor(0);
	uint8_t txdata[8] = {0};						   	//��������
	//��������

	FDCAN_TxHeaderTypeDef TxMessage = {0};
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_Can_ID<<24|Set_CAN_ID<<16|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
}
/*******************************************************************************
* @����     		: RobStride������û�е��� ��ͨ������6��
* @����         : None
* @����ֵ 			: void
* @����  				: ��ѵ�ǰ���λ����Ϊ��е��λ�� ����ʧ�ܵ��, ��ʹ�ܵ��
*******************************************************************************/
void RobStride_Motor::Set_ZeroPos()
{
	Disenable_Motor(0);							//ʧ�ܵ��
	uint8_t txdata[8] = {0};						   	//��������
	//��������
	FDCAN_TxHeaderTypeDef TxMessage = {0};
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;

	txdata[0] = 1;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
	Enable_Motor();
}

/*******************************************************************************
* @����     		: RobStride������ݱ��� ��ͨ������22��
* @����      		: None
* @����ֵ 				: void
* @����  				: ��ѵ�ǰ������д����е�����дΪĬ��ֵ�������ϵ���������Ϊ��ָ������ʱ�Ĳ���
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_MotorDataSave()
{
	uint8_t txdata[8] = {0};
	FDCAN_TxHeaderTypeDef TxMessage = {0};
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_MotorDataSave<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;

	txdata[0] = 0x01;	// 保存当前参数到Flash
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata);
}

/*******************************************************************************
* @����     		: RobStride����������޸� ��ͨ������23��
* @����      		: ������ģʽ:	 01��1M��
									02��500K��
									03��250K��
									04��125K��
* @����ֵ 				: void
* @����  				: ������������޸�Ϊ��Ӧ��ֵ���������Ϊ01���������޸�Ϊ1M
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_BaudRateChange(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};
	FDCAN_TxHeaderTypeDef TxMessage = {0};
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_BaudRateChange<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;
	
	txdata[0] = F_CMD;	// 1=1Mbps, 2=500kbps, 3=250kbps, 4=125kbps
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata);
}

/*******************************************************************************
* @����     		: RobStride��������ϱ����� ��ͨ������24��
* @����      		: �ϱ�ģʽ��	00���رգ�
														01��������
* @����ֵ 				: void
* @����  				: ����/�ر� ��������ϱ���Ĭ���ϱ�����Ϊ10ms
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_ProactiveEscalationSet(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};
	FDCAN_TxHeaderTypeDef TxMessage = {0};
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_ProactiveEscalationSet<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;

	txdata[0] = F_CMD;	// 0=关闭主动上报, 1=开启主动上报(默认10ms周期)
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata); // ����CAN��Ϣ
}

/*******************************************************************************
* @����     		: RobStride���Э���޸� ��ͨ������25��
* @����      		: Э�����ͣ�		00��˽��Э�飩
										01��Canopen��
										02��MITЭ�飩
* @����ֵ 				: void
* @����  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_MIT_MotorModeSet(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};				//��������
//��������
	FDCAN_TxHeaderTypeDef txMsg = {0};
txMsg.IdType = FDCAN_STANDARD_ID;
txMsg.TxFrameType = FDCAN_DATA_FRAME;
txMsg.DataLength = FDCAN_DLC_BYTES_8;
txMsg.Identifier = CAN_ID; // ???????StdId??
txMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
txMsg.BitRateSwitch = FDCAN_BRS_OFF;
txMsg.FDFormat = FDCAN_CLASSIC_CAN;
txMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
txMsg.MessageMarker = 0;

	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFD;	//��һ�ֽ�����ν����ʲô�����ԣ�����д����0x08
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txMsg, txdata); // ����CAN��Ϣ
}


/*******************************************************************************
* @����     		: RobStride������ݵĲ�����ַ��ʼ��
* @����         : ���ݵĲ�����ַ����
* @����ֵ 			: void
* @����  				: ���ڴ��������ʱ�Զ�����
*******************************************************************************/
data_read_write::data_read_write(const uint16_t *index_list)
{
	run_mode.index = index_list[0];
	iq_ref.index = index_list[1];
	spd_ref.index = index_list[2];
	imit_torque.index = index_list[3];
	cur_kp.index = index_list[4];
	cur_ki.index = index_list[5];
	cur_filt_gain.index = index_list[6];
	loc_ref.index = index_list[7];
	limit_spd.index = index_list[8];
	limit_cur.index = index_list[9];
	mechPos.index = index_list[10];
	iqf.index = index_list[11];
	mechVel.index = index_list[12];
	VBUS.index = index_list[13];	
	rotation.index = index_list[14];
}

void RobStride_Motor::RobStride_Motor_MotorModeSet(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};
	FDCAN_TxHeaderTypeDef TxMessage = {0};
TxMessage.IdType = FDCAN_EXTENDED_ID;
TxMessage.TxFrameType = FDCAN_DATA_FRAME;
TxMessage.DataLength = FDCAN_DLC_BYTES_8;
TxMessage.Identifier = Communication_Type_MotorModeSet<<24|Master_CAN_ID<<8|CAN_ID;
TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxMessage.MessageMarker = 0;

	txdata[6] = F_CMD;	// 00=Private, 01=Canopen, 02=MIT (需断电重启生效)
  	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, txdata);
}
