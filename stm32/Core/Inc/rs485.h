#ifndef __RS485_H
#define __RS485_H

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"

/* ============ RS485 编码器通信参数 ============ */
#define RS485_BAUDRATE          2500000UL  // 编码器固定使用2.5Mbps
#define RS485_REC_BUFF_SIZE     200         // 接收缓冲区大小
#define RS485_TX_BUFF_SIZE      64          // 发送缓冲区大小

/* ============ DE引脚定义 ============ */
#define RS485_DE_HIGH()         HAL_GPIO_WritePin(RS485_DE1_GPIO_Port, RS485_DE1_Pin, GPIO_PIN_SET)
#define RS485_DE_LOW()          HAL_GPIO_WritePin(RS485_DE1_GPIO_Port, RS485_DE1_Pin, GPIO_PIN_RESET)

/* ============ 编码器协议相关常量 ============ */
#define ENCODER_ADDR_MAX        0x6F        // EEPROM最大地址

/* ============ 数据ID代码 (Data ID Code) ============ */
#define ENC_DATA_ID_0           0x02        // 读取: 单圈数据 (AS0~AS2)
#define ENC_DATA_ID_1           0x8A        // 读取: 多圈数据 (AM0~AM2)
#define ENC_DATA_ID_2           0x92        // 读取: 编码器ID (EI)
#define ENC_DATA_ID_3           0x1A        // 读取: 全部数据 (单圈+多圈+ID+错误)
#define ENC_DATA_ID_6           0x32        // 写EEPROM
#define ENC_DATA_ID_8           0xC2        // 复位单圈数据
#define ENC_DATA_ID_C           0x62        // 复位多圈数据和错误
#define ENC_DATA_ID_D           0xEA        // 读EEPROM

/* ============ 全局变量 ============ */
extern volatile uint8_t RS485_REC_Flag;
extern volatile uint8_t RS485_buff[RS485_REC_BUFF_SIZE];
extern volatile uint32_t RS485_rec_counter;

extern uint8_t RS485_RX_BUF[64];
extern uint8_t RS485_RX_CNT;

/* ============ 底层RS485通信API ============ */
void RS485_Init(uint32_t bound);
void RS485_Send_Data(uint8_t *buf, uint8_t len);
void RS485_Receive_Data(uint8_t *buf, uint8_t *len);
void RS485_TX_Set(uint8_t en);
void RS485_UART_IRQHandler(void);  /* USART2中断处理函数 */

/* ============ 编码器高级API ============ */
uint8_t Encoder_Build_CM(uint8_t data_id);                 // 构建控制域字节
uint8_t Encoder_Read_SingleTurn(uint32_t *angle);           // 读取单圈角度
uint8_t Encoder_Read_MultiTurn(uint16_t *turns);            // 读取多圈数据
uint8_t Encoder_Read_EncoderID(uint8_t *id);                // 读取编码器ID
uint8_t Encoder_Read_All(uint32_t *angle, uint16_t *turns, uint8_t *id, uint8_t *error);
uint8_t Encoder_Reset_SingleTurn(void);                // 复位单圈
uint8_t Encoder_Reset_MultiTurn(void);                 // 复位多圈
uint8_t Encoder_Write_EEPROM(uint8_t addr, uint8_t data);        // 写EEPROM
uint8_t Encoder_Read_EEPROM(uint8_t addr, uint8_t *data);        // 读EEPROM

/* ============ 位置数据解析和转换API ============ */
float Encoder_AngleToDegree(uint32_t angle);             // 角度转度数
float Encoder_AngleToRadian(uint32_t angle);             // 角度转弧度
int32_t Encoder_GetTotalPosition(uint32_t angle, uint16_t turns); // 获取总位置
float Encoder_GetTotalAngle(int32_t total_pos);          // 总位置转度数
uint8_t Encoder_ParsePosition(uint8_t *rx_buf, uint32_t *angle, uint16_t *turns, uint8_t *id, uint8_t *error);
void Encoder_PrintPosition(uint32_t angle, uint16_t turns, uint8_t id, uint8_t error);
void Encoder_PrintRawData(uint8_t *buf, uint8_t len);

/* ============ CRC8 校验 ============ */
uint8_t CRC8_Calculate(uint8_t *buf, uint8_t len);
void CRC8_Table_Init(void);

/* ============ 编码器2 (USART3) ============ */
// DE引脚定义
#define RS485_2_DE_HIGH()      HAL_GPIO_WritePin(RS485_DE2_GPIO_Port, RS485_DE2_Pin, GPIO_PIN_SET)
#define RS485_2_DE_LOW()       HAL_GPIO_WritePin(RS485_DE2_GPIO_Port, RS485_DE2_Pin, GPIO_PIN_RESET)

// 全局变量声明
extern volatile uint8_t RS485_2_REC_Flag;
extern volatile uint8_t RS485_2_buff[RS485_REC_BUFF_SIZE];
extern volatile uint32_t RS485_2_rec_counter;

// 底层RS485通信API
void RS485_2_Init(uint32_t bound);
void RS485_2_Send_Data(uint8_t *buf, uint8_t len);
void RS485_2_TX_Set(uint8_t en);
void RS485_2_UART_IRQHandler(void);  /* USART3中断处理函数 */

// 编码器2高级API（完全复用编码器1的逻辑）
uint8_t Encoder2_Read_SingleTurn(uint32_t *angle);
uint8_t Encoder2_Read_MultiTurn(uint16_t *turns);
uint8_t Encoder2_Read_EncoderID(uint8_t *id);
uint8_t Encoder2_Read_All(uint32_t *angle, uint16_t *turns, uint8_t *id, uint8_t *error);
uint8_t Encoder2_Reset_SingleTurn(void);
uint8_t Encoder2_Reset_MultiTurn(void);

#endif
