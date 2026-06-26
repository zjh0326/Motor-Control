/**
 ******************************************************************************
 * @file    rs485.c
 * @brief   RS485底层驱动 + 磁编码器通信协议实现 (适配STM32H723 HAL库)
 * @note    编码器通信参数: 2.5Mbps, 8N1, 被动响应模式
 ******************************************************************************
 */
#include "rs485.h"
#include "stdio.h"
#include "string.h"

/* ============ 全局变量 ============ */
volatile uint8_t RS485_REC_Flag = 0;
volatile uint8_t RS485_buff[RS485_REC_BUFF_SIZE];
volatile uint32_t RS485_rec_counter = 0;

uint8_t RS485_RX_BUF[64];
uint8_t RS485_RX_CNT = 0;

/* ============ CRC8 查表 ============ */
static uint8_t CRC8_TABLE[256];
static uint8_t crc8_table_inited = 0;

/* ============ 编码器命令帧缓冲区 ============ */
static uint8_t enc_tx_buf[16];

/* ============ USART2 中断处理 (在stm32h7xx_it.c中调用) ============ */
void RS485_UART_IRQHandler(void)
{
    uint8_t res;
    if (__HAL_UART_GET_IT(&huart2, UART_IT_RXNE) != RESET)
    {
        /* 直接读取接收寄存器 (非阻塞) */
        res = (uint8_t)(huart2.Instance->RDR & 0xFF);
        
        if (RS485_rec_counter < RS485_REC_BUFF_SIZE)
        {
            RS485_buff[RS485_rec_counter] = res;
            RS485_rec_counter++;
        }
    }
    /* 清除溢出错误 */
    if (__HAL_UART_GET_IT(&huart2, UART_IT_ORE) != RESET)
    {
        __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_OREF);
        /* 读取RDR以清除ORE标志 */
        (void)(huart2.Instance->RDR);
    }
}

/* ============ RS485 底层驱动 ============ */

/**
 * @brief  初始化RS485 (USART2)
 * @param  bound: 波特率 (编码器要求2.5Mbps)
 * @note   CubeMX已初始化USART2，此函数仅配置DE引脚和中断
 */
void RS485_Init(uint32_t bound)
{
    /* PD7已通过CubeMX配置为GPIO输出, 这里只需确认低电平(接收模式) */
    RS485_DE_LOW();
    printf("[INFO] RS485 DE pin (PD7) initialized\r\n");

    /* 使能USART2接收中断 */
    __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_TCF);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);

    /* 初始化CRC8表 */
    CRC8_Table_Init();

    printf("[INFO] RS485 init done, baudrate=%lu\r\n", bound);
}

/**
 * @brief  RS485发送数据
 */
void RS485_Send_Data(uint8_t *buf, uint8_t len)
{
    uint8_t i;

    RS485_TX_Set(1);  /* 切换到发送模式 */
    HAL_Delay(1);     /* 等待DE引脚稳定 */

    HAL_UART_Transmit(&huart2, buf, len, 1000);

    /* 等待发送完成 */
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);

    RS485_RX_CNT = 0;
    RS485_TX_Set(0);  /* 切换回接收模式 */
}

/**
 * @brief  RS485方向控制
 */
void RS485_TX_Set(uint8_t en)
{
    if (en == 0) RS485_DE_LOW();
    if (en == 1) RS485_DE_HIGH();
}

/* ============ CRC8 校验 ============ */

void CRC8_Table_Init(void)
{
    uint16_t i, j;
    uint8_t crc;

    for (i = 0; i < 256; i++)
    {
        crc = (uint8_t)i;
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x01);
            else
                crc = (uint8_t)(crc << 1);
        }
        CRC8_TABLE[i] = crc;
    }
    crc8_table_inited = 1;
}

uint8_t CRC8_Calculate(uint8_t *buf, uint8_t len)
{
    uint8_t crc_result = 0;
    uint8_t idx = 0;
    if (!crc8_table_inited) CRC8_Table_Init();
    while (idx < len)
    {
        crc_result ^= buf[idx];
        crc_result = CRC8_TABLE[crc_result];
        idx++;
    }
    return crc_result;
}

/* ============ 编码器命令帧构建 ============ */

uint8_t Encoder_Build_CM(uint8_t data_id)
{
    return data_id;  /* 直接返回Data ID Code */
}

/* ============ 编码器通信核心函数 ============ */

static uint8_t Encoder_sendCmd_GetResp(uint8_t *cmd_buf, uint8_t cmd_len, uint8_t expected_resp_len)
{
    uint32_t timeout_cnt;
    uint8_t old_counter;

    /* 清空接收缓冲区 */
    RS485_rec_counter = 0;
    RS485_REC_Flag = 0;
    memset((void*)RS485_buff, 0, RS485_REC_BUFF_SIZE);

    /* 发送命令 */
    RS485_Send_Data(cmd_buf, cmd_len);

    /* 等待响应 */
    timeout_cnt = 50000;
    while (RS485_rec_counter < expected_resp_len)
    {
        if (timeout_cnt == 0)
        {
            /* 调试信息：超时等待响应 */
            // printf("[DEBUG] Timeout waiting for %d bytes, got %d\r\n",
            //        expected_resp_len, RS485_rec_counter);
            return (uint8_t)RS485_rec_counter;
        }
        timeout_cnt--;
        __NOP(); __NOP(); __NOP(); __NOP();
    }

    /* 等待帧结束 */
    old_counter = RS485_rec_counter;
    HAL_Delay(1);

    if (RS485_rec_counter == old_counter)
    {
        return (uint8_t)RS485_rec_counter;
    }
    else
    {
        HAL_Delay(1);
        return (uint8_t)RS485_rec_counter;
    }
}

/* ============ 编码器数据读取API ============ */

uint8_t Encoder_Read_SingleTurn(uint32_t *angle)
{
    uint8_t resp_len;
    uint8_t crc_calc;
    uint8_t crc_recv;
    uint32_t raw_angle;

    enc_tx_buf[0] = Encoder_Build_CM(ENC_DATA_ID_0);

    resp_len = Encoder_sendCmd_GetResp(enc_tx_buf, 1, 6);
	
	
    if (resp_len < 6) return 1;

    crc_calc = CRC8_Calculate((uint8_t*)RS485_buff, 5);
    crc_recv = RS485_buff[5];
    if (crc_calc != crc_recv) return 2;

    /* Data ID 0响应格式: [CM][SA][D0][D1][D2][CRC] */
    /* 根据调试输出，正确的解析方式是： */
    raw_angle  = (uint32_t)RS485_buff[2];  /* D0: 角度低8位 */
    raw_angle |= (uint32_t)RS485_buff[3] << 8;  /* D1: 角度中8位 */
    raw_angle |= (uint32_t)RS485_buff[4] << 16;  /* D2: 角度高5位 */
    raw_angle &= 0x001FFFFF;  /* 屏蔽高11位，保留21位角度 */

    *angle = raw_angle;
    return 0;
}

uint8_t Encoder_Read_MultiTurn(uint16_t *turns)
{
    uint8_t resp_len;
    uint8_t crc_calc, crc_recv;
    uint16_t raw_turns;

    enc_tx_buf[0] = Encoder_Build_CM(ENC_DATA_ID_1);

    /* Data ID 1响应格式: [CM][SA][D0][D1][D2][CRC] = 6字节 */
    resp_len = Encoder_sendCmd_GetResp(enc_tx_buf, 1, 6);
    if (resp_len < 6) return 1;

    crc_calc = CRC8_Calculate((uint8_t*)RS485_buff, 5);
    crc_recv = RS485_buff[5];
    if (crc_calc != crc_recv) return 2;

    /* Data ID 1响应格式: [CM][SA][D0][D1][D2][CRC] */
    /* 根据调试输出，正确的解析方式是： */
    raw_turns  = (uint16_t)RS485_buff[2];  /* D0: 多圈低8位 */
    raw_turns |= (uint16_t)RS485_buff[3] << 8;  /* D1: 多圈高8位 */

    *turns = raw_turns;
    return 0;
}

uint8_t Encoder_Read_EncoderID(uint8_t *id)
{
    uint8_t resp_len;
    uint8_t crc_calc, crc_recv;

    enc_tx_buf[0] = Encoder_Build_CM(ENC_DATA_ID_2);

    resp_len = Encoder_sendCmd_GetResp(enc_tx_buf, 1, 4);
    if (resp_len < 4) return 1;

    crc_calc = CRC8_Calculate((uint8_t*)RS485_buff, 3);
    crc_recv = RS485_buff[3];
    if (crc_calc != crc_recv) return 2;

    *id = RS485_buff[2];
    return 0;
}

uint8_t Encoder_Read_All(uint32_t *angle, uint16_t *turns, uint8_t *id, uint8_t *error)
{
    uint8_t resp_len;
    uint8_t crc_calc, crc_recv;
    uint8_t i;

    enc_tx_buf[0] = Encoder_Build_CM(ENC_DATA_ID_3);

    resp_len = Encoder_sendCmd_GetResp(enc_tx_buf, 1, 11);

    /* 调试信息：注释掉以避免输出过多 */
    // printf("[DEBUG] Data ID 3: resp_len=%d\r\n", resp_len);
    // if (resp_len > 0)
    // {
    //     printf("[DEBUG] Raw response: ");
    //     for (i = 0; i < resp_len; i++)
    //     {
    //         printf("%02X ", RS485_buff[i]);
    //     }
    //     printf("\r\n");
    // }

    if (resp_len < 11) return 1;

    crc_calc = CRC8_Calculate((uint8_t*)RS485_buff, 10);
    crc_recv = RS485_buff[10];

    if (crc_calc != crc_recv) return 2;

    /* Data ID 3响应格式: [CM][SA][D0][D1][D2][D3][D4][D5][D6][D7][CRC] */
    /* 根据调试输出，正确的解析方式是： */
    *angle  = (uint32_t)RS485_buff[2];  /* D0: 角度低8位 */
    *angle |= (uint32_t)RS485_buff[3] << 8;  /* D1: 角度中8位 */
    *angle |= (uint32_t)RS485_buff[4] << 16;  /* D2: 角度高5位 */
    *angle &= 0x001FFFFF;  /* 屏蔽高11位，保留21位角度 */

    *id = RS485_buff[5];  /* D3: 编码器ID */

    *turns  = (uint16_t)RS485_buff[6];  /* D4: 多圈低8位 */
    *turns |= (uint16_t)RS485_buff[7] << 8;  /* D5: 多圈高8位 */

    *error = RS485_buff[8];  /* D6: 错误标志 */

    return 0;
}

uint8_t Encoder_Reset_SingleTurn(void)
{
    uint8_t i;
    enc_tx_buf[0] = Encoder_Build_CM(ENC_DATA_ID_8);

    for (i = 0; i < 10; i++)
    {
        RS485_Send_Data(enc_tx_buf, 1);
        HAL_Delay(1);
    }

    HAL_Delay(600);
    return 0;
}

uint8_t Encoder_Reset_MultiTurn(void)
{
    uint8_t i;
    enc_tx_buf[0] = Encoder_Build_CM(ENC_DATA_ID_C);

    for (i = 0; i < 10; i++)
    {
        RS485_Send_Data(enc_tx_buf, 1);
        HAL_Delay(1);
    }

    HAL_Delay(600);
    return 0;
}

/* ============ 位置数据解析和转换函数 ============ */

float Encoder_AngleToDegree(uint32_t angle)
{
    return (float)angle * 360.0f / 2097152.0f;
}

float Encoder_AngleToRadian(uint32_t angle)
{
    return (float)angle * 2.0f * 3.1415926535f / 2097152.0f;
}

int32_t Encoder_GetTotalPosition(uint32_t angle, uint16_t turns)
{
    int32_t total;
    int16_t signed_turns = (int16_t)turns;
    total = (int32_t)signed_turns * 2097152 + (int32_t)angle;
    return total;
}

float Encoder_GetTotalAngle(int32_t total_pos)
{
    return (float)total_pos * 360.0f / 2097152.0f;
}

void Encoder_PrintPosition(uint32_t angle, uint16_t turns, uint8_t id, uint8_t error)
{
    float degree = Encoder_AngleToDegree(angle);
    int32_t total_pos = Encoder_GetTotalPosition(angle, turns);
    float total_angle = Encoder_GetTotalAngle(total_pos);

    printf("  Encoder Position Information\n");
    printf("  Encoder ID: 0x%02X (%d)\n", id, id);
    printf("  Single-turn Angle: %lu (0x%06lX) | %.3f°\n",
           angle, angle, degree);
    printf("  Multi-turn Data: %u (0x%04X)\n", turns, turns);
    printf("  Total Angle: %.3f°\n", total_angle);
    printf("  Error Flags (AMC): 0x%02X\n", error);
    if (error == 0)
        printf("  -> No error\n");
    else
    {
        if (error & (1 << 2))  printf("  -> Counting error!\n");
        if (error & (1 << 5))  printf("  -> Multi-turn error!\n");
        if (error & (1 << 6))  printf("  -> Battery error!\n");
        if (error & (1 << 7))  printf("  -> Battery alarm!\n");
    }
}

/* ============ 编码器2实现 (USART3, 完全复用编码器1的逻辑) ============ */

/* 编码器2全局变量 */
volatile uint8_t RS485_2_REC_Flag = 0;
volatile uint8_t RS485_2_buff[RS485_REC_BUFF_SIZE];
volatile uint32_t RS485_2_rec_counter = 0;
static uint8_t enc2_tx_buf[16];

/* ============ USART3 中断处理 ============ */
void RS485_2_UART_IRQHandler(void)
{
    uint8_t res;
    if (__HAL_UART_GET_IT(&huart3, UART_IT_RXNE) != RESET)
    {
        /* 直接读取接收寄存器 (非阻塞) */
        res = (uint8_t)(huart3.Instance->RDR & 0xFF);
        
        if (RS485_2_rec_counter < RS485_REC_BUFF_SIZE)
        {
            RS485_2_buff[RS485_2_rec_counter] = res;
            RS485_2_rec_counter++;
        }
    }
    /* 清除溢出错误 */
    if (__HAL_UART_GET_IT(&huart3, UART_IT_ORE) != RESET)
    {
        __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_OREF);
        /* 读取RDR以清除ORE标志 */
        (void)(huart3.Instance->RDR);
    }
}

/* ============ RS485_2 底层驱动 ============ */

/**
 * @brief  初始化RS485_2 (USART3)
 * @param  bound: 波特率 (编码器要求2.5Mbps)
 */
void RS485_2_Init(uint32_t bound)
{
    /* 关键：重新设置波特率并初始化USART3 */
    huart3.Init.BaudRate = bound;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        printf("[ERR] USART3 re-init failed!\n");
    }
    else
    {
        printf("[INFO] USART3 re-init ok, baudrate=%lu\n", huart3.Init.BaudRate);
    }

    /* PD10已通过CubeMX配置为GPIO输出, 这里只需确认低电平(接收模式) */
    RS485_2_DE_LOW();
    printf("[INFO] RS485_2 DE pin (PD10) initialized\n");

    /* 使能USART3接收中断 */
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_TCF);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);  /* 优先级低于USART2，避免冲突 */

    printf("[INFO] RS485_2 init done, baudrate=%lu\n", bound);
}

/**
 * @brief  RS485_2发送数据
 */
void RS485_2_Send_Data(uint8_t *buf, uint8_t len)
{
    uint8_t i;

    RS485_2_TX_Set(1);  /* 切换到发送模式 */
    HAL_Delay(1);     /* 等待DE引脚稳定 */

    HAL_UART_Transmit(&huart3, buf, len, 1000);

    /* 等待发送完成 */
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET);

    RS485_2_TX_Set(0);  /* 切换回接收模式 */
}

/**
 * @brief  RS485_2方向控制
 */
void RS485_2_TX_Set(uint8_t en)
{
    if (en == 0) RS485_2_DE_LOW();
    if (en == 1) RS485_2_DE_HIGH();
}

/* ============ 编码器2通信核心函数 ============ */

static uint8_t Encoder2_sendCmd_GetResp(uint8_t *cmd_buf, uint8_t cmd_len, uint8_t expected_resp_len)
{
    uint32_t timeout_cnt;
    uint8_t old_counter;

    /* 清空接收缓冲区 */
    RS485_2_rec_counter = 0;
    RS485_2_REC_Flag = 0;
    memset((void*)RS485_2_buff, 0, RS485_REC_BUFF_SIZE);

    /* 发送命令 */
    RS485_2_Send_Data(cmd_buf, cmd_len);

    /* 等待响应 */
    timeout_cnt = 50000;
    while (RS485_2_rec_counter < expected_resp_len)
    {
        if (timeout_cnt == 0)
        {
            return (uint8_t)RS485_2_rec_counter;
        }
        timeout_cnt--;
        __NOP(); __NOP(); __NOP(); __NOP();
    }

    /* 等待帧结束 */
    old_counter = RS485_2_rec_counter;
    HAL_Delay(1);

    if (RS485_2_rec_counter == old_counter)
    {
        return (uint8_t)RS485_2_rec_counter;
    }
    else
    {
        HAL_Delay(1);
        return (uint8_t)RS485_2_rec_counter;
    }
}

/* ============ 编码器2数据读取API ============ */

uint8_t Encoder2_Read_SingleTurn(uint32_t *angle)
{
    uint8_t resp_len;
    uint8_t crc_calc;
    uint8_t crc_recv;
    uint32_t raw_angle;

    enc2_tx_buf[0] = 0x02;  /* ENC_DATA_ID_0 */

    resp_len = Encoder2_sendCmd_GetResp(enc2_tx_buf, 1, 6);

    if (resp_len < 6) return 1;

    crc_calc = CRC8_Calculate((uint8_t*)RS485_2_buff, 5);
    crc_recv = RS485_2_buff[5];
    if (crc_calc != crc_recv) return 2;

    /* Data ID 0响应格式: [CM][SA][D0][D1][D2][CRC] */
    raw_angle  = (uint32_t)RS485_2_buff[2];  /* D0: 角度低8位 */
    raw_angle |= (uint32_t)RS485_2_buff[3] << 8;  /* D1: 角度中8位 */
    raw_angle |= (uint32_t)RS485_2_buff[4] << 16;  /* D2: 角度高5位 */
    raw_angle &= 0x001FFFFF;  /* 屏蔽高11位，保留21位角度 */

    *angle = raw_angle;
    return 0;
}

uint8_t Encoder2_Read_MultiTurn(uint16_t *turns)
{
    uint8_t resp_len;
    uint8_t crc_calc, crc_recv;
    uint16_t raw_turns;

    enc2_tx_buf[0] = 0x8A;  /* ENC_DATA_ID_1 */

    resp_len = Encoder2_sendCmd_GetResp(enc2_tx_buf, 1, 6);
    if (resp_len < 6) return 1;

    crc_calc = CRC8_Calculate((uint8_t*)RS485_2_buff, 5);
    crc_recv = RS485_2_buff[5];
    if (crc_calc != crc_recv) return 2;

    raw_turns  = (uint16_t)RS485_2_buff[2];  /* D0: 多圈低8位 */
    raw_turns |= (uint16_t)RS485_2_buff[3] << 8;  /* D1: 多圈高8位 */

    *turns = raw_turns;
    return 0;
}

uint8_t Encoder2_Read_EncoderID(uint8_t *id)
{
    uint8_t resp_len;
    uint8_t crc_calc, crc_recv;

    enc2_tx_buf[0] = 0x92;  /* ENC_DATA_ID_2 */

    resp_len = Encoder2_sendCmd_GetResp(enc2_tx_buf, 1, 4);
    if (resp_len < 4) return 1;

    crc_calc = CRC8_Calculate((uint8_t*)RS485_2_buff, 3);
    crc_recv = RS485_2_buff[3];
    if (crc_calc != crc_recv) return 2;

    *id = RS485_2_buff[2];
    return 0;
}

uint8_t Encoder2_Read_All(uint32_t *angle, uint16_t *turns, uint8_t *id, uint8_t *error)
{
    uint8_t resp_len;
    uint8_t crc_calc, crc_recv;
    uint8_t i;

    enc2_tx_buf[0] = 0x1A;  /* ENC_DATA_ID_3 */

    resp_len = Encoder2_sendCmd_GetResp(enc2_tx_buf, 1, 11);

    if (resp_len < 11) return 1;

    crc_calc = CRC8_Calculate((uint8_t*)RS485_2_buff, 10);
    crc_recv = RS485_2_buff[10];

    if (crc_calc != crc_recv) return 2;

    /* Data ID 3响应格式: [CM][SA][D0][D1][D2][D3][D4][D5][D6][D7][CRC] */
    *angle  = (uint32_t)RS485_2_buff[2];  /* D0: 角度低8位 */
    *angle |= (uint32_t)RS485_2_buff[3] << 8;  /* D1: 角度中8位 */
    *angle |= (uint32_t)RS485_2_buff[4] << 16;  /* D2: 角度高5位 */
    *angle &= 0x001FFFFF;  /* 屏蔽高11位，保留21位角度 */

    *id = RS485_2_buff[5];  /* D3: 编码器ID */

    *turns  = (uint16_t)RS485_2_buff[6];  /* D4: 多圈低8位 */
    *turns |= (uint16_t)RS485_2_buff[7] << 8;  /* D5: 多圈高8位 */

    *error = RS485_2_buff[8];  /* D6: 错误标志 */

    return 0;
}

uint8_t Encoder2_Reset_SingleTurn(void)
{
    uint8_t i;
    enc2_tx_buf[0] = 0xC2;  /* ENC_DATA_ID_8 */

    for (i = 0; i < 10; i++)
    {
        RS485_2_Send_Data(enc2_tx_buf, 1);
        HAL_Delay(1);
    }

    HAL_Delay(600);
    return 0;
}

uint8_t Encoder2_Reset_MultiTurn(void)
{
    uint8_t i;
    enc2_tx_buf[0] = 0x62;  /* ENC_DATA_ID_C */

    for (i = 0; i < 10; i++)
    {
        RS485_2_Send_Data(enc2_tx_buf, 1);
        HAL_Delay(1);
    }

    HAL_Delay(600);
    return 0;
}
