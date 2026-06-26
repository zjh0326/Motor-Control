/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"

#include "usart.h"
#include "gpio.h"

#include "RobStride.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "rs485.h"
#include "fdcan.h"
#ifdef __cplusplus
}
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MASTER_ID 0xFD
#define MOTOR_ID 0x7F // 你的电机ID=127
// 电机使能运行ID（通信类型3）
#define MOTOR_ENABLE_ID ((0x03 << 24) | (0x00 << 16) | (MASTER_ID << 8) | MOTOR_ID)
// 运控模式控制ID（通信类型1）
#define MOTOR_CONTROL_ID ((0x01 << 24) | (0x00 << 16) | (MASTER_ID << 8) | MOTOR_ID)
// 测试帧ID（保留调试用）
#define TEST_FRAME_ID ((0x12 << 24) | (0x00 << 16) | (MASTER_ID << 8) | MOTOR_ID)

// AT帧格式：[0x41,0x54] + [4字节CAN_ID] + [1字节数据长度] + [N字节数据] + [0x0D,0x0A]
#define AT_FRAME_HEAD_1 0x41
#define AT_FRAME_HEAD_2 0x54
#define AT_FRAME_TAIL_1 0x0D
#define AT_FRAME_TAIL_2 0x0A
#define AT_FRAME_MAX_LEN 20 // 最大帧长度


void AT_SendFrame(uint32_t can_id, uint8_t *data, uint8_t len);
// AT帧解析状态机
typedef enum {
    AT_STATE_WAIT_HEAD1,
    AT_STATE_WAIT_HEAD2,
    AT_STATE_READ_CAN_ID,
    AT_STATE_READ_LEN,
    AT_STATE_READ_DATA,
    AT_STATE_WAIT_TAIL1,
    AT_STATE_WAIT_TAIL2
} AT_ParseState;

// AT帧结构体
typedef struct {
    uint32_t can_id;
    uint8_t data_len;
    uint8_t data[8];
} AT_Frame;

extern "C" int fputc(int ch, FILE *f) {
    // 把printf的字符也放到AT发送队列，避免抢占
    // 注意：printf会增加串口负载，调试完建议关闭
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

//所有串口输出（包括printf和编码器/电机AT帧）都走同一个DMA环形缓冲区，完全无冲突。
//extern "C" int fputc(int ch, FILE *f) {
//    // 把printf的字符存入临时缓冲区，攒够一行再发送，减少DMA调用
//    static uint8_t printf_buf[64];
//    static uint8_t printf_len = 0;
//    
//    printf_buf[printf_len++] = (uint8_t)ch;
//    // 遇到换行或者缓冲区满，就通过AT帧发送（自定义ID=0x400，Unity过滤即可）
//    if (ch == '\n' || printf_len >= 64) {
//        AT_SendFrame(0x400, printf_buf, printf_len);
//        printf_len = 0;
//    }
//    return ch;
//}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// ✅ 增大缓冲区到64帧，可以存64ms的数据，足够应对突发情况
#define AT_TX_RING_SIZE 64
static uint8_t at_tx_ring_buf[AT_TX_RING_SIZE][AT_FRAME_MAX_LEN];
static volatile uint8_t at_tx_ring_head = 0;
static volatile uint8_t at_tx_ring_tail = 0;
static volatile uint8_t at_tx_busy = 0;
// ✅ 增加丢帧统计
static volatile uint32_t at_tx_dropped_frames = 0;
// AT帧解析相关
uint8_t at_rx_buf[AT_FRAME_MAX_LEN];
uint8_t at_rx_ptr = 0;
AT_ParseState at_parse_state = AT_STATE_WAIT_HEAD1;
AT_Frame at_rx_frame;

  uint8_t receiveData[50];

uint8_t pRxdata[8], pTxdata[8]; 
RobStride_Motor RobStride_01(0x7F, false);

uint8_t mode = 0; 

	// 保存最后一条Unity发来的CAN命令，用于定期轮询触发反馈
static AT_Frame last_can_cmd = {0};

// FDCAN全局变量
uint8_t FDCAN_RxData[8];
uint32_t FDCAN_RxId;
uint8_t FDCAN_RxLen;
uint8_t FDCAN_RxFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

extern DMA_HandleTypeDef hdma_usart1_rx;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        at_tx_ring_tail = (at_tx_ring_tail + 1) % AT_TX_RING_SIZE;
        if (at_tx_ring_head != at_tx_ring_tail)
        {
            uint8_t tx_len = 9 + at_tx_ring_buf[at_tx_ring_tail][6];
            HAL_UART_Transmit_DMA(&huart1, at_tx_ring_buf[at_tx_ring_tail], tx_len);
        }
        else
        {
            at_tx_busy = 0;
        }
    }
}

//CAN 


/************************** 节点5：反馈数据解析 **************************/
typedef struct {
    float position;  // 当前位置(rad)
    float speed;     // 当前速度(rad/s)
    float current;   // 当前电流(A)
    float temp;      // 当前温度(℃)
    uint8_t error;   // 错误码
    const char* error_msg; // 错误信息
} Motor_Feedback_t;

// ✅ 厂家官方错误码解析表（手册4.1.10）
const char* Motor_GetErrorMsg(uint8_t error)
{
    if(error == 0x00) return "no error";
    if(error & 0x01) return "over voltage";    // 过压
    if(error & 0x02) return "driver fault";    // 驱动芯片故障
    if(error & 0x04) return "over temp";       // 过温
    if(error & 0x08) return "encoder fault";   // 磁编码故障
    if(error & 0x10) return "overload";        // 过载
    if(error & 0x20) return "not calibrated";  // 未标定
    if(error & 0x80) return "hall fault";      // HALL编码故障
    return "unknown error";
}



extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	
 if(huart == &huart1)
 {
 //  HAL_UART_Transmit_DMA(&huart1, receiveData,Size);	// ★ echo 回 Unity
	 
	 for(int i = 0; i < Size; i++)
        {
            uint8_t ch = receiveData[i];
            switch(at_parse_state)
            {
                case AT_STATE_WAIT_HEAD1:
                    if(ch == AT_FRAME_HEAD_1) at_parse_state = AT_STATE_WAIT_HEAD2;
                    break;
                case AT_STATE_WAIT_HEAD2:
                    if(ch == AT_FRAME_HEAD_2)
                    {
                        at_parse_state = AT_STATE_READ_CAN_ID;
                        at_rx_ptr = 0;
                    }
                    else at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                case AT_STATE_READ_CAN_ID:
                    at_rx_buf[at_rx_ptr++] = ch;
                    if(at_rx_ptr == 4)
                    {
                        // 解析CAN_ID（大端序）
                        at_rx_frame.can_id = (at_rx_buf[0] << 24) | (at_rx_buf[1] << 16) | (at_rx_buf[2] << 8) | at_rx_buf[3];
                        at_parse_state = AT_STATE_READ_LEN;
                        at_rx_ptr = 0;
                    }
                    break;
                case AT_STATE_READ_LEN:
                    at_rx_frame.data_len = ch;
                    if(at_rx_frame.data_len > 8) at_rx_frame.data_len = 8; // 限制最大8字节
                    at_parse_state = AT_STATE_READ_DATA;
                    at_rx_ptr = 0;
                    break;
                case AT_STATE_READ_DATA:
                    at_rx_frame.data[at_rx_ptr++] = ch;
                    if(at_rx_ptr == at_rx_frame.data_len)
                    {
                        at_parse_state = AT_STATE_WAIT_TAIL1;
                    }
                    break;
                case AT_STATE_WAIT_TAIL1:
                    if(ch == AT_FRAME_TAIL_1) at_parse_state = AT_STATE_WAIT_TAIL2;
                    else at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                case AT_STATE_WAIT_TAIL2:
									
                    if(ch == AT_FRAME_TAIL_2)
                    {
                        // ✅ 收到完整AT帧，发送到CAN总线
                        FDCAN_SendExtFrame(at_rx_frame.can_id, at_rx_frame.data, at_rx_frame.data_len);
                        // 保存最后一条命令用于定期轮询触发反馈
                        // 过滤 GetParam(0x11) — 避免覆盖运动指令导致反馈中断
                        uint32_t rx_commType = (at_rx_frame.can_id >> 24) & 0xFF;
                        if (rx_commType != 0x11) {
                            last_can_cmd.can_id = at_rx_frame.can_id;
                            last_can_cmd.data_len = at_rx_frame.data_len;
                            memcpy(last_can_cmd.data, at_rx_frame.data, at_rx_frame.data_len);
                        }
                        // 翻转LED2表示收到指令
                        HAL_GPIO_TogglePin(GPIOE, LED2_Pin);
                    }
                    at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
                default:
                    at_parse_state = AT_STATE_WAIT_HEAD1;
                    break;
            }
        }
    }
//	 if(receiveData[0]==0xAA)
//	 {
//	  if (receiveData[1]==Size)
//		 {
//		  uint8_t sum=0;
//			for (int i=0;i<Size-1 ; i++)  sum+=receiveData[i];
//			if (sum==receiveData[Size -1])
//			{
//			
//		  	 for (int i=2;i<Size-1;i+=2)
//				{
//					GPIO_PinState state = GPIO_PIN_SET;
//				  if (receiveData[i+1] == 0x00)  state = GPIO_PIN_RESET;
//					if (receiveData[i] == 0x01)
//					HAL_GPIO_WritePin(GPIOE,LED1_Pin, state) ;
//					else if (receiveData[i] == 0x02)
//					HAL_GPIO_WritePin(GPIOE,LED2_Pin, state) ;
//					else if (receiveData[i] == 0x03)
//					HAL_GPIO_WritePin(GPIOE,LED3_Pin, state) ;
//				}
//			}
//		 }
	
	 
	 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData,sizeof(receiveData));
   __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
 }

// ✅ 打印丢帧统计
void AT_PrintStats(void) {
    if (at_tx_dropped_frames > 0) {
        printf("Warning: Dropped %lu AT frames due to buffer full\r\n", at_tx_dropped_frames);
        at_tx_dropped_frames = 0;
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
	  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	  // ✅ 新增：清空所有缓冲区，避免上电噪声触发错误解析
  memset(receiveData, 0, sizeof(receiveData));
  memset(at_rx_buf, 0, sizeof(at_rx_buf));
  memset((void*)RS485_buff, 0, RS485_REC_BUFF_SIZE);
  
	
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

 
  /* 初始化RS485 */
	  RS485_Init(2500000);
printf("RS485 Init Done\r\n");

/* 新增：初始化编码器2 (USART3) */
  MX_USART3_UART_Init();
  RS485_2_Init(2500000);
printf("Encoder 2 (USART3) Init Done\r\n");

// 配置FDCAN
    FDCAN_FilterConfig();
     HAL_FDCAN_Start(&hfdcan1);
 		
		RobStride_01.RobStride_Motor_ProactiveEscalationSet(1);

// ✅ 先测试基础通信，再初始化厂家类库

  // ✅ 初始化厂家类库相关功能

  RobStride_01.Disenable_Motor(1);
  HAL_Delay(200);
  RobStride_01.Set_RobStride_Motor_parameter(0x7028, 0.0f, Set_parameter);
  HAL_Delay(200);
  printf("HALL fault detection closed\r\n");
	  // 4. 所有初始化完成后再激活FDCAN接收中断，避免冲突
HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

   uint8_t motor_running = 0;
				  /* 磁编码器相关变量定义（移到循环外部，避免反复重置） */
   uint32_t enc_angle = 0;                  // 存储21位原始角度值
   uint8_t enc_ret = 1;                     // 编码器读取返回值：0成功，1长度错误，2CRC错误
   static uint32_t last_poll = 0;           // CAN轮询时间戳
   static uint32_t last_encoder_read = 0;   // 编码器读取时间戳
   static uint32_t last_encoder_print = 0;  // 角度打印时间戳
   
   /* 新增：编码器2相关变量 */
   uint32_t enc2_angle = 0;                // 编码器2的21位原始角度值
   uint8_t enc2_ret = 2;                   // 编码器2读取返回值
   static uint32_t last_encoder2_read = 0;   // 编码器2读取时间戳
   static uint32_t last_encoder2_print = 0;  // 编码器2角度打印时间戳
   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
			 /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_tick = HAL_GetTick();
    
    // 1. CAN轮询发送：每20ms一次（非阻塞）
    if (current_tick - last_poll >= 20 && last_can_cmd.data_len > 0) {
        FDCAN_SendExtFrame(last_can_cmd.can_id, last_can_cmd.data, last_can_cmd.data_len);
        last_poll = current_tick;
    }

					   // 每20ms读一次编码器，读取频率可根据编码器手册调整
		  /* ==================== 磁编码器读取逻辑 ==================== */
      // 2. 编码器读取：每20ms一次（与电机反馈同步）
    if (current_tick - last_encoder_read >= 20) {
			    last_encoder_read = current_tick;
			
			 // ---------- 编码器1读取 + 发送 ---------
        enc_ret = Encoder_Read_SingleTurn(&enc_angle);
    
        if (enc_ret == 0) {
            HAL_GPIO_TogglePin(GPIOE, LED2_Pin); // 只有读取成功才翻转LED
					
        /* 方式2：直接发送浮点角度值（Unity直接解析为float即可） */
    uint8_t can_data[8];
    memset(can_data, 0, 8);
    float angle_deg = Encoder_AngleToDegree(enc_angle);
    memcpy(can_data, &angle_deg, 4);  // 前4字节放浮点角度
    uint16_t raw_angle = (uint16_t)(enc_angle & 0xFFFF);  // 后2字节放原始值低16位
    memcpy(&can_data[4], &raw_angle, 2);
    AT_SendFrame(0x300, can_data, 6);  // 总共6字节有效数据
//					   printf("Raw: %lu (0x%06lX) | %.3f deg\r\n", 
//               enc_angle, enc_angle, Encoder_AngleToDegree(enc_angle));
        last_encoder_print = current_tick;
        }
		    else {
          /* 读取失败时每1秒打印一次错误，避免刷屏 */
          static uint32_t last_enc_err_print = 0;
          if (current_tick - last_enc_err_print >= 20) {
              printf("[ERR] Encoder read failed, ret=%d\r\n", enc_ret);
              last_enc_err_print = current_tick;
          }
				}
    
    // ---------- 编码器2读取 + 发送（和编码器1同周期执行）----------
    
        enc2_ret = Encoder2_Read_SingleTurn(&enc2_angle);
   
        if (enc2_ret == 0) {
            /* 发送编码器2数据到Unity，CAN ID = 0x301 */
            uint8_t can_data[8];
            memset(can_data, 0, 8);
            float angle_deg = Encoder_AngleToDegree(enc2_angle);
            memcpy(can_data, &angle_deg, 4);  // 前4字节放浮点角度
            uint16_t raw_angle = (uint16_t)(enc2_angle & 0xFFFF);  // 后2字节放原始值低16位
            memcpy(&can_data[4], &raw_angle, 2);
            AT_SendFrame(0x301, can_data, 6);  // 总共6字节有效数据
					
//              printf("Raw2: %lu (0x%06lX) | %.3f deg\r\n", 
//               enc2_angle, enc2_angle, Encoder_AngleToDegree(enc2_angle));
        last_encoder2_print = current_tick;
//  
        }
        else {
            /* 读取失败时每1秒打印一次错误 */
            static uint32_t last_enc2_err_print = 0;
            if (current_tick - last_enc2_err_print >= 1000) {
                printf("[ERR] Encoder2 read failed, ret=%d\r\n", enc2_ret);
                last_enc2_err_print = current_tick;
            }
        }
    }
  }
//    // 3. 角度打印：每100ms一次（非阻塞）
//    if (enc_ret == 0 && current_tick - last_encoder_print >= 100) {
//        printf("Raw: %lu (0x%06lX) | %.3f deg\r\n", 
//               enc_angle, enc_angle, Encoder_AngleToDegree(enc_angle));
//        last_encoder_print = current_tick;
//    }
    
//    // 4. LED1翻转：每50ms一次（非阻塞，替代原来的HAL_Delay）
//    static uint32_t last_led1_toggle = 0;
//    if (current_tick - last_led1_toggle >= 50) {
//        HAL_GPIO_TogglePin(GPIOE, LED1_Pin);
//        last_led1_toggle = current_tick;
//    }
           
//			uint8_t key = KEY_Scan();
//	if(key==KEY1_PRES ) {
//    RobStride_01.Enable_Motor();
//		motor_running = 0;
//		
//    printf("Enable\n");
//}
//else if(key==KEY2_PRES) {
//    RobStride_01.Disenable_Motor(1);
//	  motor_running = 1;
//    printf("Disable\n");
//}

//else if(key==KEY3_PRES && !motor_running) {
//    printf("Sending speed control command...\r\n");
//    RobStride_01.RobStride_Motor_Speed_control(5.0f, 10.0f);
//   
//    printf("Speed command sent\r\n");
//}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief 通过串口发送一个 CAN 帧（包装成原厂格式）
 * @param can_id  29位真实CAN ID
 * @param data    8字节数据
 * @param len     数据长度 (0~8)
 */
void AT_SendFrame(uint32_t can_id, uint8_t *data, uint8_t len)
{
    if (len > 8) len = 8;
    uint8_t next_head = (at_tx_ring_head + 1) % AT_TX_RING_SIZE;
    // ✅ 缓冲区满时统计丢帧
    if (next_head == at_tx_ring_tail) {
        at_tx_dropped_frames++;
        return;
    }
	
    uint8_t *buf = at_tx_ring_buf[at_tx_ring_head];
    buf[0] = AT_FRAME_HEAD_1;
    buf[1] = AT_FRAME_HEAD_2;
    buf[2] = (can_id >> 24) & 0xFF;
    buf[3] = (can_id >> 16) & 0xFF;
    buf[4] = (can_id >> 8) & 0xFF;
    buf[5] = can_id & 0xFF;
    buf[6] = len;
    memcpy(&buf[7], data, len);
    buf[7 + len] = 0x0D;
    buf[8 + len] = 0x0A;
    at_tx_ring_head = next_head;
    if (!at_tx_busy)
    {
        at_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, at_tx_ring_buf[at_tx_ring_tail], 9 + len);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t FDCAN_RxData[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, FDCAN_RxData) == HAL_OK)
        {
            HAL_GPIO_TogglePin(GPIOE, LED3_Pin);
            RobStride_01.RobStride_Motor_Analysis(FDCAN_RxData, RxHeader.Identifier);
           AT_SendFrame(RxHeader.Identifier, FDCAN_RxData, RxHeader.DataLength & 0x0F);
					
        }
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
      uint32_t err = HAL_FDCAN_GetError(hfdcan);
    // 清除所有错误状态
    HAL_FDCAN_Stop(hfdcan);
    HAL_FDCAN_Start(hfdcan);
	
    // 可选打印一次
   // printf("FDCAN Error: 0x%08lX\r\n", err);
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
