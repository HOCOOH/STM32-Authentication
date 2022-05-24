/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "SM3.h"

/* USER CODE BEGIN Includes */
#include "zlg7290.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ZLG_READ_ADDRESS1         0x01
#define ZLG_READ_ADDRESS2         0x10
#define ZLG_WRITE_ADDRESS1        0x10
#define ZLG_WRITE_ADDRESS2        0x11
#define BUFFER_SIZE1              (countof(Tx1_Buffer))
#define BUFFER_SIZE2              (countof(Rx2_Buffer))
#define countof(a) (sizeof(a) / sizeof(*(a)))

/*
#define MAX_USERNAME_LEN 6
#define MIN_USERNAME_LEN 3
#define MAX_PASSWORD_LEN 8
#define MIN_PASSWORD_LEN 5
#define N_WORKER 5
*/

#define STATE_INIT 0
#define STATE_WAIT 1
#define STATE_INPUT 2
#define STATE_CHECK 3
#define STATE_EDIT 4
#define STATE_EXCEPTION 5

#define MAX_PASSWORD_LEN 8
#define MIN_PASSWORD_LEN 5
#define DIGEST_LEN 32

#define REFRESH_COUNT_INIT 500

typedef struct {
	u8 currentState;
	u8 lastState;
	u8 errerCode;
} State;

u32 refreshCount = 0;
State state = {0, 0, 0};
uint8_t flag;//不同的按键有不同的标志位值
uint8_t flagInterrupt = 0;//中断标志位，每次按键产生一次中断，并开始读取8个数码管的值
uint8_t Rx2_Buffer[8]={0};
uint8_t Tx1_Buffer[8]={0};
uint8_t Rx1_Buffer[1]={0};
uint8_t edit_mode = 0;
uint8_t status_code = 0;

uint8_t pass_buf[MAX_PASSWORD_LEN+1] = {0};
uint8_t pass_buf_sm3[DIGEST_LEN+1] = {0};
uint8_t pass_store[MAX_PASSWORD_LEN+1] = {0x1, 0x2, 0x3, 0x4, 0, 0, 0, 0};
uint8_t pass_store_sm3[DIGEST_LEN+1] = {0};
uint8_t input_cursor = 0;
/*
uint8_t un_buf[MAX_USERNAME_LEN] = {0};
uint8_t pass_buf[MAX_PASSWORD_LEN] = {0};
uint8_t username[N_WORKER][MAX_USERNAME_LEN] = {0};
uint8_t password[N_WORKER][MAX_PASSWORD_LEN]={0};
uint8_t user_cursor = 0;
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void swtich_key(void);
void switch_flag(void);
void disp_str(uint8_t* code);
uint8_t cmp(uint8_t* arr1, uint8_t* arr2, uint8_t len);
uint32_t len(uint8_t* arr);
void disp_in_serial(uint8_t* arr);
void UpdateState(u8 nextState);
u8 IsStateValid(u8 expectState);

int main(void) {
	// printf("\n\r");
	// printf("\n\r===================================\r\n");
	// printf("\n\r FS-STM32身份验证系统\r\n");
	// printf("\n test : len(pass_store):%d\n", len(pass_store));
	// printf("\n test : pass_store:\n\r");
	// disp_in_serial(pass_store);
	
	// SM3_Hash(pass_store, len(pass_store), (void*)pass_store_sm3);
	// printf("SM3哈希后的pass_store:\n");
	// disp_in_serial(pass_store_sm3);
	// printf("\n");

	// 主循环
	while (1) {
		if (state.currentState == STATE_INIT) {	// 初始化
			/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
			HAL_Init();

			/* Configure the system clock */
			SystemClock_Config();

			/* Initialize all configured peripherals */
			MX_GPIO_Init();
			MX_I2C1_Init();
			MX_USART1_UART_Init();

			refreshCount = REFRESH_COUNT_INIT;

			// switch to state wait
			UpdateState(STATE_WAIT);
		}
		else if (state.currentState == STATE_WAIT) {	// 待机
			if (!IsStateValid(STATE_WAIT)) {
				UpdateState(STATE_EXCEPTION);
				continue;
			}
			if (flagInterrupt == 1) {
				UpdateState(STATE_INPUT);
				continue;
			}
			// set refresh timer 
			if (refreshCount-- == 0) {
				UpdateState(STATE_INIT);
				continue;
			}
		}
		else if (state.currentState == STATE_INPUT) {	// 输入
			if (!IsStateValid(STATE_INPUT)) {
				UpdateState(STATE_EXCEPTION);
				continue;
			}

		}
		else if (state.currentState == STATE_CHECK) {	// 判断密码
			if (!IsStateValid(STATE_CHECK)) {
				UpdateState(STATE_EXCEPTION);
				continue;
			}

		}
		else if (state.currentState == STATE_EDIT) {	// 修改密码
			if (!IsStateValid(STATE_EDIT)) {
				UpdateState(STATE_EXCEPTION);
				continue;
			}
		}
		else {	// default 异常
			UpdateState(STATE_INIT);
		}
	}




	/* 给pass_store初始化 */

	/* 计算pass_store SM3摘要 */

	// 接收密码输入
	while (1) {

		if(flagInterrupt == 1 && input_cursor < MAX_PASSWORD_LEN) {
			flagInterrupt = 0;
			I2C_ZLG7290_Read(&hi2c1,0x71,0x01,Rx1_Buffer,1);//读键值
			swtich_key();//扫描键值，写标志位
			I2C_ZLG7290_Read(&hi2c1,0x71,0x10,Rx2_Buffer,8);//读8位数码管
			switch_flag();//扫描到相应的按键并且向数码管写进数值
			printf("\n\rflag:%d\n\r", flag);
			if (flag == 14) {//
				printf("密码输入完毕\n");
				break;
			} else if (flag == 16 && input_cursor == 0) { // 首字符按键为*
				// 修改密码模式
				edit_mode = 1;
			} else if (flag == 16 && input_cursor != 0) { // 非首字符按键为*
				
				// 异常!!!:无效的字符
			} else {
				pass_buf[input_cursor++] = flag;
			}
		}
  }

	/* 检查长度问题 */
	if (input_cursor >= MAX_PASSWORD_LEN) {
		// 异常!!!:密码长度过长
	} else if (input_cursor < MIN_PASSWORD_LEN) {
		// 异常!!!:密码长度过短
	}

	/* 更新密码或者身份验证 */
	if (edit_mode == 1) {
		// 更新密码
		int new_len = (len(pass_buf) > len(pass_store)) ? len(pass_buf) : len(pass_store);
		for (int i = 0; i < MAX_PASSWORD_LEN; i++) {
			if (i < new_len)
				pass_store[i] = pass_buf[i];
			else
				pass_store[i] = 0;
		}
		// 显示update
		uint8_t up_code[] = {0xCE, 0x7C};
		disp_str(up_code);
		
		// 设置status_code

	} else { // 进行验证
		
		printf("pass_buf:\n");
		disp_in_serial(pass_buf_sm3);
		
		/* 对输入的密码进行SM3摘要 */
		SM3_Hash(pass_buf, len(pass_buf), (void*)pass_buf_sm3);
		printf("SM3哈希后的pass_store:\n");
		disp_in_serial(pass_buf_sm3);
		printf("\n");
		
		/* 比对输入摘要和真实密码摘要 */
		int cmp_res = cmp(pass_buf_sm3, pass_store_sm3, DIGEST_LEN);
		printf("cmp_res:%d\n", cmp_res);
		if (cmp_res == 0) { // 匹配
			// 显示success
			printf("密码匹配\n");
			uint8_t success_code[] = {0xB6, 0xB6, 0x9E, 0x9C, 0x9C, 0x7C, 0xB6};
			disp_str(success_code);
		
		} else { // 不匹配
			// 显示error
			printf("密码不匹配\n");
			uint8_t error_code[] = {0xB6, 0xB6, 0x9E, 0x9C, 0x9C, 0x7C, 0xB6};
			disp_str(error_code);
		}
		// 设置status_code
	}
}

u8 IsStateValid(u8 expectState) {
	if (state.currentState != expectState) {
		return 1;
	}
	// todo: check last state

	return 0;
}

void UpdateState(u8 nextState) {
	// TODO: check if next state is vaild
	state.lastState = state.currentState;
	state.currentState = nextState;

	// TODO: edit error code 

	// todo: 备份state
}

void disp_str(uint8_t* code) {
	printf("Enter function disp_str!\n");
	for (int i = 0;i < len(code); i++) {
		I2C_ZLG7290_Read(&hi2c1,0x71,0x10,Rx2_Buffer,8);
		Tx1_Buffer[0] = code[i];
		if(Rx2_Buffer[0] == 0) {
			I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
		} else {									
			I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
			I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);					
		}
	}
}

void disp_in_serial(uint8_t* arr) {
	printf("Enter function disp_in_serial!\n");
	for (int i = 0; i < len(arr); i++) {
		printf("%x", arr[i]);
	}
	printf("\n\r");
}
uint8_t cmp(uint8_t* arr1, uint8_t* arr2, uint8_t len){
	printf("Enter function cmp!\n");
	for (int i = 0; i < len; i++)
		if (arr1[i] != arr2[i])
			return 1;
		
	return 0;
}

uint32_t len(uint8_t* arr){
	//printf("Enter function len!\n");
	uint32_t cnt = 0;
	uint32_t i = 0;
	while (arr[i] != 0) {
		cnt++;
		i++;
	}
	//printf("cnt:%d\n", cnt);
	return cnt;
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void swtich_key(void)
{
	switch(Rx1_Buffer[0])
	{
        case 0x1C:
					flag = 1;					
          break;
        case 0x1B:	
					flag = 2;
          break;
        case 0x1A:	
					flag = 3;
          break;
        case 0x14:
					flag = 4;
          break;   
				case 0x13:
					flag = 5;
					break;
        case 0x12:
					flag = 6;
          break;
        case 0x0C:
					flag = 7;
          break;
        case 0x0B:
          flag = 8;
          break;
		case 0x0A:
			flag = 9;
			break;
		case 0x03:
			flag = 15;
			break;
		case 0x19:
			flag = 10;
			break;
		case 0x11:
			flag = 11;
			break;
		case 0x09:
			flag = 12;
			break;
		case 0x01:
			flag = 13;
			break;
		case 0x02:
			flag = 14;
			break;
		case 0x04:
			flag = 16;
        default:
          break;
			}
}

void switch_flag(void){
	switch(flag){
			case 1:
				Tx1_Buffer[0] = 0x0c;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{									
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);					
					}
				break;
			case 2:
				Tx1_Buffer[0] = 0xDA;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);		
					}
				break;
			case 3:
				Tx1_Buffer[0] = 0xF2;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);		
					}
				break;
			case 4:
				Tx1_Buffer[0] = 0x66;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);					
					}
				break;
			case 5:
				Tx1_Buffer[0] = 0xB6;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			case 6:
				Tx1_Buffer[0] = 0xBE;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			case 7:
				Tx1_Buffer[0] = 0xE0;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			case 8:
				Tx1_Buffer[0] = 0xFE;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);					
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);							
					}
				break;
			case 9:
				Tx1_Buffer[0] = 0xE6;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);					
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);					
					}
				break;
			case 10:
				Tx1_Buffer[0] = 0xEE;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);					
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			case 11:
				Tx1_Buffer[0] = 0x3E;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);							
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
					case 12:
				Tx1_Buffer[0] = 0x9C;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);								
					}
				break;
					case 13:
				Tx1_Buffer[0] = 0x7A;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);									
					}
				break;
					case 14:
							Tx1_Buffer[0] = 0x00;
							I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,8);
						break;
					case 15:
				Tx1_Buffer[0] = 0xFC;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			default:
				break;
		}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	flagInterrupt = 1;
}
int fputc(int ch, FILE *f)
{ 
  uint8_t tmp[1]={0};
	tmp[0] = (uint8_t)ch;
	HAL_UART_Transmit(&huart1,tmp,1,10);	
	return ch;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
