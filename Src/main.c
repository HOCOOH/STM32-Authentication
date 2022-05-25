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
#include "stdlib.h"
#include "flash.h"

/* USER CODE BEGIN Includes */
#include "zlg7290.h"
#include "stdio.h"
#include <time.h>
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

#define FLASH_ADDR_BASE 0x08080000
#define ALIGN_PASSWD_BACKUP 0x100
#define ADDR_PASSWD_BACKUP(i) (FLASH_ADDR_BASE + ((i) * ALIGN_PASSWD_BACKUP))

/*
#define MAX_USERNAME_LEN 6
#define MIN_USERNAME_LEN 3
#define MAX_PASSWORD_LEN 8
#define MIN_PASSWORD_LEN 5
#define N_WORKER 5
*/

#define N_STATE 6
#define STATE_INIT 0
#define STATE_WAIT 1
#define STATE_INPUT 2
#define STATE_CHECK 3
#define STATE_EDIT 4
#define STATE_EXCEPTION 5

#define EXCEPT_UNVALIAD_STATE 1
#define EXCEPT_TIMEOUT 2
#define EXCEPT_PASSLEN_LONG 3
#define EXCEPT_PASSLEN_SHORT 4

#define MAX_PASSWORD_LEN 12
#define MIN_PASSWORD_LEN 5
#define DIGEST_LEN 32
#define PASSWD_LEN_U32 ((DIGEST_LEN + 4) / 4)

#define REFRESH_COUNT_INIT 500
#define HOT_BOOT_FLAG 0x1234ABCD
#define TIMEOUT_THRESHOLD 5

#define N_BACKUP 3

typedef struct {
    u8 currentState;
    u8 lastState;
    u8 errorCode;
} State;

typedef struct {
    u8 hashVal[DIGEST_LEN + 1];
    u8 CRCVal;
    u8 align[2];
} Passwd;

u32 boot_flag __attribute__((at(0x10000000)));

//u32 refreshCount __attribute__((at(0x10002100))) = 0;
State state __attribute__((at(0x10005000)));
State stateBackup[N_BACKUP + 1]  __attribute__((at(0x10006000))) = {0};
uint8_t flag __attribute__((at(0x10000500)));//不同的按键有不同的标志位值
uint8_t flagInterrupt __attribute__((at(0x10000600))) = 0;//中断标志位，每次按键产生一次中断，并开始读取8个数码管的值
uint8_t edit_mode __attribute__((at(0x10000A00)))  = 0;
uint64_t time_count __attribute__((at(0x10000B00)))  = 0;
uint8_t pass_buf[MAX_PASSWORD_LEN + 1] __attribute__((at(0x10000D00))) = {0};
uint8_t pass_buf_sm3[DIGEST_LEN + 1] __attribute__((at(0x10000E00))) = {0};
uint8_t input_cursor __attribute__((at(0x10001100))) = 0;
u8 hashMark __attribute__((at(0x10001200))) = 0;
Passwd passwdRAM __attribute__((at(0x10000F00))) = {0};

uint8_t Rx2_Buffer[8] = {0};
uint8_t Tx1_Buffer[8] = {0};
uint8_t Rx1_Buffer[1] = {0};

const u8 passwdDefault[MAX_PASSWORD_LEN + 1] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0, 0};
const double delay_choices[] = {5, 10, 15};
const u8 prev_state_table[N_STATE][N_STATE] = {	
									{0, 0, 0, 0, 0, 1}, 
									{1, 0, 1, 1, 1, 0},
									{0, 1, 0, 0, 0, 0},
									{0, 0, 1, 0, 0, 0},
									{0, 0, 1, 0, 0, 0},
									{1, 1, 1, 1, 1, 0} }; // exception待添加

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
void rand_delay();
void UpdatePasswdBackup();
u8 IsPasswdValid();
u8 ResumePasswd(u8* validMask, Passwd* tmpPasswd);
void mymemcpy(void* dest, const void* src, u32 len);
void delay_ms(u32 time);
uint8_t crc4_itu(uint8_t *data, uint16_t length);
void UpdateStateBackup();
void GlobalInit();

void UpdateStateBackup() {
    for (int i = 0; i < N_BACKUP; i++) {
        mymemcpy((void*)(stateBackup + i), (void*)&state, sizeof(State));
    }
}

void GlobalInit() {
    edit_mode = 0;
    hashMark = 0;
    input_cursor = 0;
    time_count = 0;
    flag = 0;//不同的按键有不同的标志位值
    flagInterrupt = 0;//中断标志位，每次按键产生一次中断，并开始读取8个数码管的值
    for (int i = 0; i <= MAX_PASSWORD_LEN; i++) {
        pass_buf[i] = 0;
    }
    for (int i = 0; i <= DIGEST_LEN; i++) {
        pass_buf_sm3[i] = 0;
    }
    passwdRAM = (Passwd){0};

    // reset state
    state.currentState = STATE_INIT;
    state.lastState = STATE_EXCEPTION;
    state.errorCode = 0;
    UpdateStateBackup();
}

int main(void) {
    if (boot_flag != HOT_BOOT_FLAG) {
        // cold boot
		boot_flag = HOT_BOOT_FLAG;
        // init
        GlobalInit();

        /* 冷启动恢复密码 */
        if (IsPasswdValid()) {
            // printf("存储的密码损坏且无法恢复\n\r");
            // while (1) {}
            SM3_Hash(passwdDefault, len(passwdDefault), (void*)passwdRAM.hashVal);
            UpdatePasswdBackup();
        }
    } else {
        // hot boot
        // load the backend vars and check

    }
    
    // printf("\n\r");
    // printf("\n\r===================================\n\r");
    // printf("\n\r FS-STM32身份验证系统\n\r");
    // printf("\n\r test : len(pass_store):%d\n\r", len(pass_store));
    // printf("\n\r test : pass_store:\n\r");
    // disp_in_serial(pass_store);
    
    // SM3_Hash(pass_store, len(pass_store), (void*)pass_store_sm3);
    // printf("SM3哈希后的pass_store:\n\r");
    // disp_in_serial(pass_store_sm3);
    // printf("\n\r");

    // 主循环
    while (1) {
        /* 初始化状态 */
        if (state.currentState == STATE_INIT) {
            /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
            HAL_Init();

            /* Configure the system clock */
            SystemClock_Config();

            /* Initialize all configured peripherals */
            MX_GPIO_Init();
            MX_I2C1_Init();
            MX_USART1_UART_Init();
            //MX_CRC_Init();

            /* Initialize variables */
           // refreshCount = REFRESH_COUNT_INIT;
            input_cursor = 0;
			edit_mode = 0;
            for (int i = 0; i < MAX_PASSWORD_LEN; i++) {
                pass_buf[i] = 0;
            }

            rand_delay();

            // switch to state wait
            UpdateState(STATE_WAIT);
        }
        /* 待机状态 */
        else if (state.currentState == STATE_WAIT) {
            rand_delay();
            if (IsStateValid(STATE_WAIT)) {
                state.errorCode = EXCEPT_UNVALIAD_STATE;
                UpdateState(STATE_EXCEPTION);
                continue;
            }
            input_cursor = 0;
            /* 检测到输入，转到输入状态 */
            if (flagInterrupt == 1) {
                UpdateState(STATE_INPUT);
                continue;
            }
            // set refresh timer 
            //if (refreshCount-- == 0) {
                //UpdateState(STATE_INIT);
                //continue;
            //}
        }
        /* 输入状态 */
        else if (state.currentState == STATE_INPUT) {
            rand_delay();
            if (IsStateValid(STATE_INPUT)) {
                state.errorCode = EXCEPT_UNVALIAD_STATE;
                UpdateState(STATE_EXCEPTION);
                continue;
            }
						
						time_count = 0;
            // get input 
            while (input_cursor < MAX_PASSWORD_LEN) {
							
								if (time_count > 5000000 * TIMEOUT_THRESHOLD) {
									time_count = 0;
									state.errorCode = EXCEPT_TIMEOUT;
                  UpdateState(STATE_EXCEPTION); // return wait state 
									goto expcetion_handler;
								}
								
                if(flagInterrupt == 1) {
                    flagInterrupt = 0;
                    I2C_ZLG7290_Read(&hi2c1,0x71,0x01,Rx1_Buffer,1);	//读键值
                    swtich_key();	//扫描键值，写标志位
                    I2C_ZLG7290_Read(&hi2c1,0x71,0x10,Rx2_Buffer,8);	//读8位数码管
                    switch_flag();	//扫描到相应的按键并且向数码管写进数值
                    printf("flag: %d\n\r", flag);

                    if (flag == 14) {// #
                        printf("Password input done!\n\r");
                        hashMark = 0;
                        break;
                    }
                    else if (flag == 16 && hashMark == 0) { // *
                        printf("Edit password\n\r");
                        hashMark = 1;
                        break;
                    }
										else if (flag == 16 && hashMark == 1) { // *
                        // expception
                    }
                    else {
                        pass_buf[input_cursor++] = flag;
                    }
										time_count = 0;
              // todo: 异常!!!:无效的字?
									}
								time_count++;
							}
            
						pass_buf[input_cursor] = 0;
            // 异常: 密码长度过长
            if (input_cursor >= MAX_PASSWORD_LEN) {
                printf("Password is too long\n\r");
                state.errorCode = EXCEPT_PASSLEN_LONG;
                UpdateState(STATE_EXCEPTION);
                continue;
            }
            // 异常: 密码长度过短
            else if (input_cursor < MIN_PASSWORD_LEN) {
                printf("Password is too short\n\r");
                state.errorCode = EXCEPT_PASSLEN_SHORT;
                UpdateState(STATE_EXCEPTION);
                continue;
            }
            // todo: 重复输入**********************
						IsStateValid(STATE_INPUT);
						printf("editmode:%d\n", edit_mode);
            /* 输入合法 */
            if (edit_mode == 1) {
                UpdateState(STATE_EDIT);
            }
            else {
				UpdateState(STATE_CHECK);
            }
        }
        /* 判断密码状态 */
        else if (state.currentState == STATE_CHECK) {
            rand_delay();
            if (IsStateValid(STATE_CHECK)) {
                state.errorCode = EXCEPT_UNVALIAD_STATE;
                UpdateState(STATE_EXCEPTION);
                continue;
            }
            // printf("\n\rlen: %d  ", input_cursor);
            // printf("pass_buf:\n\r");
            // disp_in_serial(pass_buf);
            /* 对输入的密码进行SM3摘要 */
            SM3_Hash(pass_buf, len(pass_buf), (void*)pass_buf_sm3);
            printf("SM3 hash value of pass_buf: ");
            disp_in_serial(pass_buf_sm3);
            printf("SM3 hash value of pass_store: ");
            disp_in_serial(passwdRAM.hashVal);
            
            /* 比对输入摘要和真实密码摘要 */
            if (IsPasswdValid()) {
                printf("存储的密码损坏且无法恢复\n\r");
                while (1) {}
            }
            int cmp_res = cmp(pass_buf_sm3, passwdRAM.hashVal, input_cursor);
            if (cmp_res == 0) { // 匹配
                // 显示success
                printf("password match!\n\r");
                uint8_t success_code[] = {0xB6, 0xB6, 0x9E, 0x9C, 0x9C, 0x7C, 0xB6};
                disp_str(success_code);
            
            } else { // 不匹配
                // 显示error
                printf("password not match!\n\r");
                // todo: edit code
                uint8_t error_code[] = {0xB6, 0xB6, 0x9E, 0x9C, 0x9C, 0x7C, 0xB6};
                disp_str(error_code);
            }
            
            /* 返回待机状态 */
            edit_mode = hashMark;
            UpdateState(STATE_WAIT);
        }
        /* 修改密码状态 */
        else if (state.currentState == STATE_EDIT) {
            rand_delay();
            if (IsStateValid(STATE_EDIT)) {
                state.errorCode = EXCEPT_UNVALIAD_STATE;
                UpdateState(STATE_EXCEPTION);
                continue;
            }

            // 更新密码
            // for (int i = 0; i < MAX_PASSWORD_LEN; i++) {
            //     if (i < input_cursor)
            //         pass_store[i] = pass_buf[i];
            //     else
            //         pass_store[i] = 0;
            // }
            for (int i = input_cursor; i < MAX_PASSWORD_LEN; i++) {
                pass_buf[i] = 0;
            }
            SM3_Hash(pass_buf, len(pass_buf), (void*)passwdRAM.hashVal);
            UpdatePasswdBackup();

            // 显示update
            uint8_t up_code[] = {0xCE, 0x7C};
            disp_str(up_code);
            printf("Password update to:\n");
            disp_in_serial(pass_buf);
            
            /* 返回待机状态 */
            edit_mode = 0;
            UpdateState(STATE_WAIT);
        }
        /* 异常状态 */
        else {	// default 异常
expcetion_handler:
            rand_delay();
            // =============
            uint8_t error_code[] = {0xB6, 0xB6, 0x9E, 0x9C, 0x9C, 0x7C, 0xB6};
            disp_str(error_code);
            // 需要延时!!!
            switch (state.errorCode) {
                case EXCEPT_UNVALIAD_STATE:
										printf("unvaliad state!\n");
                   // UpdateState(STATE_INIT);
                    break;
                case EXCEPT_TIMEOUT:
										printf("time out!\n");
                    //UpdateState(STATE_WAIT);
                    break;
                case EXCEPT_PASSLEN_LONG:
									printf("pass long!\n");
                   // UpdateState(STATE_WAIT);
                    break;
                case EXCEPT_PASSLEN_SHORT:
									printf("pass short!\n");
                   // UpdateState(STATE_WAIT);
                    break;
                default:
                    //UpdateState(STATE_INIT);
									break;
            }
						UpdateState(STATE_INIT);
        }
    }

}
void delay_ms(u32 time){    
   u32 i=0;  
   while(time--)
   {
      i=60000;
      while(i--) ;    
   }
}

u8 IsStateValid(u8 expectState) {
	if (state.currentState != expectState) {
		return 1;
	}
	// check pre state
	u8 check_pre_res = prev_state_table[state.currentState][state.lastState];
	if (check_pre_res == 0) {
		return 1;
	}
    // check state backup
    for (int i = 0; i < N_BACKUP; i++) {
        if (cmp((u8*)&state, (u8*)(stateBackup + i), sizeof(State))) {
            return 1;
        }
    }

	return 0;	
}

void UpdateState(u8 nextState) {
    // TODO: check if next state is vaild
    state.lastState = state.currentState;
    state.currentState = nextState;

    if (nextState != STATE_EXCEPTION) {
        state.errorCode = 0;
    }

    // 备份state
    UpdatePasswdBackup();
}

uint8_t crc4_itu(uint8_t *data, uint16_t length) {
    uint8_t i;
    uint8_t crc = 0;                // Initial value
    while(length--)
    {
        crc ^= *data++;                 // crc ^= *data; data++;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x0C;// 0x0C = (reverse 0x03)>>(8-4)
            else
                crc = (crc >> 1);
        }
    }
    return crc;
}

void UpdatePasswdBackup() {
    passwdRAM.CRCVal = crc4_itu(passwdRAM.hashVal, DIGEST_LEN);
	// passwdCRC = HAL_CRC_Calculate(&hcrc, (u32*)pass_store_sm3, DIGEST_LEN / 4);
    for (int i = 0; i < N_BACKUP; i++) {
        STMFLASH_Write(ADDR_PASSWD_BACKUP(i), (u32*)&passwdRAM, PASSWD_LEN_U32);
    }
}

u8 IsPasswdValid() {
    u8 flag = 0;
    u8 validMask[N_BACKUP + 1] = {0};
    /* 首先校验密码备份 */
    u32 tmpCRC = 0;
    Passwd tmpPasswd[N_BACKUP + 1] = {0};
    for (int i = 0; i < N_BACKUP; i++) {
        STMFLASH_Read(ADDR_PASSWD_BACKUP(i), (u32*)(tmpPasswd + i), PASSWD_LEN_U32);
		tmpCRC = crc4_itu(tmpPasswd[i].hashVal, DIGEST_LEN);;
        if (tmpCRC == tmpPasswd[i].CRCVal) {
            validMask[i] = 1;
        }
    }
	tmpCRC = crc4_itu(passwdRAM.hashVal, DIGEST_LEN);;
    /* 检查密码哈希 */
    if (tmpCRC != passwdRAM.CRCVal) {
        flag = 1;
    }
    else {
        validMask[N_BACKUP] = 1;
        tmpPasswd[N_BACKUP].CRCVal = tmpCRC;
        mymemcpy((void*)tmpPasswd[N_BACKUP].hashVal, (void*)passwdRAM.hashVal, DIGEST_LEN);
        for (int i = 0; i < N_BACKUP; i++) {
            if (validMask[i] && tmpPasswd[i].CRCVal != passwdRAM.CRCVal) {
                flag = 1;
            }
        }
    }

    /* 若密码失效，尝试恢复 */
    if (flag) {
        flag = ResumePasswd(validMask, tmpPasswd);
    }

    return flag;
}

u8 ResumePasswd(u8* validMask, Passwd* tmpPasswd) {
    u8 validCnt[N_BACKUP + 1] = {0};
    for (int i = 0; i <= N_BACKUP; i++) {
        for (int j = 0; j <= N_BACKUP; j++) {
            if (validMask[i] && validMask[j] && tmpPasswd[i].CRCVal == tmpPasswd[j].CRCVal) {
                validCnt[i]++;
            }
        }
    }
    
    /* 找到匹配数最多的密码 */
    u8 maxVal = 0, maxInex = 0;
    for (int i = 0; i <= N_BACKUP; i++) {
        if (maxVal < validCnt[i]) {
            maxVal = validCnt[i];
            maxInex = i;
        }
    }
    /* 没有两个相同的密码备份，炸了 */
    if (maxVal < 2) {
        return 1;
    }
    /* 恢复密码 */
    mymemcpy((void*)passwdRAM.hashVal, (void*)tmpPasswd[maxInex].hashVal, DIGEST_LEN);
    UpdatePasswdBackup();
    return 0;
}

void disp_str(uint8_t* code) {
    // printf("Enter function disp_str!\n\r");
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

void mymemcpy(void* dest, const void* src, u32 len) {
    u8* destp = (u8*)dest;
    u8* srcp = (u8*)src;
    for (u32 i = 0; i < len; i++) {
        *destp++ = *srcp++;
    }
}

void disp_in_serial(uint8_t* arr) {
    // printf("Enter function disp_in_serial!\n\r");
    for (int i = 0; i < len(arr); i++) {
        printf("%x", arr[i]);
    }
    printf("\n\r\n\r");
}
uint8_t cmp(uint8_t* arr1, uint8_t* arr2, uint8_t len){
    printf("Enter function cmp!\n\r");
    for (int i = 0; i < len; i++)
        if (arr1[i] != arr2[i])
            return 1;
        // todo: 侧信道攻击
    return 0;
}

uint32_t len(uint8_t* arr){
    //printf("Enter function len!\n\r");
    uint32_t cnt = 0;
    uint32_t i = 0;
    while (arr[i] != 0) {
        cnt++;
        i++;
    }
    //printf("cnt:%d\n\r", cnt);
    return cnt;
    /*
    asm("nop");
    asm("nop");
    asm("jmp 0x8040000");
    */
}

void rand_delay() {
		unsigned int seed = 12345;
    srand(seed);
    u8  choice = rand() % 3;
    double nms = delay_choices[choice];
		delay_ms(nms);
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
    ex: printf("Wrong parameters value: file %s on line %d\n\r", file, line) */
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
