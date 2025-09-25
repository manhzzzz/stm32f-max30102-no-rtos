/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdio.h"
#include "string.h"
#include "max30102_for_stm32_hal.h"
#include <math.h>
#include "ds18b20.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ====== Tham số thuật toán ======
#define FS_HZ              50.0f     // Tốc độ mẫu thuật toán (50Hz mặc định)
#define DT                 (1.0f/FS_HZ)

#define ALPHA_DC           0.01f     // EMA cho DC (~1s)
#define ALPHA_FAST         0.25f     // nhanh (~5Hz)
#define ALPHA_SLOW         0.02f     // chậm (~0.5Hz)
#define ALPHA_ENV          0.10f     // EMA cho envelope |AC|
#define ALPHA_HR_EMA       0.20f     // làm mượt HR
#define ALPHA_SPO2_EMA     0.15f     // làm mượt SpO2

#define REFRACTORY_SEC     0.35f     // 300ms chống đếm trùng
#define MIN_THR_ABS        8.0f      // ngưỡng tối thiểu
#define THR_K_ENV          0.5f      // ngưỡng thích nghi = K*envelope

#define HR_AVG_BEATS       6         // số chu kỳ để lấy trung bình HR

#define WAVEFORM_WIDTH     128
#define WAVEFORM_HEIGHT    20
// ====== Layout OLED (không đè nhau) ======
#define UI_HEADER_H   40              // vùng header: y = 0..39
#define WAVE_Y0       UI_HEADER_H     // đồ thị bắt đầu từ y=40
#define WAVE_H        (64 - UI_HEADER_H)

// 3 cột ngang trong header
#define COL_W   42
#define COL0_X  2
#define COL1_X  (COL0_X + COL_W)   // 44
#define COL2_X  (COL1_X + COL_W)   // 86

// Font dùng cho label và giá trị
#define FONT_LABEL    Font_6x8
#define FONT_VALUE    Font_7x10

// ====== LM75 (I2C) ======
#define LM75_DEFAULT_ADDR  0x48
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for ds18b20Queue */
osMessageQueueId_t ds18b20QueueHandle;
const osMessageQueueAttr_t ds18b20Queue_attributes = {
  .name = "ds18b20Queue"
};
/* Definitions for I2Cmutex */
osMutexId_t I2CmutexHandle;
const osMutexAttr_t I2Cmutex_attributes = {
  .name = "I2Cmutex"
};
/* Definitions for UartMutex */
osMutexId_t UartMutexHandle;
const osMutexAttr_t UartMutex_attributes = {
  .name = "UartMutex"
};
/* USER CODE BEGIN PV */
// Kết quả hiển thị
uint8_t heartRate = 0;
uint8_t spo2 = 0;
// Buffer vẽ waveform
static uint8_t  waveform_x = 0;
static uint8_t  waveform_buffer[WAVEFORM_WIDTH];
// Biến để vẽ (được cập nhật trong max30102_cal)
volatile float g_plot_sig = 0.0f;
volatile float g_plot_env = 1.0f;
//biencuamax30102
max30102_t max30102;
char uartBuf[64];
uint32_t ir_val = 0, red_val = 0;
//biencuabody
uint8_t lm75_addr = LM75_DEFAULT_ADDR;
float t_body = 0.0f;
static float t_body_ema = 0.0f;
//biencuaoutdoor
float t_outdoor = NAN;       // Nhiệt độ ngoài trời
volatile uint8_t t_outdoor_valid = 0; // Cờ báo dữ liệu ngoài trời đã hợp lệ
osMutexId_t i2cMutex;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "I2Cmutex"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void Startmaxtask(void *argument);
void Startds18b20task(void *argument);
void Startlm75task(void *argument);
void Startoledtask(void *argument);
void Startuarttask(void *argument);

/* USER CODE BEGIN PFP */
float LM75_ReadTemp_C(I2C_HandleTypeDef *hi2c, uint8_t addr7);
void Draw_Waveform(uint32_t ir_val);  // prototype
void max30102_cal(uint32_t red_raw, uint32_t ir_raw);
void DWT_Delay_Init(void);
void delay_us(uint32_t us);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Retarget printf -> UART
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
void max30102_cal(uint32_t red_raw, uint32_t ir_raw)
{
    if (ir_raw < 40000) { heartRate = 0; spo2 = 0; return; }

    static float ir_dc=0, red_dc=0;
    static float ir_fast=0, ir_slow=0, red_fast=0, red_slow=0;
    static float ir_bpf=0, red_bpf=0;
    static float ir_env=1, red_env=1;
    static float prev2=0, prev1=0;
    static uint16_t ref_count = 0;
    static uint16_t ibi_buf[HR_AVG_BEATS] = {0};
    static uint8_t  ibi_len = 0;
    static uint16_t samples_since_peak = 0;
    static float hr_ema = 0.0f;
    static float spo2_ema = 0.0f;

    ir_dc  += ALPHA_DC * ((float)ir_raw  - ir_dc);
    red_dc += ALPHA_DC * ((float)red_raw - red_dc);

    ir_fast  += ALPHA_FAST * ((float)ir_raw  - ir_fast);
    ir_slow  += ALPHA_SLOW * ((float)ir_raw  - ir_slow);
    ir_bpf    = ir_fast - ir_slow;

    red_fast += ALPHA_FAST * ((float)red_raw - red_fast);
    red_slow += ALPHA_SLOW * ((float)red_raw - red_slow);
    red_bpf   = red_fast - red_slow;

    ir_env  += ALPHA_ENV * (fabsf(ir_bpf)  - ir_env);
    red_env += ALPHA_ENV * (fabsf(red_bpf) - red_env);

    if (ref_count > 0) ref_count--;
    samples_since_peak++;

    float thr = fmaxf(MIN_THR_ABS, THR_K_ENV * ir_env);
    if ((prev2 < prev1) && (prev1 > ir_bpf) && (prev1 >= thr) && (ref_count==0)) {
        ref_count = (uint16_t)(FS_HZ * REFRACTORY_SEC);
        uint16_t ibi = samples_since_peak;
        samples_since_peak = 0;

        if (ibi_len < HR_AVG_BEATS) ibi_buf[ibi_len++] = ibi;
        else { for (int i = HR_AVG_BEATS-1; i > 0; --i) ibi_buf[i] = ibi_buf[i-1]; ibi_buf[0] = ibi; }

        if (ibi_len >= 2) {
            uint32_t sum = 0; for (int i = 0; i < ibi_len; ++i) sum += ibi_buf[i];
            float ibi_avg = (float)sum / (float)ibi_len;
            float hr_inst = 60.0f * (FS_HZ / ibi_avg);
            if (hr_ema <= 1.0f) hr_ema = hr_inst; else hr_ema += ALPHA_HR_EMA * (hr_inst - hr_ema);
            if (hr_ema < 30.0f)  hr_ema = 30.0f;
            if (hr_ema > 220.0f) hr_ema = 220.0f;
            heartRate = (uint8_t)(hr_ema + 0.5f);
        }
    }

    prev2 = prev1; prev1 = ir_bpf;

    float rdc = fmaxf(1.0f, red_dc), idc = fmaxf(1.0f, ir_dc);
    float rac = fmaxf(1.0f, red_env), iac = fmaxf(1.0f, ir_env);
    float R = ( (rac/rdc) / (iac/idc) );
    float spo2_inst = 110.0f - 25.0f * R;

    if (spo2_ema <= 1.0f) spo2_ema = spo2_inst; else spo2_ema += ALPHA_SPO2_EMA * (spo2_inst - spo2_ema);
    if (spo2_ema < 70.0f)  spo2_ema = 70.0f;
    if (spo2_ema > 100.0f) spo2_ema = 100.0f;
    spo2 = (uint8_t)(spo2_ema + 0.5f);

    g_plot_sig = ir_bpf;
    g_plot_env = ir_env;
}
// ====== Vẽ waveform ======
void Draw_Waveform(uint32_t ir_val_ignored)
{
    (void)ir_val_ignored;
    float scale = (g_plot_env > 1.0f) ? (3.0f * g_plot_env) : 1000.0f;
    float norm  = g_plot_sig / scale; if (norm > 1.0f) norm = 1.0f; if (norm < -1.0f) norm = -1.0f;

    uint8_t y0=WAVE_Y0, yh=WAVE_H, mid=y0+(yh/2);
    uint8_t y=(uint8_t)( mid - norm*((yh/2)-1) );
    if (y < y0) y = y0;
    if (y > (y0+yh-1)) y = y0+yh-1;
    waveform_buffer[waveform_x] = y;

    for (int x = 0; x < 128; x += 8) { ssd1306_DrawPixel(x, y0, White); ssd1306_DrawPixel(x, y0+yh-1, White); }
    for (int yy=y0; yy<=y0+yh-1; yy+=4) { ssd1306_DrawPixel(0, yy, White); ssd1306_DrawPixel(127, yy, White); }

    for (int i=1;i<WAVEFORM_WIDTH;i++){
        int x1=i-1, y1=waveform_buffer[(waveform_x+i-1)%WAVEFORM_WIDTH];
        int x2=i,   y2=waveform_buffer[(waveform_x+i)%WAVEFORM_WIDTH];
        ssd1306_Line(x1,y1,x2,y2,White);
    }
    waveform_x = (waveform_x + 1) % WAVEFORM_WIDTH;
}
// ====== LM75: đọc nhiệt độ (°C) ======
  float LM75_ReadTemp_C(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2c, (addr7<<1), 0x00, I2C_MEMADD_SIZE_8BIT, buf, 2, 100) != HAL_OK) {
        return NAN;
    }
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    raw >>= 7;                 // 9-bit sign
    return (float)raw * 0.5f;  // 0.5°C/LSB
}

void DWT_Delay_Init(void){ CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT=0;
DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk; }
void delay_us(uint32_t us){
uint32_t startTick=DWT->CYCCNT;
uint32_t ticks=us*(HAL_RCC_GetHCLKFreq()/1000000);
while((DWT->CYCCNT-startTick)<ticks); }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  max30102_init(&max30102, &hi2c1);
        max30102_reset(&max30102);
        max30102_set_mode(&max30102, max30102_spo2);
        max30102_set_fifo_config(&max30102, max30102_smp_ave_4, 1, 0x0F);
        max30102_set_led_current_1(&max30102, 7.0f);
        max30102_set_led_current_2(&max30102, 7.0f);
  DWT_Delay_Init();
  DS18B20_Init();
        printf("Sensors Ready!\r\n");
     /* Hien Thi Oled */
  ssd1306_Init();
  	  ssd1306_Fill(Black);
  	  ssd1306_UpdateScreen();
  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of I2Cmutex */
  I2CmutexHandle = osMutexNew(&I2Cmutex_attributes);
  /* creation of UartMutex */
  UartMutexHandle = osMutexNew(&UartMutex_attributes);
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* Create the queue(s) */
  /* creation of ds18b20Queue */
  ds18b20QueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &ds18b20Queue_attributes);
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(Startmaxtask, NULL, &defaultTask_attributes);
  /* creation of myTask02 */
  myTask02Handle = osThreadNew(Startds18b20task, NULL, &myTask02_attributes);
  /* creation of myTask03 */
  myTask03Handle = osThreadNew(Startlm75task, NULL, &myTask03_attributes);
  /* creation of myTask04 */
  myTask04Handle = osThreadNew(Startoledtask, NULL, &myTask04_attributes);
  /* creation of myTask05 */
  myTask05Handle = osThreadNew(Startuarttask, NULL, &myTask05_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Startmaxtask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Startmaxtask */
void Startmaxtask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(I2CmutexHandle, osWaitForever);     // khóa I2C
		        max30102_read_fifo(&max30102);
		        osMutexRelease(I2CmutexHandle);                   // nhả I2C
		  	          ir_val  = max30102._ir_samples[0];
		  	          red_val = max30102._red_samples[0];
		  	          max30102_cal(red_val, ir_val);
	    osDelay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Startds18b20task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startds18b20task */
void Startds18b20task(void *argument)
{
  /* USER CODE BEGIN Startds18b20task */
  /* Infinite loop */
  for(;;)
  {
	  t_outdoor = DS18B20_ReadTemp();
	 	osDelay(120);
  }
  /* USER CODE END Startds18b20task */
}

/* USER CODE BEGIN Header_Startlm75task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startlm75task */
void Startlm75task(void *argument)
{
  /* USER CODE BEGIN Startlm75task */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(I2CmutexHandle, osWaitForever);
	 	 	          float t_body_raw = LM75_ReadTemp_C(&hi2c1, lm75_addr);
	 	 	         osMutexRelease(I2CmutexHandle);
	 	 	          if (!isnan(t_body_raw)) {
	 	 	              t_body_ema += 0.3f * (t_body_raw - t_body_ema);   // alpha = 0.2
	 	 	              t_body = t_body_ema;  // dùng biến này để hiển thị
	 	 	          }
	     osDelay(20);
  }
  /* USER CODE END Startlm75task */
}

/* USER CODE BEGIN Header_Startoledtask */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startoledtask */
void Startoledtask(void *argument)
{
  /* USER CODE BEGIN Startoledtask */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(I2CmutexHandle, osWaitForever);
	  	  	  ssd1306_Fill(Black);

	  	  	  	          // --- HR ---
	  	  	  	          ssd1306_SetCursor(COL0_X, 0);
	  	  	  	          ssd1306_WriteString("HR", FONT_LABEL, White);
	  	  	  	          snprintf(uartBuf, sizeof(uartBuf), "%3d", heartRate);
	  	  	  	          ssd1306_SetCursor(COL0_X, 12);
	  	  	  	          ssd1306_WriteString(uartBuf, FONT_VALUE, White);

	  	  	  	          // --- SpO2 ---
	  	  	  	          ssd1306_SetCursor(COL1_X, 0);
	  	  	  	          ssd1306_WriteString("SpO2", FONT_LABEL, White);
	  	  	  	          snprintf(uartBuf, sizeof(uartBuf), "%2d%%", spo2);
	  	  	  	          ssd1306_SetCursor(COL1_X, 12);
	  	  	  	          ssd1306_WriteString(uartBuf, FONT_VALUE, White);

	  	  	  	          // --- BODY (LM75) ---
	  	  	  	          ssd1306_SetCursor(COL2_X, 0);
	  	  	  	          ssd1306_WriteString("BODY", FONT_LABEL, White);
	  	  	  	          snprintf(uartBuf, sizeof(uartBuf), isnan(t_body) ? "--.-C" : "%.1fC", t_body);
	  	  	  	          ssd1306_SetCursor(COL2_X, 12);
	  	  	  	          ssd1306_WriteString(uartBuf, FONT_VALUE, White);

	  	  	  	          // --- OUT (DS18B20) ---
	  	  	  	          ssd1306_SetCursor(2, 30);
	  	  	  	          ssd1306_WriteString("OUT:", FONT_LABEL, White);
	  	  	  	          snprintf(uartBuf, sizeof(uartBuf), isnan(t_outdoor) ? "--.-C" : "%.1fC", t_outdoor);
	  	  	  	          ssd1306_SetCursor(30, 30);
	  	  	  	          ssd1306_WriteString(uartBuf, FONT_LABEL, White);

	  	  	  	          // --- Waveform ---
	  	  	  	          Draw_Waveform(ir_val);

	  	  	  	          // update màn hình
	  	  	  	          ssd1306_UpdateScreen();
	  	  	  	        osMutexRelease(I2CmutexHandle);
	  	      osDelay(200);
  }
  /* USER CODE END Startoledtask */
}

/* USER CODE BEGIN Header_Startuarttask */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startuarttask */
void Startuarttask(void *argument)
{
  /* USER CODE BEGIN Startuarttask */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(UartMutexHandle, osWaitForever);
	 	  snprintf(uartBuf, sizeof(uartBuf),
	 		  	                   "HR:%3d bpm | SpO2:%3d%% | OUT:%.1fC | BODY:%.1fC\r\n",
	 		  	                   heartRate, spo2,
	 		  	                   isnan(t_outdoor)?0.0f:t_outdoor,
	 		  	                   isnan(t_body)?0.0f:t_body);

	 		  	          HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
	 		  	          osMutexRelease(UartMutexHandle);
	 		  	          osDelay(200);
  }
  /* USER CODE END Startuarttask */
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
