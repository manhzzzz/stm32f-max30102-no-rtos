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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdio.h"
#include "string.h"
#include "max30102_for_stm32_hal.h"
#include <math.h>

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

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

max30102_t max30102;
char uartBuf[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void Draw_Waveform(uint32_t ir_val);  // prototype
void max30102_cal(uint32_t red_raw, uint32_t ir_raw);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Retarget printf -> UART
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// ======= Thuật toán tính HR/SpO2 ổn định =======
void max30102_cal(uint32_t red_raw, uint32_t ir_raw)
{
    // 1) Không có ngón tay
    if (ir_raw < 40000) {
        heartRate = 0;
        spo2 = 0;
        return;
    }

    // 2) Trạng thái lọc & peak detection
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

    // 3) DC bằng EMA
    ir_dc  += ALPHA_DC * ((float)ir_raw  - ir_dc);
    red_dc += ALPHA_DC * ((float)red_raw - red_dc);

    // 4) Band-pass kiểu fast - slow
    ir_fast  += ALPHA_FAST * ((float)ir_raw  - ir_fast);
    ir_slow  += ALPHA_SLOW * ((float)ir_raw  - ir_slow);
    ir_bpf    = ir_fast - ir_slow;

    red_fast += ALPHA_FAST * ((float)red_raw - red_fast);
    red_slow += ALPHA_SLOW * ((float)red_raw - red_slow);
    red_bpf   = red_fast - red_slow;

    // 5) Envelope |AC|
    ir_env  += ALPHA_ENV * (fabsf(ir_bpf)  - ir_env);
    red_env += ALPHA_ENV * (fabsf(red_bpf) - red_env);

    // 6) Peak detection với refractory + ngưỡng thích nghi
    if (ref_count > 0) ref_count--;
    samples_since_peak++;

    float thr = fmaxf(MIN_THR_ABS, THR_K_ENV * ir_env);
    if ((prev2 < prev1) && (prev1 > ir_bpf) && (prev1 >= thr) && (ref_count==0)) {
        // Ghi nhận một đỉnh tại prev1
        ref_count = (uint16_t)(FS_HZ * REFRACTORY_SEC);

        uint16_t ibi = samples_since_peak;   // khoảng cách đỉnh-đỉnh (theo mẫu)
        samples_since_peak = 0;

        if (ibi_len < HR_AVG_BEATS) ibi_buf[ibi_len++] = ibi;
        else {
            for (int i = HR_AVG_BEATS-1; i > 0; --i) ibi_buf[i] = ibi_buf[i-1];
            ibi_buf[0] = ibi;
        }

        if (ibi_len >= 2) {
            uint32_t sum = 0;
            for (int i = 0; i < ibi_len; ++i) sum += ibi_buf[i];
            float ibi_avg = (float)sum / (float)ibi_len;
            float hr_inst = 60.0f * (FS_HZ / ibi_avg);

            if (hr_ema <= 1.0f) hr_ema = hr_inst;
            else hr_ema += ALPHA_HR_EMA * (hr_inst - hr_ema);

            if (hr_ema < 30.0f)  hr_ema = 30.0f;
            if (hr_ema > 220.0f) hr_ema = 220.0f;

            heartRate = (uint8_t)(hr_ema + 0.5f);
        }
    }

    prev2 = prev1;
    prev1 = ir_bpf;

    // 7) SpO2 từ R = (ACred/DCred)/(ACir/DCir), AC ~ envelope
    float rdc = fmaxf(1.0f, red_dc);
    float idc = fmaxf(1.0f, ir_dc);
    float rac = fmaxf(1.0f, red_env);
    float iac = fmaxf(1.0f, ir_env);

    float R = ( (rac/rdc) / (iac/idc) );
    float spo2_inst = 110.0f - 25.0f * R;    // tuyến tính điển hình

    if (spo2_ema <= 1.0f) spo2_ema = spo2_inst;
    else                  spo2_ema += ALPHA_SPO2_EMA * (spo2_inst - spo2_ema);

    if (spo2_ema < 70.0f)  spo2_ema = 70.0f;
    if (spo2_ema > 100.0f) spo2_ema = 100.0f;

    spo2 = (uint8_t)(spo2_ema + 0.5f);

    // 8) Cập nhật biến để vẽ waveform (AC chuẩn hoá)
    g_plot_sig = ir_bpf;
    g_plot_env = ir_env;
}

// ====== Vẽ waveform đã chuẩn hoá (giống monitor y tế) ======
void Draw_Waveform(uint32_t ir_val_ignored)
{
    (void)ir_val_ignored;

    float scale = (g_plot_env > 1.0f) ? (3.0f * g_plot_env) : 1000.0f;
    float norm  = g_plot_sig / scale;       // khoảng ~[-0.33..+0.33]
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;

    uint8_t mid = 44 + (WAVEFORM_HEIGHT/2); // vùng [44..63]
    uint8_t y   = (uint8_t)( mid - norm * (WAVEFORM_HEIGHT/2 - 1) );
    waveform_buffer[waveform_x] = y;

    // Lưới mảnh tạo cảm giác như monitor
    for (int x = 0; x < WAVEFORM_WIDTH; x += 8) {
        ssd1306_DrawPixel(x, 44, White);
        ssd1306_DrawPixel(x, 63, White);
    }
    for (int yline = 44; yline <= 63; yline += 4) {
        ssd1306_DrawPixel(0,   yline, White);
        ssd1306_DrawPixel(127, yline, White);
    }

    // Vẽ polyline theo buffer
    for (int i = 1; i < WAVEFORM_WIDTH; i++) {
        int x1 = i - 1;
        int y1 = waveform_buffer[(waveform_x + i - 1) % WAVEFORM_WIDTH];
        int x2 = i;
        int y2 = waveform_buffer[(waveform_x + i) % WAVEFORM_WIDTH];
        ssd1306_Line(x1, y1, x2, y2, White);
    }

    waveform_x = (waveform_x + 1) % WAVEFORM_WIDTH;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();
  SystemClock_Config();        // Đặt clock trước khi init ngoại vi

  /* USER CODE BEGIN Init */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  ssd1306_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_PWR_CLK_ENABLE();
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  }
  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET); // Nếu không dùng, có thể bỏ tránh chặn
  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
  printf("STM32 + MAX30102 Init...\r\n");

  // Khởi tạo MAX30102
  max30102_init(&max30102, &hi2c1);
  max30102_reset(&max30102);

  // Cấu hình chế độ SpO2
  max30102_set_mode(&max30102, max30102_spo2);

  // FIFO: trung bình 4 mẫu, rollover enable, FIFO full ngưỡng 15
  max30102_set_fifo_config(&max30102, max30102_smp_ave_4, 1, 0x0F);

  // LED: IR và RED ~7mA (tinh chỉnh theo thực tế để DC đẹp, không bão hoà)
  max30102_set_led_current_1(&max30102, 7.0f);
  max30102_set_led_current_2(&max30102, 7.0f);

  printf("MAX30102 Ready!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Đọc FIFO (tuỳ lib: nếu có nhiều mẫu đọc được, có thể duyệt hết)
      max30102_read_fifo(&max30102);
      uint32_t ir_val  = max30102._ir_samples[0];   // hoặc mẫu mới nhất tuỳ lib
      uint32_t red_val = max30102._red_samples[0];

      // Tính HR/SpO2 với thuật toán mới
      max30102_cal(red_val, ir_val);

      // Hiển thị
      ssd1306_Fill(Black);

      ssd1306_SetCursor(2, 8);
      ssd1306_WriteString("HR:", Font_7x10, White);
      sprintf(uartBuf, "%3d bpm", heartRate);
      ssd1306_SetCursor(40, 8);
      ssd1306_WriteString(uartBuf, Font_7x10, White);

      ssd1306_SetCursor(2, 24);
      ssd1306_WriteString("SpO2:", Font_7x10, White);
      sprintf(uartBuf, "%3d %%", spo2);
      ssd1306_SetCursor(50, 24);
      ssd1306_WriteString(uartBuf, Font_7x10, White);

      Draw_Waveform(ir_val);
      ssd1306_UpdateScreen();

      // (Tuỳ chọn) In UART thưa hơn để nhẹ CPU
      static uint32_t tick = 0;
      if (++tick >= (uint32_t)(FS_HZ/5)) { // ~5 lần/giây
          tick = 0;
          sprintf(uartBuf, "HR:%3d bpm | SpO2:%3d%%\r\n", heartRate, spo2);
          HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
      }

      HAL_Delay((uint32_t)(1000.0f / FS_HZ)); // 20ms nếu FS_HZ=50
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
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
  // Nếu dùng HAL blocking, KHÔNG cần enable IRQ sự kiện I2C:
  // HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);
  // HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PA0: nếu dùng để wakeup/nhấn, nên cấu hình Pull phù hợp để tránh treo
  GPIO_InitStruct.Pin  = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // hoặc GPIO_PULLDOWN/UP theo mạch
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PB12: ví dụ EXTI nếu cần (chưa dùng callback trong code này)
  GPIO_InitStruct.Pin  = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */
