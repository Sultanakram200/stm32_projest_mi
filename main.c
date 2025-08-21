/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (organized, sections, and centered OLED output)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Main program body for Frequency/Temperature/PWM functionality
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h> // إضافة مكتبة الرياضيات للـ log

/* USER CODE BEGIN 0 */
/* --- Typedefs --- */
typedef enum {
    MODE_FREQUENCY,
    MODE_TEMPERATURE,
    MODE_PWM
} FunctionMode;

typedef enum {
    PWM_SET_DUTY,
    PWM_SET_FREQ
} PWMSetMode;

/* --- Defines --- */
#define PWM_FREQ_MIN   1
#define PWM_FREQ_MAX   10000
#define PWM_DUTY_MIN   0
#define PWM_DUTY_MAX   100

#define BUTTON_REPEAT_START 400
#define BUTTON_REPEAT_SPEED 80
#define FREQ_FILTER_SIZE   10
#define FREQ_CORRECTION_FACTOR (50.0f/458.96f)

/* --- Global Variables --- */
FunctionMode currentMode = MODE_FREQUENCY;
PWMSetMode pwmSetMode = PWM_SET_DUTY;
uint32_t pwmFreq = 10;
uint32_t pwmDuty = 50;
/* --- تعريفات الحرارة NTC --- */
#define B_PARAM 3950 //معامل B (انظر ورقة البيانات)
#define MY_RESISTOR 4630 // المقاومة 4.7 كيلو أوم
#define THERMISTOR_R 4700 // المقاومة الاسمية للثرمستور 4.7 كيلو أوم (عند درجة حرارة 25 درجة)
#define NOMINAL_T 25 //درجة الحرارة الاسمية
extern ADC_HandleTypeDef hadc1;
void MX_ADC1_Init(void);
float stein_hart = 0;
//------------------------
// مؤقت تحديث العرض للحرارة
//------------------------
#define TEMP_UPDATE_PERIOD_MS  500
volatile uint32_t last_temp_update_ms = 0;
//------------------------
// متغيرات قياس التردد
//------------------------
typedef struct {
    volatile uint32_t icValue1;
    volatile uint32_t icValue2;
    volatile uint32_t diffCapture;
    volatile uint8_t  isFirstCaptured;
    volatile uint32_t measuredFrequency;
    volatile uint8_t freq_ready;
    volatile float measuredFrequencyFloat;
    float filter_buffer[FREQ_FILTER_SIZE];
    uint8_t filter_index;
    uint8_t filter_count;
    float filtered;
} FrequencyMeasurement;

FrequencyMeasurement freq = {0};

//------------------------
// مؤقت تحديث العرض
//------------------------
#define FREQ_UPDATE_PERIOD_US  300
volatile uint32_t last_freq_update_us = 0;

//------------------------
// ماكروز أزرار التحكم
//------------------------
#define READ_INCR_BTN()   (HAL_GPIO_ReadPin(SW_INCR_GPIO_Port, SW_INCR_Pin) == GPIO_PIN_RESET)
#define READ_DECR_BTN()   (HAL_GPIO_ReadPin(SW_DECR_GPIO_Port, SW_DECR_Pin) == GPIO_PIN_RESET)
#define READ_SET_BTN()    (HAL_GPIO_ReadPin(SET_PWM_GPIO_Port, SET_PWM_Pin) == GPIO_PIN_RESET)
#define READ_FREQ_BTN()   (HAL_GPIO_ReadPin(Frequency_Mode_GPIO_Port, Frequency_Mode_Pin) == GPIO_PIN_RESET)
#define READ_TEMP_BTN()   (HAL_GPIO_ReadPin(Temperature_Mode_GPIO_Port, Temperature_Mode_Pin) == GPIO_PIN_RESET)
#define READ_PWM_BTN()    (HAL_GPIO_ReadPin(PWM_Mode_GPIO_Port, PWM_Mode_Pin) == GPIO_PIN_RESET)
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
#define TEMP_FILTER_SIZE 10
float temp_buffer[TEMP_FILTER_SIZE] = {0};
uint8_t temp_index = 0;
uint8_t temp_count = 0;

void Read_Temperature(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint16_t adc = HAL_ADC_GetValue(&hadc1);
    float tr = 4095.0 / adc - 1;
    tr = MY_RESISTOR / tr;
    float temp = tr / THERMISTOR_R;           // (R/Ro)
    temp = log(temp);                         // ln(R/Ro)
    temp /= B_PARAM;                          // 1/B_PARAM * ln(R/Ro)
    temp += 1.0 / (NOMINAL_T + 273.15);       // + (1/To)
    temp = 1.0 / temp;                        // Invert
    temp -= 273.15;
    temp = temp * 1.08;

    // إضافة القراءة إلى الفلتر
    temp_buffer[temp_index] = temp;
    temp_index = (temp_index + 1) % TEMP_FILTER_SIZE;
    if (temp_count < TEMP_FILTER_SIZE) temp_count++;
}

float Get_Temperature_Average(void)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < temp_count; i++)
        sum += temp_buffer[i];
    return (temp_count > 0) ? (sum / temp_count) : 0.0f;
}

void Temperature_PeriodicUpdate(void)
{
    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_temp_update_ms) < TEMP_UPDATE_PERIOD_MS) return;
    last_temp_update_ms = now_ms;

    Read_Temperature();

    float temp_avg = Get_Temperature_Average();
    char temp_msg[32];
    sprintf(temp_msg, "NTC:%.2fC", temp_avg);

    SSD1306_Clear();
    SSD1306_GotoXY(0, 22);
    SSD1306_Puts(temp_msg, &Font_11x18, 1);
    SSD1306_UpdateScreen();
}
/* ------------ OLED DISPLAY FUNCTIONS ------------- */
void oled_print_pwm_param_value(void) {
    char label[8];
    char value[24];
    SSD1306_Clear();

    if (pwmSetMode == PWM_SET_DUTY) {
        strcpy(label, "DUTY");
        sprintf(value, "%u%%", pwmDuty);
    } else {
        strcpy(label, "FREQ");
        sprintf(value, "%uHz", pwmFreq);
    }

    SSD1306_GotoXY(32, 0);
    SSD1306_Puts(label, &Font_16x26, 1);
    SSD1306_GotoXY(0, 34);
    SSD1306_Puts(value, &Font_11x18, 1);

    SSD1306_UpdateScreen();
}

void oled_print_pwm_param_switch(void) {
    SSD1306_Clear();
    if (pwmSetMode == PWM_SET_DUTY) {
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("DUTY", &Font_16x26, 1);
    } else {
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("FREQ", &Font_16x26, 1);
    }
    SSD1306_UpdateScreen();
}

void oled_centered_print(const char *line1, const char *line2, const char *line3)
{
    SSD1306_Clear();
    if (line1) {
        SSD1306_GotoXY(42, 0);
        SSD1306_Puts((char*)line1, &Font_11x18, 1);
    }
    if (line2) {
        SSD1306_GotoXY(25, 22);
        SSD1306_Puts((char*)line2, &Font_16x26, 1);
    }
    if (line3) {
        SSD1306_GotoXY(42, 50);
        SSD1306_Puts((char*)line3, &Font_11x18, 1);
    }
    SSD1306_UpdateScreen();
}

void oled_print_mode(const char* txt)
{
    oled_centered_print("", txt, "");
}

void oled_print_startup(void)
{
    SSD1306_Clear();
    SSD1306_GotoXY(8, 24);
    SSD1306_Puts("SE_V0.0", &Font_16x26, 1);
    SSD1306_UpdateScreen();
    HAL_Delay(1000);
}
/* USER CODE END 1 */

/* ----------------------------------------------------------- */
/* USER CODE BEGIN FREQ_FUNCTIONS */
/*
======================== قياس التردد ========================
*/
void Frequency_Init(void) {
    freq.icValue1 = 0;
    freq.icValue2 = 0;
    freq.diffCapture = 0;
    freq.isFirstCaptured = 0;
    freq.measuredFrequency = 0;
    freq.measuredFrequencyFloat = 0.0f;
    freq.freq_ready = 0;
    freq.filter_index = 0;
    freq.filter_count = 0;
    freq.filtered = 0.0f;
    memset(freq.filter_buffer, 0, sizeof(freq.filter_buffer));
    last_freq_update_us = 0;
}

void Frequency_Start(void) {
    Frequency_Init();
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    // تشغيل تايمر 4 بسرعة 1MHz لتحديث العرض كل 300us
    HAL_TIM_Base_Start(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

void Frequency_Stop(void) {
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim4);
}

void Frequency_FilterAdd(float raw_freq) {
    float corrected_freq = raw_freq * FREQ_CORRECTION_FACTOR;
    freq.filter_buffer[freq.filter_index] = corrected_freq;
    freq.filter_index = (freq.filter_index + 1) % FREQ_FILTER_SIZE;
    if (freq.filter_count < FREQ_FILTER_SIZE)
        freq.filter_count++;
    float sum = 0.0f;
    for (uint8_t i = 0; i < freq.filter_count; i++) {
        sum += freq.filter_buffer[i];
    }
    freq.filtered = sum / freq.filter_count;
}

// تحديث القراءة وإظهار النتائج كل 300 ميكرو ثانية باستخدام TIM4
void Frequency_PeriodicUpdate(void) {
    uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim4);
    if ((now_us - last_freq_update_us) < FREQ_UPDATE_PERIOD_US) return;
    last_freq_update_us = now_us;

    char msg[32];
    float display_freq = freq.filtered;

    // الشرط الأول: إذا كان التردد أكبر من 50 وأصغر من 180
    if (display_freq > 45.0f && display_freq < 180.0f) {
        HAL_GPIO_WritePin(GPIOA, compressor_Led_Pin , GPIO_PIN_SET);       
        HAL_GPIO_WritePin(GPIOA, Erorr_Led_Pin , GPIO_PIN_RESET);
        strcpy(msg, "ok signal");
    }
    // الشرط الثاني: إذا كان التردد أصغر من 45 أو أكبر من 200
    else if (display_freq < 40.0f || display_freq > 200.0f) {
        HAL_GPIO_WritePin(GPIOA, compressor_Led_Pin , GPIO_PIN_RESET);       
        HAL_GPIO_WritePin(GPIOA, Erorr_Led_Pin , GPIO_PIN_SET);
        strcpy(msg, "no signal");
    }
    // الحالة الافتراضية
    else {
        HAL_GPIO_WritePin(GPIOA, compressor_Led_Pin , GPIO_PIN_RESET);       
        HAL_GPIO_WritePin(GPIOA, Erorr_Led_Pin , GPIO_PIN_RESET);
        strcpy(msg, "no signal");
    }

    SSD1306_Clear();
    SSD1306_GotoXY(8, 22);
    SSD1306_Puts(msg, &Font_11x18, 1);

    char freq_msg[20];
    if (display_freq < 999999)
        sprintf(freq_msg, "%.2f Hz", display_freq);
    else
        strcpy(freq_msg, "OVFL");
    SSD1306_GotoXY(8, 40);
    SSD1306_Puts(freq_msg, &Font_7x10, 1);

    SSD1306_UpdateScreen();
}

/*
  كول باك لمقاطعة Input Capture: يستقبل النبضات ويحسب الفرق ويحدث الفلتر
*/
void Frequency_IC_Callback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 && currentMode == MODE_FREQUENCY) {
        if (freq.isFirstCaptured == 0) {
            freq.icValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            freq.isFirstCaptured = 1;
        } else if (freq.isFirstCaptured == 1) {
            freq.icValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            if (freq.icValue2 > freq.icValue1)
                freq.diffCapture = freq.icValue2 - freq.icValue1;
            else
                freq.diffCapture = (0xFFFF - freq.icValue1) + freq.icValue2 + 1;
            // حساب قيمة التردد الخام من فرق النبضات
            if (freq.diffCapture != 0) {
                freq.measuredFrequency = 1000000UL / freq.diffCapture;
                freq.measuredFrequencyFloat = 1000000.0f / (float)(freq.diffCapture);
            } else {
                freq.measuredFrequency = 0;
                freq.measuredFrequencyFloat = 0.0f;
            }

            Frequency_FilterAdd(freq.measuredFrequencyFloat);
            if (freq.filter_count >= FREQ_FILTER_SIZE) {
                freq.freq_ready = 1;
            }
            freq.isFirstCaptured = 0;
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    Frequency_IC_Callback(htim);
}
/* USER CODE END FREQ_FUNCTIONS */
/* ----------------------------------------------------------- */

/* USER CODE BEGIN PWM_FUNCTION */
void PWM_StartAndUpdate(void);
void PWM_UpdateOnly(void);

void PWM_StartAndUpdate(void) {
    PWM_UpdateOnly();
    oled_print_pwm_param_value();
}
void PWM_UpdateOnly(void) {
    uint32_t timer_clk = 72000000;
    uint32_t prescaler = 0;
    uint32_t period = 0;

    period = timer_clk / pwmFreq;
    if (period > 65535) {
        prescaler = period / 65536;
        period = (timer_clk / (prescaler + 1)) / pwmFreq;
    }
    if (period < 1) period = 1;
    __HAL_TIM_SET_PRESCALER(&htim3, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1);

    uint32_t pulse = (period * pwmDuty) / 100;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);

    oled_print_pwm_param_value();
}
/* USER CODE END PWM_FUNCTION */

/* USER CODE BEGIN BUTTON_FUNCTION */
void show_current_mode(void);

void BUTTONS_HandleAll(void)
{
    static bool freq_btn_prev = false, temp_btn_prev = false, pwm_btn_prev = false, set_btn_prev = false;
    static bool incr_btn_prev = false, decr_btn_prev = false;
    static uint32_t incr_press_time = 0, decr_press_time = 0;
    static uint32_t incr_last_action = 0, decr_last_action = 0;
    static bool pwm_started = false;
    static bool freq_measuring = false;

    bool freq_now = READ_FREQ_BTN();
    if (freq_now && !freq_btn_prev) {
        currentMode = MODE_FREQUENCY;
        show_current_mode();
        Frequency_Start();
        freq_measuring = true;
    }
    freq_btn_prev = freq_now;

    // قياس التردد: لا توقف القياس، فقط اعرض باستمرار كل 300us
    if (currentMode == MODE_FREQUENCY && freq_measuring) {
        Frequency_PeriodicUpdate();
    }

    bool temp_now = READ_TEMP_BTN();
    if (temp_now && !temp_btn_prev) {
        currentMode = MODE_TEMPERATURE;
        show_current_mode();
        // لا حاجة لتشغيل أي مؤقتات خاصة هنا
    }
    temp_btn_prev = temp_now;

    // تحديث عرض درجة الحرارة عند تفعيل وضع الحرارة
    if (currentMode == MODE_TEMPERATURE) {
        Temperature_PeriodicUpdate();
    }

    bool pwm_now = READ_PWM_BTN();
    if (pwm_now && !pwm_btn_prev) {
        currentMode = MODE_PWM;
        show_current_mode();
        if (!pwm_started) {
            PWM_StartAndUpdate();
            pwm_started = true;
        }
    }
    pwm_btn_prev = pwm_now;

    bool set_now = READ_SET_BTN();
    if (set_now && !set_btn_prev && currentMode == MODE_PWM) {
        pwmSetMode = (pwmSetMode == PWM_SET_DUTY) ? PWM_SET_FREQ : PWM_SET_DUTY;
        oled_print_pwm_param_value();
    }
    set_btn_prev = set_now;

    bool incr_now = READ_INCR_BTN();
    if (currentMode == MODE_PWM) {
        uint32_t now = HAL_GetTick();
        if (incr_now && !incr_btn_prev) {
            incr_press_time = now;
            incr_last_action = now;
            if (pwmSetMode == PWM_SET_DUTY && pwmDuty < PWM_DUTY_MAX) {
                pwmDuty++;
                PWM_UpdateOnly();
            } else if (pwmSetMode == PWM_SET_FREQ && pwmFreq < PWM_FREQ_MAX) {
                pwmFreq += 10;
                PWM_UpdateOnly();
            }
        } else if (incr_now) {
            if (now - incr_press_time > BUTTON_REPEAT_START &&
                now - incr_last_action > BUTTON_REPEAT_SPEED) {
                incr_last_action = now;
                if (pwmSetMode == PWM_SET_DUTY && pwmDuty < PWM_DUTY_MAX) {
                    pwmDuty++;
                    PWM_UpdateOnly();
                } else if (pwmSetMode == PWM_SET_FREQ && pwmFreq < PWM_FREQ_MAX) {
                    pwmFreq += 10;
                    PWM_UpdateOnly();
                }
            }
        }
    }
    incr_btn_prev = incr_now;

    bool decr_now = READ_DECR_BTN();
    if (currentMode == MODE_PWM) {
        uint32_t now = HAL_GetTick();
        if (decr_now && !decr_btn_prev) {
            decr_press_time = now;
            decr_last_action = now;
            if (pwmSetMode == PWM_SET_DUTY && pwmDuty > PWM_DUTY_MIN) {
                pwmDuty--;
                PWM_UpdateOnly();
            } else if (pwmSetMode == PWM_SET_FREQ && pwmFreq > PWM_FREQ_MIN + 10) {
                pwmFreq -= 10;
                PWM_UpdateOnly();
            }
        } else if (decr_now) {
            if (now - decr_press_time > BUTTON_REPEAT_START &&
                now - decr_last_action > BUTTON_REPEAT_SPEED) {
                decr_last_action = now;
                if (pwmSetMode == PWM_SET_DUTY && pwmDuty > PWM_DUTY_MIN) {
                    pwmDuty--;
                    PWM_UpdateOnly();
                } else if (pwmSetMode == PWM_SET_FREQ && pwmFreq > PWM_FREQ_MIN + 10) {
                    pwmFreq -= 10;
                    PWM_UpdateOnly();
                }
            }
        }
    }
    decr_btn_prev = decr_now;
}
/* USER CODE END BUTTON_FUNCTION */

/* ========== SHOW CURRENT MODE FUNCTION ========== */
void show_current_mode(void)
{
    switch (currentMode) {
        case MODE_FREQUENCY:
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
            oled_print_mode("KART");
            break;
        case MODE_TEMPERATURE:
            oled_print_mode("NTC");
            break;
        case MODE_PWM:
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
            oled_print_mode("MOTOR");
            break;
        default:
            oled_print_mode("UNKNOWN");
            break;
    }
}

int main(void)
{
    HAL_Init();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    SSD1306_Init();

    oled_print_startup();

    while (1)
    {
        BUTTONS_HandleAll();
        HAL_Delay(1); // قلل التأخير لزيادة سرعة التحديث
    }
}
//==============================================================================================================
//==============================================================================================================
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
