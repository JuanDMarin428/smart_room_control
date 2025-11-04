/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "comms.h"
#include "uart_parser.h"
#include <stdio.h>
#include "kalman3.h"
#include "kalman1d.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

volatile meas_packet_t g_last_meas;
volatile meas_packet_t g_filt_meas;
volatile uint32_t      g_meas_seq = 0;
volatile uint32_t      g_last_meas_tick = 0;
volatile uint32_t      g_last_meas_age_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static comms_t comms;
static kf3_t  kf;
static kf1d_t kfN;
static uint32_t last_kf_tick = 0;

static const float V      = 50.0f;      // m^3
static const float rho    = 1.2f;       // kg/m^3
static const float cp     = 1005.0f;    // J/kg/°C
static const float Ph     = 1500.0f;    // W
static const float eta_h  = 0.95f;      // -
static const float q_max  = 0.056f;     // m^3/s
static const float k_stack= 0.002f;     // m^3/(s·°C)
static const float Q_person = 75.0f;    // W/person
static const float G_w      = 1.1e-5f;  // kg/s/person
static const float gamma_c  = 0.1f;     // ppm/s/person

static const float heater_scale = 5.0f; //Depends on python test
static const float fan_scale = 20.0f;	//Depends on python test

float q_flow(float uf, float T, float To){
    return (q_max * fan_scale) * clampf(uf, 0, 1) + k_stack * (T - To);
}

static volatile bool g_send_kf_frames = true;
static inline void set_send_kf_frames(bool enable) { g_send_kf_frames = enable; }



/* ---- Control setpoints (can be updated later via UART if desired) ------- */
static volatile float T_ref = 21.0f;     /* °C */
static volatile float w_ref = 0.0060f;   /* kg/kg absolute humidity */
static volatile float c_ref = 800.0f;    /* ppm CO2 */

/* ---- Controller gains (starting points; tune on your rig) --------------- */
/* Temperature → heater (uh) */
static float Kp_T  = 0.80f;
static float Ki_T  = 0.05f;    /* [1/s] */
static float K_aw_T = 0.5f;    /* anti-windup back-calculation */

/* Ventilation → fan (uf), combining humidity & CO2 */
static float Kw_w  = 0.7f;     /* weight for humidity error */
static float Kw_c  = 0.3f;     /* weight for CO2 error */
static float Kp_F  = 2.0f;
static float Ki_F  = 0.10f;    /* [1/s] */
static float K_aw_F = 0.5f;    /* anti-windup back-calculation */

/* Output limits and simple per-step rate limits */
static const float UH_MIN = 0.0f, UH_MAX = 1.0f;
static const float UF_MIN = 0.0f, UF_MAX = 1.0f;
static const float UH_RATE = 0.10f;      /* max Δuh per control step */
static const float UF_RATE = 0.20f;      /* max Δuf per control step */

/* ---- PI controller state ------------------------------------------------ */
typedef struct {
    float uh, uf;   /* current outputs */
    float I_T;      /* integrator for temperature loop */
    float I_F;      /* integrator for ventilation loop */
} ctrl_state_t;

static ctrl_state_t g_ctrl = { .uh = 0.5f, .uf = 0.25f, .I_T = 0.0f, .I_F = 0.0f };

/**
 * @brief Combine humidity and CO2 into a single ventilation error.
 *        Rough normalization prevents unit dominance.
 *
 * Sign convention:
 *  - If w_hat < w_ref → positive e_w → tends to *reduce* ventilation.
 *  - If c_hat > c_ref → positive e_c → tends to *increase* ventilation.
 */
static inline float ventilation_error(float w_hat, float c_hat) {
    float e_w = (w_ref - w_hat) / 0.0020f;  /* ~2 g/kg typical indoor span */
    float e_c = (c_hat - c_ref) / 400.0f;   /* ~±400 ppm typical span */
    return (Kw_w * e_w) + (Kw_c * e_c);
}

/**
 * @brief One control step: PI(uh) for temperature and PI(uf) for ventilation.
 * @param dt      Control step (seconds)
 * @param T_hat   Estimated temperature
 * @param w_hat   Estimated absolute humidity
 * @param c_hat   Estimated CO2
 * @param out     Output control packet (uh, uf saturated & rate-limited)
 */
static void control_step(float dt, float T_hat, float w_hat, float c_hat, ctrl_packet_t *out) {
    /* --- Temperature loop → uh (PI + anti-windup) --- */
    float eT = (T_ref - T_hat);
    float uh_unsat = Kp_T * eT + g_ctrl.I_T;
    float uh_cmd   = clampf(uh_unsat, UH_MIN, UH_MAX);
    /* back-calculation anti-windup */
    g_ctrl.I_T += (Ki_T * eT + K_aw_T * (uh_cmd - uh_unsat)) * dt;

    /* Rate limit */
    float d_uh = uh_cmd - g_ctrl.uh;
    if (d_uh >  UH_RATE) d_uh =  UH_RATE;
    if (d_uh < -UH_RATE) d_uh = -UH_RATE;
    g_ctrl.uh = clampf(g_ctrl.uh + d_uh, UH_MIN, UH_MAX);

    /* --- Ventilation loop → uf (PI on combined error) --- */
    float eF = ventilation_error(w_hat, c_hat);
    float uf_unsat = Kp_F * eF + g_ctrl.I_F;
    float uf_cmd   = clampf(uf_unsat, UF_MIN, UF_MAX);
    g_ctrl.I_F += (Ki_F * eF + K_aw_F * (uf_cmd - uf_unsat)) * dt;

    /* Rate limit */
    float d_uf = uf_cmd - g_ctrl.uf;
    if (d_uf >  UF_RATE) d_uf =  UF_RATE;
    if (d_uf < -UF_RATE) d_uf = -UF_RATE;
    g_ctrl.uf = clampf(g_ctrl.uf + d_uf, UF_MIN, UF_MAX);

    /* Return control packet */
    out->uh = g_ctrl.uh;
    out->uf = g_ctrl.uf;
}


/**
 * @brief Measurement callback: linearize model, run 3-state KF, publish filtered values,
 *        and optionally transmit a machine-parsable KF frame "<KF,T,w,c,N>\r\n".
 *
 * Steps:
 *  1) Compute dt since last callback.
 *  2) Build inputs and disturbances (N filtered via 1D KF).
 *  3) Linearize f(x,u,d) around current estimate x = [T, w, c] to form discrete A,B,E.
 *  4) Kalman predict/update with z = [T, w, c] measured.
 *  5) Publish filtered values. Optionally TX "<KF,...>" if g_send_kf_frames = true.
 *
 * UART output:
 *  - Human-readable log: "MEAS ... | KF ..."
 *  - Optional machine-parsable: "<KF,T,w,c,N>\r\n"
 */
static void on_meas_cb(const meas_packet_t *m) {
    g_last_meas = *m;

    /* 1) Time step [s] */
    uint32_t now = HAL_GetTick();
    float dt = (now - last_kf_tick) * 0.001f;
    if (dt < 1e-3f) dt = 1e-3f;
    if (dt > 60.0f) dt = 60.0f;
    last_kf_tick = now;

    /* 2) Inputs and disturbances (replace with live signals if available) */
    float u[2] = { 0.5f, 0.25f };             /* [u_h, u_f] */
    float To = 10.0f, wo = 0.004f, co = 420.0f;

    /* N filtering (current measurement as observation) */
    float Nhat = kf1d_update(&kfN, m->N);
    float d[4] = { To, wo, co, Nhat };

    /* 3) Linearization around current estimate x = [T, w, c] */
    const float T = kf.x[0], w = kf.x[1], c = kf.x[2];
    float q = q_flow(u[1], T, To);
    float alpha = q / V;
    float beta  = k_stack / V;

    /* dfdx */
    float dfdx[3][3] = {
        { -alpha + beta*(To - T),  0.0f,                   0.0f },
        {  beta*(wo - w),         -alpha,                 0.0f },
        {  beta*(co - c),          0.0f,                 -alpha }
    };

    /* dfdu, u=[uh, uf] */
    float dfdu[3][2] = {0};
    dfdu[0][0] = (eta_h * Ph * heater_scale) / (rho * cp * V);
    float kv = (q_max)/V;   /* dq/duf scaled by volume */
    dfdu[0][1] = kv*(To - T);
    dfdu[1][1] = kv*(wo - w);
    dfdu[2][1] = kv*(co - c);

    /* dfdd, d=[To, wo, co, N] */
    float dfdd[3][4] = {0};
    dfdd[0][0] =  alpha - beta*(To - T);
    dfdd[1][0] = -beta*(wo - w);
    dfdd[2][0] = -beta*(co - c);
    dfdd[1][1] =  alpha;
    dfdd[2][2] =  alpha;
    dfdd[0][3] = (Q_person)/(rho*cp*V);
    dfdd[1][3] = (G_w)/(rho*V);
    dfdd[2][3] = gamma_c;

    /* Discrete model: A = I + dt*dfdx ; B = dt*dfdu ; E = dt*dfdd */
    float A[3][3] = {
        {1.0f + dt*dfdx[0][0], dt*dfdx[0][1],            dt*dfdx[0][2]},
        {dt*dfdx[1][0],        1.0f + dt*dfdx[1][1],     dt*dfdx[1][2]},
        {dt*dfdx[2][0],        dt*dfdx[2][1],            1.0f + dt*dfdx[2][2]}
    };
    float B[3][2] = {
        {dt*dfdu[0][0], dt*dfdu[0][1]},
        {dt*dfdu[1][0], dt*dfdu[1][1]},
        {dt*dfdu[2][0], dt*dfdu[2][1]}
    };
    float E[3][4] = {
        {dt*dfdd[0][0], dt*dfdd[0][1], dt*dfdd[0][2], dt*dfdd[0][3]},
        {dt*dfdd[1][0], dt*dfdd[1][1], dt*dfdd[1][2], dt*dfdd[1][3]},
        {dt*dfdd[2][0], dt*dfdd[2][1], dt*dfdd[2][2], dt*dfdd[2][3]}
    };

    /* 4) Kalman filter (z = measured states) */
    float z[3] = { m->T, m->w, m->c };
    if (!kf.inited) {
        float x0[3]    = { z[0], z[1], z[2] };
        float Rdiag[3] = { 0.01f, 1e-8f, 400.0f };
        float Qdiag[3] = { 4e-4f, 1e-8f, 25.0f };
        kf3_init(&kf, x0, 1.0f, Qdiag, Rdiag);
    } else {
        kf3_predict(&kf, A, B, u, E, d);
        kf3_update(&kf, z);
    }

    /* 5) Publish filtered values */
    g_filt_meas.T = kf.x[0];
    g_filt_meas.w = kf.x[1];
    g_filt_meas.c = kf.x[2];
    g_filt_meas.N = Nhat;

    g_last_meas_tick = now;
    g_meas_seq++;

    /* Human-readable log (kept) */
    {
        char msg[192];
        int n = snprintf(msg, sizeof(msg),
            "MEAS T=%.3f w=%.6f c=%.1f N=%.0f | KF T=%.3f w=%.6f c=%.1f N=%.2f\r\n",
            m->T, m->w, m->c, m->N,
            g_filt_meas.T, g_filt_meas.w, g_filt_meas.c, g_filt_meas.N);
        if (n > 0) {
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)n, HAL_MAX_DELAY);
        }
    }

    /* Optional machine-parsable KF frame: "<KF,T,w,c,N>\r\n" */
    if (g_send_kf_frames) {
        char kf_frame[96];
        int k = snprintf(kf_frame, sizeof(kf_frame),
                         "<KF,%.3f,%.6f,%.1f,%.2f>\r\n",
                         g_filt_meas.T, g_filt_meas.w, g_filt_meas.c, g_filt_meas.N);
        if (k > 0) {
            HAL_UART_Transmit(&huart3, (uint8_t*)kf_frame, (uint16_t)k, HAL_MAX_DELAY);
        }
    }
}



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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // const char *hello = "STM32F767ZI UART3 via ST-LINK VCP (PD8/PD9) OK\r\n";
  // HAL_UART_Transmit(&huart3, (uint8_t*)hello, strlen(hello), HAL_MAX_DELAY);
  comms_init(&comms, &huart3, on_meas_cb);

  float x0[3] = {20.0f, 0.006f, 800.0f};
  float Rdiag[3] = { 0.10f*0.10f, 1e-4f*1e-4f, 20.0f*20.0f }; // [T,w,c]
  float Qdiag[3] = { 4e-4f, 1e-8f, 25.0f };
  kf3_init(&kf, x0, 1.0f, Qdiag, Rdiag);
  kf1d_init(&kfN, 2.0f, 1.0f, 1e-6f, 1e-2f); // N casi constante, ruido chico
  last_kf_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    comms_uart_rx_callback(&comms, huart);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  ctrl_packet_t ctrl = { .uh = 0.5f, .uf = 0.25f };

  const uint32_t Ts_ms = 100;                 /* 10 Hz control */
  uint32_t last_ctrl = osKernelGetTickCount();

  for (;;) {
      comms_poll(&comms);

      uint32_t now = osKernelGetTickCount();
      g_last_meas_age_ms = now - g_last_meas_tick;

      /* Periodic control step */
      if ((now - last_ctrl) >= Ts_ms) {
          float dt = (now - last_ctrl) * 0.001f;
          if (dt < 1e-3f) dt = 1e-3f;
          last_ctrl = now;

          /* Use KF estimates (latest available) */
          float T_hat = g_filt_meas.T;
          float w_hat = g_filt_meas.w;
          float c_hat = g_filt_meas.c;

          control_step(dt, T_hat, w_hat, c_hat, &ctrl);

          /* Send control every step */
          comms_send_ctrl(&comms, &ctrl);

          /* Optional quick log:
          // char buf[96];
          // int n = snprintf(buf, sizeof(buf), "CTRL uh=%.3f uf=%.3f\r\n", ctrl.uh, ctrl.uf);
          // if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
          */
      }

      osDelay(10);
  }
  /* USER CODE END 5 */
}

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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
