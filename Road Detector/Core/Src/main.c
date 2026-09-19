/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "road_detector.h"
#include "bluetooth_link.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR             0x68U

#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_WHO_AM_I     0x75U

#define MPU6050_ACCEL_LSB_PER_G  8192.0f  /* ±4 g */
#define MPU6050_GYRO_LSB_PER_DPS 65.5f    /* ±500 °/s */
/* Stream diagnostico su ST-LINK; gli eventi e lo stato strada passano via BLE. */
#ifndef DEBUG_STREAM
#define DEBUG_STREAM 0  /* 1 = stream diagnostico su ST-LINK, 0 = solo BLE */
#endif

#define DEBUG_STREAM_HZ        100U
#define DEBUG_STREAM_PERIOD_MS (1000U / DEBUG_STREAM_HZ)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t mpu_raw[14];

static int32_t ax_zero = 0;
static int32_t ay_zero = 0;
static int32_t az_zero = 0;
static int32_t gx_zero = 0;
static int32_t gy_zero = 0;
static int32_t gz_zero = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef MPU6050_Init(void);
static HAL_StatusTypeDef MPU6050_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                                         int16_t *gx, int16_t *gy, int16_t *gz);
static HAL_StatusTypeDef MPU6050_Calibrate(void);
static HAL_StatusTypeDef MPU6050_Recover(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

static HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t who_am_i = 0;

  HAL_Delay(100);

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, MPU6050_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (who_am_i != MPU6050_ADDR)
  {
    return HAL_ERROR;
  }

  if (MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK)
    return HAL_ERROR;  /* Wake-up e PLL */

  if (MPU6050_WriteRegister(MPU6050_REG_CONFIG, 0x03) != HAL_OK)
    return HAL_ERROR;  /* DLPF circa 44 Hz */

  if (MPU6050_WriteRegister(MPU6050_REG_SMPLRT_DIV, 0x09) != HAL_OK)
    return HAL_ERROR;  /* 100 Hz: 1 kHz / (1 + 9), con DLPF attivo */

  if (MPU6050_WriteRegister(MPU6050_REG_GYRO_CONFIG, 0x08) != HAL_OK)
    return HAL_ERROR;  /* ±500 °/s */

  if (MPU6050_WriteRegister(MPU6050_REG_ACCEL_CONFIG, 0x08) != HAL_OK)
    return HAL_ERROR;  /* ±4 g */

  return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                                         int16_t *gx, int16_t *gy, int16_t *gz)
{
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, MPU6050_REG_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, mpu_raw, sizeof(mpu_raw), 100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *ax = (int16_t)((mpu_raw[0] << 8) | mpu_raw[1]);
  *ay = (int16_t)((mpu_raw[2] << 8) | mpu_raw[3]);
  *az = (int16_t)((mpu_raw[4] << 8) | mpu_raw[5]);

  *gx = (int16_t)((mpu_raw[8] << 8) | mpu_raw[9]);
  *gy = (int16_t)((mpu_raw[10] << 8) | mpu_raw[11]);
  *gz = (int16_t)((mpu_raw[12] << 8) | mpu_raw[13]);

  return HAL_OK;
}

/* Tenere immobile il sensore durante questa funzione. */
static HAL_StatusTypeDef MPU6050_Calibrate(void)
{
  const uint16_t samples = 200;
  int16_t ax, ay, az, gx, gy, gz;
  int64_t ax_sum = 0;
  int64_t ay_sum = 0;
  int64_t az_sum = 0;
  int64_t gx_sum = 0;
  int64_t gy_sum = 0;
  int64_t gz_sum = 0;

  for (uint16_t i = 0; i < samples; i++)
  {
    if (MPU6050_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz) != HAL_OK)
    {
      return HAL_ERROR;
    }

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    HAL_Delay(APP_SAMPLE_PERIOD_MS);
  }

  ax_zero = (int32_t)(ax_sum / samples);
  ay_zero = (int32_t)(ay_sum / samples);
  az_zero = (int32_t)(az_sum / samples);
  gx_zero = (int32_t)(gx_sum / samples);
  gy_zero = (int32_t)(gy_sum / samples);
  gz_zero = (int32_t)(gz_sum / samples);

  return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_Recover(void)
{
  HAL_I2C_DeInit(&hi2c1);
  HAL_Delay(5);

  /* Sblocca un bus I2C rimasto appeso prima di reinizializzare la periferica. */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  for (int i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(5);

  MX_I2C1_Init();
  return MPU6050_Init();
}

/* USER CODE END 0 */

int main(void)
{
  uint32_t last_sample_ms = 0;
#if DEBUG_STREAM
  uint32_t last_debug_ms = 0;
#endif
  uint8_t consecutive_i2c_errors = 0;
  uint32_t total_i2c_errors = 0;

  int16_t ax = 0, ay = 0, az = 0;
  int16_t gx = 0, gy = 0, gz = 0;

  char message[160];
#if DEBUG_STREAM
  int length;
#endif

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  if (MPU6050_Init() != HAL_OK && MPU6050_Recover() != HAL_OK)
  {
    const char error[] = "MPU-6050 non trovata: controlla alimentazione e I2C\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)error, sizeof(error) - 1, 100);
    Error_Handler();
  }

  {
    const char calibration[] = "Calibrazione: non muovere il sensore...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)calibration,
                      sizeof(calibration) - 1, 100);
  }

  if (MPU6050_Calibrate() != HAL_OK)
  {
    const char error[] = "Errore durante la calibrazione MPU-6050\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)error, sizeof(error) - 1, 100);
    Error_Handler();
  }

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  BluetoothLink_Init(&huart1);
  (void)BluetoothLink_SendText("HELLO,Sistema_Completo,IMU+BLE_READY\n");

  RoadDetector_Init();
  (void)BluetoothLink_SendText("ROAD_DETECTOR,READY\n");

  {
    const char ready[] = "MPU-6050 pronta: acquisizione 100 Hz\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)ready, sizeof(ready) - 1, 100);
  }
  /* USER CODE END 2 */

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* Acquisizione IMU a 100 Hz */
    if ((now - last_sample_ms) >= APP_SAMPLE_PERIOD_MS)
    {
      int32_t acc_x_mg;
      int32_t acc_y_mg;
      int32_t acc_z_mg;
      int16_t gx_dps;
      int16_t gy_dps;
      int16_t gz_dps;

      last_sample_ms += APP_SAMPLE_PERIOD_MS;

      if (MPU6050_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz) == HAL_OK)
      {
        consecutive_i2c_errors = 0;

        /* IMU ruotata di 90 gradi antiorario attorno a Z.
         * Frame veicolo: X avanti, Y sinistra, Z alto; Xv = -Ys, Yv = Xs. */
        acc_x_mg = -(int32_t)((ay - ay_zero) * 1000.0f / MPU6050_ACCEL_LSB_PER_G);
        acc_y_mg =  (int32_t)((ax - ax_zero) * 1000.0f / MPU6050_ACCEL_LSB_PER_G);
        acc_z_mg = (int32_t)((az - az_zero) * 1000.0f / MPU6050_ACCEL_LSB_PER_G);
        gx_dps = (int16_t)(-((float)(gy - gy_zero) / MPU6050_GYRO_LSB_PER_DPS));
        gy_dps = (int16_t)( ((float)(gx - gx_zero) / MPU6050_GYRO_LSB_PER_DPS));
        gz_dps = (int16_t)((float)(gz - gz_zero) / MPU6050_GYRO_LSB_PER_DPS);

        RoadDetector_Update(now, (int16_t)acc_x_mg, (int16_t)acc_y_mg,
                            (int16_t)acc_z_mg, gx_dps, gy_dps, gz_dps);

#if DEBUG_STREAM
        if ((now - last_debug_ms) >= DEBUG_STREAM_PERIOD_MS)
        {
          last_debug_ms = now;
          length = snprintf(message, sizeof(message),
              "$IMU,%lu,%ld,%ld,%ld,%d,%d,%d,%ld,%ld,%d\r\n",
              (unsigned long)now,
              (long)acc_x_mg,
              (long)acc_y_mg,
              (long)acc_z_mg,
              gx_dps, gy_dps, gz_dps,
              (long)RoadDetector_GetCurrentThreshold(),
              (long)RoadDetector_GetSigmaNoise(),
              (int)RoadDetector_GetState());

          if ((length > 0) && ((size_t)length < sizeof(message)))
          {
            (void)HAL_UART_Transmit(&huart2, (uint8_t *)message,
                                    (uint16_t)length, 20);
          }
        }
#endif

        road_event_t event;
        if (RoadDetector_GetPendingEvent(&event))
        {
          const char *class_str = "UNKNOWN";

          switch (event.event_class)
          {
            case EVENT_POTHOLE_LEFT:     class_str = "POTHOLE_L";  break;
            case EVENT_POTHOLE_RIGHT:    class_str = "POTHOLE_R";  break;
            case EVENT_POTHOLE_CENTER:   class_str = "POTHOLE_C";  break;
            case EVENT_BUMP:             class_str = "BUMP";       break;
            case EVENT_DIP:              class_str = "DIP";        break;
            case EVENT_ROUGH:            class_str = "ROUGH";      break;
            case EVENT_LATERAL_TILT:     class_str = "LAT_TILT";   break;
            case EVENT_ROUGH_TRANSITION: class_str = "ROUGH_TR";   break;
            case EVENT_ANOMALY:          class_str = "ANOMALY";    break;
            default:                     class_str = "UNKNOWN";    break;
          }

          (void)snprintf(message, sizeof(message),
              "$EVENT,%lu,%s,SEV:%u,CONF:%u,PK:%ld,GX:%d,GY:%d,DUR:%u,JERK:%ld\n",
              (unsigned long)event.timestamp_ms,
              class_str,
              event.severity,
              event.confidence,
              (long)event.peak_z_mg,
              event.peak_gx_dps,
              event.peak_gy_dps,
              event.duration_ms,
              (long)event.jerk_max_mg_s);
          (void)BluetoothLink_SendText(message);
        }

      }
      else
      {
        consecutive_i2c_errors++;
        total_i2c_errors++;

        if (consecutive_i2c_errors >= 3U)
        {
          MPU6050_Recover();
          consecutive_i2c_errors = 0;
        }
      }
    }

    /* Lo stato continua a essere trasmesso anche se il sensore I2C non risponde. */
    road_status_t status;
    if (RoadDetector_Get1HzStatus(now, &status))
    {
      (void)snprintf(message, sizeof(message),
          "$STATUS,%lu,NOISE:%ld,N10S:%ld,N30S:%ld,SCORE:%u,EVTS:%lu,TH:%ld,ST:%d,I2CERR:%lu,I2CSTREAK:%u\n",
          (unsigned long)status.uptime_ms,
          (long)status.sigma_noise_mg,
          (long)status.sigma_noise_10s_mg,
          (long)status.sigma_noise_30s_mg,
          status.roughness_score,
          (unsigned long)status.total_events_count,
          (long)status.current_threshold_mg,
          (int)status.current_state,
          (unsigned long)total_i2c_errors,
          (unsigned int)consecutive_i2c_errors);
      (void)BluetoothLink_SendText(message);
    }

    /* Dump del macro-storico su richiesta dal terminale ST-LINK (D, d o ?). */
    uint8_t rx_cmd = 0;
    if (HAL_UART_Receive(&huart2, &rx_cmd, 1, 0) == HAL_OK)
    {
      if ((rx_cmd == 'D') || (rx_cmd == 'd') || (rx_cmd == '?'))
      {
        uint8_t count = RoadDetector_GetMacroCount();
        const char dump_start[] = "$MACRO_DUMP_START\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)dump_start, sizeof(dump_start) - 1, 50);

        for (uint8_t i = 0; i < count; i++)
        {
          if (RoadDetector_FormatMacroDumpItem(i, message, sizeof(message)))
          {
            HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 50);
          }
        }

        const char dump_end[] = "$MACRO_DUMP_END\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)dump_end, sizeof(dump_end) - 1, 50);
      }
    }

  }
}

/**
  * @brief USART1 Initialization Function
  * @note  PA9 = TX (verso RXD BLE), PA10 = RX (dal TXD BLE).
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
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
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;  /* più robusto per il primo test */
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
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
