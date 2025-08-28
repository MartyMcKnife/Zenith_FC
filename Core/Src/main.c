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
#include "app_fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bq27441.h"
#include "lsm6xx.h"
#include "ms5607.h"
#include "w25qxx.h"

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
FLIGHT_STATES flight_state = PRE_LAUNCH;

// controls number of pulses for our led
static uint8_t led_count = 0;
static uint8_t led_total = 0;
static bool enable_led = false;

// store flight data in array, and flush it to a csv after flight - reduce
// read/writes on flash take 1 sample every 100ms - array has 1800 rows,
// corresponding 3 minutes of flight time
// if we run out of space flush to csv and start again

// data stored:
// | pressure | altitude | temperature | raw_accel_x | raw_accel_y | raw_accel_z
// | raw_gyro_x | raw_gyro_y | raw_gyro_z | voltage | current | soc |
static float init_data[12] = {0};
static float flight_data[1800][12] = {0};
uint16_t sample_point = 0;
bool take_sample = false;
uint8_t flight_no = 0;

float avg_height;
float sq_values;
float height_sd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
static int16_t BQ27441_i2cWriteBytes(uint8_t DevAddress, uint8_t subAddress,
                                     uint8_t *src, uint8_t count);
static int16_t BQ27441_i2cReadBytes(uint8_t DevAddress, uint8_t subAddress,
                                    uint8_t *dest, uint8_t count);

static void blink_short(uint8_t count);
static void blink_long(uint8_t count);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  W25Qxxx_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_USB_Device_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // if we've connected a USB, don't worry about anything

  if (is_usb_connected()) {
    flight_state = DEBUG;
  }
  // tiny delay to let all the drivers initialize
  HAL_Delay(50);

  // intialize I/O
  if (MS5607_Init(&hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin) == MS5607_FAIL) {
    flight_state = INIT_FAILURE;
    blink_long(2);
  }

  if (LSM6XX_Init(&hi2c2) == LSM6XX_FAIL) {
    flight_state = INIT_FAILURE;
    blink_long(3);

  } else {
    LSM6XX_set_accel_config(LSM_ACCEL_32G, LSM_ACCEL_208HZ);
    LSM6XX_set_gyro_config(LSM_GYRO_1000, LSM_GYRO_208HZ);

    LSM6XX_CAL cal_set = {.xl_hw_x = 20,
                          .xl_hw_y = -50,
                          .xl_hw_z = 44,
                          .g_sw_x = 25,
                          .g_sw_y = -10,
                          .g_sw_z = -30};

    LSM6XX_calibrate(&cal_set);

    // TODO: Make sure triggers correctly for ignition
    LSM6XX_set_ff(INT1, LSM_FF_500, 3);
  }

  BQ27441_ctx_t BQ27441 = {
      .BQ27441_i2c_address = BQ72441_I2C_ADDRESS,
      .read_reg = BQ27441_i2cReadBytes,
      .write_reg = BQ27441_i2cWriteBytes,
  };

  // configure BQ27441 for our 1100mAh battery

  if (BQ27441_init(&BQ27441) != true) {
    flight_state = INIT_FAILURE;
    blink_long(4);

  } else {
    BQ27441_setCapacity(BATTERY_CAPACITY);
    BQ27441_setDesignEnergy(BATTERY_CAPACITY * 3.7);
    BQ27441_setTerminateVoltageMin(MIN_VOLTAGE);
    BQ27441_setTaperRateVoltage(MAX_VOLTAGE);
  }

  // bq27441 needs to learn battery usage, give a sec
  HAL_Delay(100);

  // if battery not charged enough, flash warning
  // not enough to trigger fail mode but we want to let user know
  if (BQ27441_soc(FILTERED) < 70) {
    blink_short(2);
  }

  // mount fs
  FATFS fs;
  FIL fp;
  FRESULT f_res;

  DIR dj;
  FILINFO fno;
  char fp_name[13];

  // try to mount fs, blink if we can't
  f_res = f_mount(&fs, "0:", 1);

  if (f_res != FR_OK) {
    flight_state = INIT_FAILURE;
    blink_long(5);

  } else {
    // intialize flight number
    // calculate flight number - traverse root directory until all filenames
    // read
    f_res = f_findfirst(&dj, &fno, "0:", "FL_??.csv");

    // if we can't find anything, intialize flight_no to 0
    if (!fno.fname[0]) {
      flight_no = 0;
    } else {
      while (f_res == FR_OK && fno.fname[0]) {
        // digits are represented in ascii codes - subtract 48 to convert them
        flight_no = ((fno.fname[3] - 48) * 10 + (fno.fname[4] - 48)) + 1;
        f_res = f_findnext(&dj, &fno);
      }
    }

    f_closedir(&dj);
  }

  // get initial readings

  init_samples(init_data);

  // init success
  if (flight_state == PRE_LAUNCH) {
    blink_short(2);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (flight_state) {
      // something is wrong - just blink LED
      // blocking blink is fine
    case INIT_FAILURE:
      HAL_GPIO_TogglePin(STS_LED_GPIO_Port, STS_LED_Pin);
      HAL_Delay(500);
      break;

      // don't do anything in prelaunch - just waiting for IMU to say we are
      // flying
    case PRE_LAUNCH:
      __NOP();
      break;

    // don't do anything when we are reading data
    // could accidentally trigger flight
    case DIAG:
      __NOP();
      break;

      // handle sampling
    case LAUNCH:
      if (take_sample) {
        // turn on LED whilst we are sampling
        HAL_GPIO_WritePin(STS_LED_GPIO_Port, STS_LED_Pin, GPIO_PIN_SET);
        collect_samples(flight_data[sample_point], (uint32_t)init_data[0]);
        HAL_GPIO_WritePin(STS_LED_GPIO_Port, STS_LED_Pin, GPIO_PIN_RESET);
        take_sample = false;

        // check to see if we have landed
        // landed is determined when current height sample is within 1 s.d. of
        // average height of last 5 samples
        avg_height = 0;
        height_sd = 0;
        if (sample_point >= 6) {
          // calculate average
          // get last 5 samples
          for (uint8_t i = 0; i < 5; i++) {

            avg_height += (int32_t)flight_data[sample_point - 1 - i][1];
            ;
          }

          avg_height /= 5;

          // calculate s.d.
          for (uint8_t i = 0; i < 5; i++) {
            sq_values += pow(
                (int32_t)flight_data[sample_point - 1 - i][1] - avg_height, 2);
          }

          height_sd = sqrt(sq_values / 5);

          // if current reading is inside +- 1s.d., switch to landing
          if (fabs((int32_t)flight_data[sample_point][1] - avg_height) <=
              height_sd) {
            flight_state = LANDING;
            break;
          }
        }

        sample_point += 1;
      }

      break;

    // landing state caused by flight array overflow or successful detection,
    // whichever comes first
    case LANDING:
      sprintf(fp_name, "fl_%02d.csv", flight_no);
      f_res = f_open(&fp, fp_name, FA_CREATE_ALWAYS | FA_WRITE);
      if (f_res == FR_OK) {
        // write headers
        f_res = f_puts(
            "pressure,altitude,temperature,raw_accel_x,raw_accel_y,raw_"
            "accel_z,raw_gyro_x,raw_gyro_y,raw_gyro_z,voltage,current,soc\n",
            &fp);
        char init_write_buffer[64] = {0};
        // write inital data
        sprintf(init_write_buffer,
                "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n",
                (int)init_data[0], (int)init_data[1], (int)init_data[2],
                init_data[3], init_data[4], init_data[5], init_data[6],
                init_data[7], init_data[8], (int)init_data[9],
                (int)init_data[10], (int)init_data[11]);

        f_res = f_puts(init_write_buffer, &fp);
        // only write data we have actually captured
        for (uint16_t i = 0; i <= sample_point; i++) {
          char flight_write_buffer[64] = {0};
          sprintf(flight_write_buffer,
                  "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n",
                  (int)flight_data[i][0], (int)flight_data[i][1],
                  (int)flight_data[i][2], flight_data[i][3], flight_data[i][4],
                  flight_data[i][5], flight_data[i][6], flight_data[i][7],
                  flight_data[i][8], (int)flight_data[i][9],
                  (int)flight_data[i][10], (int)flight_data[i][11]);

          f_res = f_puts(flight_write_buffer, &fp);
        }

        // turn on LED to show successful flight
        HAL_GPIO_WritePin(STS_LED_GPIO_Port, STS_LED_Pin, GPIO_PIN_SET);
      }
      f_close(&fp);

      LSM6XX_disable_ff();
      flight_state = DIAG;

      break;
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 127;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 49999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 511;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 62499;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin | SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STS_LED_GPIO_Port, STS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SOC_GPOUT_Pin */
  GPIO_InitStruct.Pin = SOC_GPOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SOC_GPOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin | SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_INT1_Pin IMU_INT2_Pin */
  GPIO_InitStruct.Pin = IMU_INT1_Pin | IMU_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STS_LED_Pin */
  GPIO_InitStruct.Pin = STS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STS_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Definitions for BQ27441 IC
static int16_t BQ27441_i2cWriteBytes(uint8_t DevAddress, uint8_t subAddress,
                                     uint8_t *src, uint8_t count) {
  if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(DevAddress << 1), subAddress, 1, src,
                        count, 50) == HAL_OK)
    return true;
  else
    return false;
}

static int16_t BQ27441_i2cReadBytes(uint8_t DevAddress, uint8_t subAddress,
                                    uint8_t *dest, uint8_t count) {
  if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(DevAddress << 1), subAddress, 1, dest,
                       count, 50) == HAL_OK)
    return true;
  else
    return false;
}

// handlers to enable led blinking

static void blink_short(uint8_t count) {
  led_total = count;
  enable_led = true;
  HAL_TIM_Base_Start_IT(&htim14);
}
static void blink_long(uint8_t count) {
  led_total = count;
  enable_led = true;
  HAL_TIM_Base_Start_IT(&htim16);
}

// Interrupt handler for IMU INT1
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  // interrupt detected
  if (GPIO_Pin == IMU_INT1_Pin && flight_state != INIT_FAILURE) {
    flight_state = LAUNCH;
    sample_point = 0;
    HAL_TIM_Base_Start_IT(&htim14);
  }
  __NOP();
}

// interrupt handler for timers
// LED pulse time controlled by timers - TIM14 is 100ms, TIM16 is 500ms
// TIM14 also handles setting sensor_update flag

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // handle LED blinking
  if (enable_led) {
    // one pulse needs both on and off
    // double the amount of pulses we want
    // led_count is 0-indexed
    if (led_count <= led_total * 2 - 1) {
      HAL_GPIO_TogglePin(STS_LED_GPIO_Port, STS_LED_Pin);
      led_count++;
    } else {
      // reset timer
      led_total = 0;
      led_count = 0;
      enable_led = false;
      // CAREFUL - only disable this timer when we aren't flying
      // otherwise we stop our samples :/
      // 500ms timer always disabled
      if (htim == &htim16 || flight_state != LAUNCH) {
        HAL_TIM_Base_Stop_IT(htim);
      }
    }
  }
  // if we are currently flying, take a sample
  if (flight_state == LAUNCH) {
    take_sample = true;
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
