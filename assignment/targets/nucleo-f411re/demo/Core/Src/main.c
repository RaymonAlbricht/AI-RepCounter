/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * @ref https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dso_reg.h"
#include "stts751_reg.h"
#include "features.h"

#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write_lsm6dso(void *handle, uint8_t reg,
                      const uint8_t *bufp, uint16_t len);
static int32_t platform_read_lsm6dso(void *handle, uint8_t reg,
                     uint8_t *bufp, uint16_t len);
static int32_t platform_write_stss751(void *handle, uint8_t reg,
                      const uint8_t *bufp, uint16_t len);
static int32_t platform_read_stss751(void *handle, uint8_t reg,
                     uint8_t *bufp, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *buf, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, (const uint8_t *)buf, len, HAL_MAX_DELAY);
  return len;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  printf("Initialize sensors\n");

  // --------------------------------------------------------------------------
  // LSM6DSO initialize
  // --------------------------------------------------------------------------

  stmdev_ctx_t dev_ctx_lsm6dso =
  {
    platform_write_lsm6dso, platform_read_lsm6dso, HAL_Delay, &hi2c1
  };

  // Delay boot time
  HAL_Delay(200);

  uint8_t id_lsm6dso;
  lsm6dso_device_id_get(&dev_ctx_lsm6dso, &id_lsm6dso);

  if(id_lsm6dso != LSM6DSO_ID)
  {
    printf("LSM6DSO not found\n");
    while(1)
    {}
  }

  // Restore default configuration
  lsm6dso_reset_set(&dev_ctx_lsm6dso, PROPERTY_ENABLE);

  uint8_t reg;
  do
  {
    lsm6dso_reset_get(&dev_ctx_lsm6dso, &reg);
  }while(reg);

  // Disable I3C interface
  lsm6dso_i3c_disable_set(&dev_ctx_lsm6dso, LSM6DSO_I3C_DISABLE);

  // Enable Block Data Update
  lsm6dso_block_data_update_set(&dev_ctx_lsm6dso, PROPERTY_ENABLE);

  // Set Output Data Rate
  lsm6dso_xl_data_rate_set(&dev_ctx_lsm6dso, LSM6DSO_XL_ODR_104Hz);
  lsm6dso_gy_data_rate_set(&dev_ctx_lsm6dso, LSM6DSO_XL_ODR_OFF);

  // Set full scale
  lsm6dso_xl_full_scale_set(&dev_ctx_lsm6dso, LSM6DSO_2g);
  lsm6dso_gy_full_scale_set(&dev_ctx_lsm6dso, LSM6DSO_2000dps);

  // Configure filtering chain (No aux interface)
  // Accelerometer - LPF1 + LPF2 path
//  lsm6dso_xl_hp_path_on_out_set(&dev_ctx_lsm6dso, LSM6DSO_LP_ODR_DIV_100);
//  lsm6dso_xl_filter_lp2_set(&dev_ctx_lsm6dso, PROPERTY_ENABLE);

  printf("LSM6DSO okay\n");
  printf("ODR = 104Hz\n");
  printf("+/-2g output scale\r\n");

  // --------------------------------------------------------------------------
  // STTS751 initialize
  // --------------------------------------------------------------------------

  stmdev_ctx_t dev_ctx_stss751 =
  {
    platform_write_stss751, platform_read_stss751, HAL_Delay, &hi2c1
  };

  stts751_id_t whoamI_stts751;
  stts751_device_id_get(&dev_ctx_stss751, &whoamI_stts751);

  if((whoamI_stts751.product_id != STTS751_ID_1xxxx) ||
     (whoamI_stts751.manufacturer_id != STTS751_ID_MAN) ||
     (whoamI_stts751.revision_id != STTS751_REV))
  {
    printf("STTS751 not found\n");
    while(1);
  }

  // Set Output Data Rate
  stts751_temp_data_rate_set(&dev_ctx_stss751, STTS751_TEMP_ODR_1Hz);

  // Set Resolution
  stts751_resolution_set(&dev_ctx_stss751, STTS751_11bit);

  printf("STTS751 okay\n");
  printf("ODR = 1Hz\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ----------------------------------------------------------------------
    uint8_t data;
    if(HAL_UART_Receive(&huart2, &data, 1, 0) == HAL_OK)
    {
      if(data == ' ')
      {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }
    }

    // ----------------------------------------------------------------------
    int16_t data_raw_acceleration[3] = {0};
    float acceleration_mg[3] = {0};

    // Read output only if new xl value is available
    reg = 0;
    lsm6dso_xl_flag_data_ready_get(&dev_ctx_lsm6dso, &reg);

    if(reg)
    {
      // Set initial timestamp
      uint32_t ms1 = HAL_GetTick();

      // Read acceleration field data
      lsm6dso_acceleration_raw_get(&dev_ctx_lsm6dso, data_raw_acceleration);
      acceleration_mg[0] = lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] = lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] = lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);

      // ----------------------------------------------------------------------
      //int16_t data_raw_temperature = 0;
      //
      //// Read temperature data
      //stts751_temperature_raw_get(&dev_ctx_stss751, &data_raw_temperature);
      //float temperature_degC = stts751_from_lsb_to_celsius(data_raw_temperature);
      //
      // ----------------------------------------------------------------------

      // Set final timestamp
      uint32_t ms2 = HAL_GetTick();

      // Send the data
      printf("%d,%d,%.1f,%.1f,%.1f\n",
        (int)ms1,
        (int)ms2,
        (double)(acceleration_mg[0]),
        (double)(acceleration_mg[1]),
        (double)(acceleration_mg[2]));

      // TODO Implement filter functions as required by the application.

      // TODO Implement normalization functions as required by the
      //      application.

      // TODO Finish this example by designing an ML model and implement the
      //      generated C code.

      // ----------------------------------------------------------------------

    }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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

#ifdef  USE_FULL_ASSERT
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

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_stss751(void *handle, uint8_t reg,
                      const uint8_t *bufp, uint16_t len)
{
  return HAL_I2C_Mem_Write(handle, STTS751_1xxxx_ADD_7K5, reg,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_stss751(void *handle, uint8_t reg,
                     uint8_t *bufp, uint16_t len)
{
  return HAL_I2C_Mem_Read(handle, STTS751_1xxxx_ADD_7K5, reg,
                          I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_lsm6dso(void *handle, uint8_t reg,
                      const uint8_t *bufp, uint16_t len)
{
  return HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, reg,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_lsm6dso(void *handle, uint8_t reg,
                     uint8_t *bufp, uint16_t len)
{
  return HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, reg,
                          I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}
