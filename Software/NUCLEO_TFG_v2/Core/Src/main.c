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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// --- Sensor de presion EPT1400 ---
#define EPT1400_PMAX        100.0f   // bar del modelo
#define EPT1400_VMIN        0.5f     // V a 0 bar (offset del sensor, del datasheet)
#define EPT1400_VSPAN       4.0f     // V de recorrido (4.5 - 0.5, del datasheet)
#define EPT1400_DIVIDER_K   0.6471f  // divisor R1=10k, R2=18.2k -> 4.5V se mapea a 2.91V

// --- Sensor de temperatura de rueda MLX90614 ---
#define MLX90614_ADDR     (0x5A << 1)   // direccion 7-bit desplazada para HAL
#define MLX90614_TOBJ1    0x07          // registro temperatura del objeto

// --- IDs de las tramas CAN ---
#define CAN_ID_APPS     0x100   // acelerador (APPS_1, APPS_2)
#define CAN_ID_SUSP     0x101   // suspension (SUSP_FL, SUSP_FR)
#define CAN_ID_BRAKE    0x102   // presion de freno
#define CAN_ID_TEMPS    0x103   // temperaturas (disco, rueda1, rueda2)

// --- Periodos del scheduler (ms) ---
#define PERIOD_FAST     10      // 100 Hz: APPS, suspension, freno
#define PERIOD_SLOW     250     // 4 Hz: temperaturas

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

//ADC
#define ADC_N_CHANNELS 5
uint16_t adc_val[ADC_N_CHANNELS];
enum { APPS_1 = 0, APPS_2, SUSP_FL, SUSP_FR, BRAKE };

//SPI
float   temp_disco    = 0.0f;   // °C, temperatura disco de freno (MAX6675)
uint8_t max6675_fault = 0;      // 1 = termopar desconectado

//I2C
float temp_tyre1 = 0.0f;   // °C, temperatura rueda 1 (MLX90614 en I2C1)
float temp_tyre2 = 0.0f;   // °C, temperatura rueda 2 (MLX90614 en I2C2)

//CAN
FDCAN_TxHeaderTypeDef TxHeaderApps;
FDCAN_TxHeaderTypeDef TxHeaderSusp;
FDCAN_TxHeaderTypeDef TxHeaderBrake;
FDCAN_TxHeaderTypeDef TxHeaderTemps;

uint32_t t_last_fast = 0;
uint32_t t_last_slow = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */

static void CAN_InitHeader(FDCAN_TxHeaderTypeDef *h, uint32_t id, uint32_t dlc);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CAN_InitHeader(FDCAN_TxHeaderTypeDef *h, uint32_t id, uint32_t dlc)
{
    h->Identifier          = id;
    h->IdType              = FDCAN_STANDARD_ID;
    h->TxFrameType         = FDCAN_DATA_FRAME;
    h->DataLength          = dlc;
    h->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h->BitRateSwitch       = FDCAN_BRS_ON;
    h->FDFormat            = FDCAN_FD_CAN;
    h->TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    h->MessageMarker       = 0;
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
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  //ADC CALIBRACIÓN
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }

  // Cabeceras de las 4 tramas
  CAN_InitHeader(&TxHeaderApps,  CAN_ID_APPS,  FDCAN_DLC_BYTES_4);  // 2x uint16
  CAN_InitHeader(&TxHeaderSusp,  CAN_ID_SUSP,  FDCAN_DLC_BYTES_4);  // 2x uint16
  CAN_InitHeader(&TxHeaderBrake, CAN_ID_BRAKE, FDCAN_DLC_BYTES_2);  // 1x uint16
  CAN_InitHeader(&TxHeaderTemps, CAN_ID_TEMPS, FDCAN_DLC_BYTES_8);  // 3x int16 + flags

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {     uint32_t now = HAL_GetTick();

  // ===== GRUPO RÁPIDO (100 Hz) =====
  if (now - t_last_fast >= PERIOD_FAST)
  {
      t_last_fast = now;

      // --- Lectura ADC (5 canales) ---
      HAL_ADC_Start(&hadc1);
      for (int i = 0; i < ADC_N_CHANNELS; i++)
      {
          if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
              adc_val[i] = HAL_ADC_GetValue(&hadc1);
          else
              adc_val[i] = 0xFFFF;
      }
      HAL_ADC_Stop(&hadc1);

      // Presion de freno en bar, escalada x10 -> uint16
      float v_brake_adc = (adc_val[BRAKE] * 3.3f) / 4095.0f;
      float v_brake     = v_brake_adc / EPT1400_DIVIDER_K;
      float p_brake     = (v_brake - EPT1400_VMIN) * EPT1400_PMAX / EPT1400_VSPAN;
      uint16_t brake_x10 = (uint16_t)(p_brake * 10.0f);

      // --- Empaquetado y envio: APPS (raw ADC, big-endian) ---
      uint8_t dApps[4];
      dApps[0] = (uint8_t)(adc_val[APPS_1] >> 8);
      dApps[1] = (uint8_t)(adc_val[APPS_1] & 0xFF);
      dApps[2] = (uint8_t)(adc_val[APPS_2] >> 8);
      dApps[3] = (uint8_t)(adc_val[APPS_2] & 0xFF);
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderApps, dApps);

      // --- Suspension (raw ADC) ---
      uint8_t dSusp[4];
      dSusp[0] = (uint8_t)(adc_val[SUSP_FL] >> 8);
      dSusp[1] = (uint8_t)(adc_val[SUSP_FL] & 0xFF);
      dSusp[2] = (uint8_t)(adc_val[SUSP_FR] >> 8);
      dSusp[3] = (uint8_t)(adc_val[SUSP_FR] & 0xFF);
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderSusp, dSusp);

      // --- Presion de freno (bar x10) ---
      uint8_t dBrake[2];
      dBrake[0] = (uint8_t)(brake_x10 >> 8);
      dBrake[1] = (uint8_t)(brake_x10 & 0xFF);
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderBrake, dBrake);
  }

  // ===== GRUPO LENTO (4 Hz) =====
  if (now - t_last_slow >= PERIOD_SLOW)
  {
      t_last_slow = now;

      // --- Lectura SPI (MAX6675) ---
      uint8_t  spi_rx[2] = {0};
      uint16_t spi_raw   = 0;
      HAL_GPIO_WritePin(SPI2_TEMP_BRK_GPIO_Port, SPI2_TEMP_BRK_Pin, GPIO_PIN_RESET);
      if (HAL_SPI_Receive(&hspi2, spi_rx, 2, 10) == HAL_OK)
      {
          spi_raw = ((uint16_t)spi_rx[0] << 8) | spi_rx[1];
          if (spi_raw & 0x0004)
          {
              max6675_fault = 1;
              temp_disco = -1.0f;
          }
          else
          {
              max6675_fault = 0;
              temp_disco = ((spi_raw >> 3) & 0x0FFF) * 0.25f;
          }
      }
      HAL_GPIO_WritePin(SPI2_TEMP_BRK_GPIO_Port, SPI2_TEMP_BRK_Pin, GPIO_PIN_SET);

      // --- Lectura I2C (2x MLX90614) ---
      uint8_t mlx_buf[2] = {0};
      if (HAL_I2C_Mem_Read(&hi2c1, MLX90614_ADDR, MLX90614_TOBJ1,
                           I2C_MEMADD_SIZE_8BIT, mlx_buf, 2, 10) == HAL_OK)
          temp_tyre1 = (((uint16_t)mlx_buf[1] << 8) | mlx_buf[0]) * 0.02f - 273.15f;

      if (HAL_I2C_Mem_Read(&hi2c2, MLX90614_ADDR, MLX90614_TOBJ1,
                           I2C_MEMADD_SIZE_8BIT, mlx_buf, 2, 10) == HAL_OK)
          temp_tyre2 = (((uint16_t)mlx_buf[1] << 8) | mlx_buf[0]) * 0.02f - 273.15f;

      // --- Empaquetado: temperaturas (int16, °C x10) + flag ---
      int16_t td  = (int16_t)(temp_disco * 10.0f);
      int16_t tt1 = (int16_t)(temp_tyre1 * 10.0f);
      int16_t tt2 = (int16_t)(temp_tyre2 * 10.0f);

      uint8_t dTemps[8];
      dTemps[0] = (uint8_t)(td  >> 8);
      dTemps[1] = (uint8_t)(td  & 0xFF);
      dTemps[2] = (uint8_t)(tt1 >> 8);
      dTemps[3] = (uint8_t)(tt1 & 0xFF);
      dTemps[4] = (uint8_t)(tt2 >> 8);
      dTemps[5] = (uint8_t)(tt2 & 0xFF);
      dTemps[6] = max6675_fault;     // byte de flags
      dTemps[7] = 0;                 // reservado
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeaderTemps, dTemps);

      // --- Debug opcional (descomentar para ver por terminal) ---
      // printf("BRK:%.1f bar | Disco:%.1f Rueda1:%.1f Rueda2:%.1f C (fault=%u)\r\n",
      //        p_brake, temp_disco, temp_tyre1, temp_tyre2, max6675_fault);
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
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
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_TEMP_BRK_GPIO_Port, SPI2_TEMP_BRK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI2_TEMP_BRK_Pin */
  GPIO_InitStruct.Pin = SPI2_TEMP_BRK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_TEMP_BRK_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
