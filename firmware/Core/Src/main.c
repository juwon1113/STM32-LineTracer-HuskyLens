/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <stdarg.h>

#include  "stm32f429i_discovery.h"
#include  "stm32f429i_discovery_lcd.h"

#include "HUSKYLENS_C.h"
#include "DcMotor.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_PC huart1
#define UART_HUSKYLENS huart5

#define abs(x)  ((x)==0 ? 0 : ((x)>0 ? (x) : -(x)  ))

#define CENTER	160   // HuskyLens 화면(320 x 240) 가로 중앙
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
int flagTim6;

int LcdMaxX, LcdMaxY;
int posX, posY;
huskey_arrow_t husky_arrow;//origin, arrow_target;
char string1[100],string2[100];

PID_t Diff, Angle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_DMA2D_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM9_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}

int HuskyLens_Start()
{
	  printf("\r\nStart\r\n"); // 429=>PC "Start\r\n"

	  HuskyLens_PacketMake(); // 허스키렌즈 관련 패킷을 만들기

	  // 4= strlen(" 1: ") 디버깅용
	  HAL_UART_Transmit(&UART_PC, (uint8_t *)" 1: ", 4, 0x100);

	  // 429=>Huskey    Knock 명령 전송
	  HuskyLens_PacketPrint(&HuskyLensPacket_Command_Knock);
	  HuskyLens_PacketTransmitHuskyLens(&HuskyLensPacket_Command_Knock);

	  if((HuskyLensPacket_Command_Status=HuskyLens_PacketReceive(&HuskyLensPacket_Return6)) == COMMAND_REQUEST_ERROR){
		  HAL_UART_Transmit(&UART_PC, (uint8_t *)" 2: ", 4, 0x100);
		  HuskyLens_PacketPrint(&HuskyLensPacket_Return6);
		  return HuskyLensPacket_Command_Status;
	  }

	  // Compare string 1 and string 2 by length
	  // if same contents, return 0, else return +1, -1 by comarision result.
	  if( strncmp((char *)(&HuskyLensPacket_Return6)->contents, (char *)(&HuskyLensPacket_Command_ReturnOk)->contents, (&HuskyLensPacket_Return6)->length) ){
		  HAL_UART_Transmit(&UART_PC, (uint8_t *)" 3: ", 4, 0x100);
		  HuskyLens_PacketPrint(&HuskyLensPacket_Return6);
		  return -1;
	  }

	  HAL_UART_Transmit(&UART_PC, (uint8_t *)" 4: ", 4, 0x100);
	  HuskyLens_PacketPrint(&HuskyLensPacket_Return6);//expect ReturnOK

	  return 1;
}

/* HuskyLens가 준 라인 화살표로 좌우 바퀴 속도를 계산한다
 * - Difference: 화살표 시작점이 화면 중앙에서 벗어난 정도 (라인 위치 편차)
 * - angle: 화살표가 세로축에서 기울어진 각도 (라인 방향)
 * 두 값을 각각 PID로 0에 맞추고, 그 합을 좌우 바퀴에 반대 부호로 더해 방향을 튼다 */
void CarDriving(huskey_arrow_t arrow)
{
	float Difference;
	float CarVelocity_L, CarVelocity_R;
	float angle;
	float basePwm = 70.0f;

	Difference = arrow.origin.x - CENTER ;
	angle = (180.0/M_PI * atan2((arrow.target.x-arrow.origin.x), (-arrow.target.y+arrow.origin.y)));

	//HuskyLens, Motor test routine
	//여기에 알고리즘 추가
	// 위치 편차 PID: 라인이 화면 중앙에 오도록
	PID_SetTarget(&Diff, 0.0f);
	PID_Compute(&Diff, Difference);

	// 각도 PID: 라인 방향과 차체 방향이 나란하도록
	PID_SetTarget(&Angle, 0.0f);
	PID_Compute(&Angle, angle);

	CarVelocity_L = basePwm - (Diff.pidOutput + Angle.pidOutput);
	CarVelocity_R = basePwm + (Diff.pidOutput + Angle.pidOutput);

	if(CarVelocity_L > 100) CarVelocity_L = 100;
	if(CarVelocity_L < 0) CarVelocity_L = 0;
	if(CarVelocity_R > 100) CarVelocity_R = 100;
	if(CarVelocity_R < 0) CarVelocity_R = 0;

	//DcMotorAllSetDutyPulse(&htim9, 50, 50);
	DcMotorAllSetDutyPulse(&htim9, CarVelocity_L, CarVelocity_R);
}

/* LCD에 라인 방향 화살표를 그린다
 * 이전 화살표를 배경색으로 지우고 새 화살표를 그려서, 화면 전체를 다시 그리지 않아도 되게 했다 */
void DrawDirectionArrow(float angle)
{
    static int prev_x = 120, prev_y = 160;
    static int prev_wx1 = 120, prev_wy1 = 160;
    static int prev_wx2 = 120, prev_wy2 = 160;

    int cx = 120, cy = 160, R = 90;

    float rad = (angle + 180.0f) * M_PI / 180.0f;

    int x = cx + (int)(R * sin(rad) + 0.5);
    int y = cy - (int)(R * cos(rad) + 0.5);
    float wa = 2.618;
    int wx1 = x + (int)(22 * sin(rad + wa) + 0.5);
    int wy1 = y - (int)(22 * cos(rad + wa) + 0.5);
    int wx2 = x + (int)(22 * sin(rad - wa) + 0.5);
    int wy2 = y - (int)(22 * cos(rad - wa) + 0.5);

    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawLine(cx, cy, prev_x, prev_y);
    BSP_LCD_DrawLine(prev_x, prev_y, prev_wx1, prev_wy1);
    BSP_LCD_DrawLine(prev_x, prev_y, prev_wx2, prev_wy2);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawLine(cx, cy, x, y);
    BSP_LCD_DrawLine(x, y, wx1, wy1);
    BSP_LCD_DrawLine(x, y, wx2, wy2);

    prev_x = x; prev_y = y;
    prev_wx1 = wx1; prev_wy1 = wy1;
    prev_wx2 = wx2; prev_wy2 = wy2;
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
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_DMA2D_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  // ── LCD 초기화 ──────────────────────────────────────────────
  BSP_LCD_Init();

  // 레이어는 하나(배경)만 사용한다.
  // 기존에 background / foreground 두 레이어를 같은 LCD_FRAME_BUFFER 주소로
  // 중복 init 하던 것을 제거하고, 그릴 레이어를 명시적으로 선택한다.
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

  BSP_LCD_DisplayOn();

  // ★ 핵심: 시작할 때 화면 전체를 검은색으로 한 번 지워준다.
  //   (프레임버퍼의 초기 쓰레기 값 때문에 화면이 깨져 보이는 문제 해결)
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  // ────────────────────────────────────────────────────────────

  printf("\r\nStart\r\n");

  HAL_GPIO_WritePin(DCMOTOR1_DIR_GPIO_Port, DCMOTOR1_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DCMOTOR2_DIR_GPIO_Port, DCMOTOR2_DIR_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DCMOTOR1_EN_GPIO_Port, DCMOTOR1_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DCMOTOR2_EN_GPIO_Port, DCMOTOR2_EN_Pin, GPIO_PIN_RESET);

  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2 );
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2 );
  HAL_TIM_Base_Start_IT(&htim6);

  HuskyLens_Start();

  // PID 초기화: Kp, Ki, Kd, dt(5 ms), 초기 목표, 출력 상한, 하한
  PID_Init(&Diff, 0.5, 0, 0.005, 0.005, 0, 100, -100);
  PID_Init(&Angle, 1.5, 0, 0.030, 0.005, 0, 100, -100);

  uint8_t start = 0, count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	  // 최적으로 제어는 이 아래를 타임인터럽트로 이동 동작.
	  if (HAL_GPIO_ReadPin(USER_GPIO_Port, USER_Pin))
	  {
		  start = 1;
	  }

	  if (start)
	  {
		  if( HuskyLens_PacketGetArrow() != COMMAND_REQUEST_ERROR)
		  {
			  count++;
			  // 출력은 화살표 5개마다 한 번만 (LCD·블루투스 출력이 제어 주기를 너무 늦추지 않도록)
			  if (count >= 5)
			  {
				  float angle = ( 180.0/M_PI * atan2(  (husky_arrow.target.x-husky_arrow.origin.x), (-husky_arrow.target.y+husky_arrow.origin.y)   ) );
				  DrawDirectionArrow(angle);
				  printf("arrow (%03d, %03d)->(%03d, %03d)\r\n",husky_arrow.origin.x, husky_arrow.origin.y,husky_arrow.target.x,husky_arrow.target.y);
				  printf("angle= %5.2f deg \r\n ", angle);

			  }
		  }
		  CarDriving(husky_arrow);

		  HAL_Delay(5);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 19;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 259;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 230400;
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

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* ★ SDRAM 기동 명령 시퀀스 (이게 빠지면 SDRAM이 동작하지 않아 화면이 지지직거림) */
  FMC_SDRAM_CommandTypeDef Command = {0};

  /* 1) Clock Configuration Enable */
  Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand(&hsdram1, &Command, 0xFFFF);

  /* 2) 최소 100us 대기 */
  HAL_Delay(1);

  /* 3) Precharge All (PALL) */
  Command.CommandMode            = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand(&hsdram1, &Command, 0xFFFF);

  /* 4) Auto-Refresh (8회) */
  Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 8;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand(&hsdram1, &Command, 0xFFFF);

  /* 5) Load Mode Register
        Burst Length 1 | Sequential | CAS Latency 3 | Standard | Write Burst Single */
  Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = (uint32_t)(0x0000   /* Burst Length = 1        */
                                            | 0x0000   /* Burst Type = Sequential */
                                            | 0x0030   /* CAS Latency = 3         */
                                            | 0x0000   /* Operating Mode = Std    */
                                            | 0x0200); /* Write Burst = Single    */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, 0xFFFF);

  /* 6) Refresh Rate 설정 (SDRAM 클럭 84MHz 기준) */
  HAL_SDRAM_ProgramRefreshRate(&hsdram1, 0x050C);

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DCMOTOR1_EN_Pin|DCMOTOR1_DIR_Pin|DCMOTOR2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMOTOR2_DIR_GPIO_Port, DCMOTOR2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DCMOTOR1_EN_Pin DCMOTOR1_DIR_Pin DCMOTOR2_EN_Pin */
  GPIO_InitStruct.Pin = DCMOTOR1_EN_Pin|DCMOTOR1_DIR_Pin|DCMOTOR2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMOTOR2_DIR_Pin */
  GPIO_InitStruct.Pin = DCMOTOR2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMOTOR2_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Pin */
  GPIO_InitStruct.Pin = USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
