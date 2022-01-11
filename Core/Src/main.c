/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * Author: Kacper Bukowski
  * Kod został przygotowany do celów dyplomowania na stopień inżyniera Wojskowej Akademii Technicznej
  *
  ******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t check_var_value_once = 0;
uint8_t znak; //Przechowywanie informacji o aktualnie pobranym znaku z interfejsu UART
uint32_t throttle_manual = 20; //Zgodnie z dokumentacją producenta sterownika silnika BLDC, minimal throttle = 20%
uint16_t dl_kom; //zmienna przechowująca długość komunikatu przesyłanego poprzez interfejs UART
uint8_t komunikat[350]; //Tablica przechowująca komunikat
uint8_t wybrana_opcja = 'c'; //Aktualnie wybrana opcja z menu (domyślnie 'c')
uint8_t number_of_menu_lines = 10; //Liczba wierszy menu
uint16_t max_duty_cycle = 2000; //Maksymalny czas trwania impulsu PWM
uint16_t min_duty_cycle = 1000; //Minimalny czas trwania impulsu PWM
uint16_t min_duty_cycle_for_balancing = 1210; //Minimalny czas trwania impulsu PWM dla opcji balansowania
uint16_t maximum_motor_power_while_balancing = 29; //Wyrażony w % maksymalny ciąg silnika. Wymagane do zachowania bezpieczeństwa
const char *menu_screen[10] ={
		"PID SELF-BALANCING ROBOT CONFIGURATION PANEL\n\r",
		"Choose the proper option from the menu below:\n\r",
		"1.Control the left BLDC motor manually\n\r",
		"2.Control the right BLDC motor manually\n\r",
		"3.Start balancing\n\r",
		"4.Check Kp value\n\r",
		"5.Check Ki value\n\r",
		"6.Check Kd value\n\r",
		"7.Live MPU values check\n\n\r",
		"You can perform an emergency stop by sending 'h' via USART interface or clicking blue button on the board\n\r"};
uint8_t exit_switch_options = 0; //0 -> główne menu, 1 -> podmenu
float level_correction = 740; // wartość poziomu w osi Y dla akcelerometru
float rgb_animation_counter = 0; // licznik do animacji RGB
pid_bldc_controller_structure pid_regulator;
PID_Motor_Speed_Calculation_Structure PID_Motor;
float min_pid_correction_value = -14000;
float max_pid_correction_value = 14000;

//Współczynniki PID
float Kp = 1.2;
float Ki = 0.0000025;
float Kd = 0.001;
float windup_correction = 300;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Choosen_USART_Option(uint8_t input_char){
	wybrana_opcja = input_char;
	dl_kom = sprintf((char *)komunikat,"Option no.%d, have been choosen.\n\r",input_char);
	HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
}
void Option_1_performer(uint32_t throttle_manual_input){
	//Sekcja poświęcona obsłudze lewego silnika BLDC
	if (throttle_manual_input < 20)
	{
		throttle_manual_input = 20;
	}
	float throttle_manual_input_temp = ((float)throttle_manual_input/100 * (max_duty_cycle-min_duty_cycle))+min_duty_cycle;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,throttle_manual_input_temp);
}
void Option_2_performer(uint32_t throttle_manual_input){
	//Sekcja poświęcona obsłudze prawego silnika BLDC
	if (throttle_manual_input < 20)
		{
			throttle_manual_input = 20;
		}
	float throttle_manual_input_temp = ((float)throttle_manual_input/100 * (max_duty_cycle-min_duty_cycle))+min_duty_cycle;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,throttle_manual_input_temp);
}
void Option_3_performer(RawData_Def *ACC_DATA){
	float left_motor_speed = PID_Motor_Speed_Calculation_duty_cycle_ms_left(&PID_Motor, pid_calculate(&pid_regulator, level_correction, ACC_DATA->y));
	float right_motor_speed = PID_Motor_Speed_Calculation_duty_cycle_ms_right(&PID_Motor, pid_calculate(&pid_regulator, level_correction, ACC_DATA->y));
	if (left_motor_speed < 1200){
			left_motor_speed = 1200;
		}
	if (right_motor_speed < 1200){
			right_motor_speed = 1200;
		}
	dl_kom = sprintf((char *)komunikat, "PID Correction value: %d\n\n\r",pid_calculate(&pid_regulator, level_correction, ACC_DATA->y)); // @suppress("Float formatting support")
	HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
	dl_kom = sprintf((char *)komunikat, "Left motor power: %d\n\n\r",(int)left_motor_speed); // @suppress("Float formatting support")
	HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
	dl_kom = sprintf((char *)komunikat, "Right motor power: %d\n\n\r",(int)right_motor_speed); // @suppress("Float formatting support")
	HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,left_motor_speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,right_motor_speed+14); //Prawy silnik ma większe opory, wymagana korekta prędkości
}
void Option_4_performer(){
		dl_kom = sprintf((char *)komunikat, "Kp value: %d\n\n\r",(int)Kp);
		HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
}
void Option_5_performer(){
		dl_kom = sprintf((char *)komunikat, "Ki value: %d\n\n\r",(int)Ki);
		HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
}
void Option_6_performer(){
		dl_kom = sprintf((char *)komunikat, "Kd value: %d\n\n\r",(int)Kd);
		HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
}
void Option_7_performer(RawData_Def *MPU6050A_RAW,RawData_Def *MPU6050G_RAW,ScaledData_Def *MPU6050A_Scaled,ScaledData_Def *MPU6050G_Scaled){
	//Sekcja poświęcona przekazywaniu danych z czujnika MPU6050 na interfejs szeregowy
	MPU6050_Get_Accel_RawData(MPU6050A_RAW);
	MPU6050_Get_Gyro_RawData(MPU6050G_RAW);
	MPU6050_Get_Accel_Scale(MPU6050A_Scaled);
	MPU6050_Get_Gyro_Scale(MPU6050G_Scaled);

	//Komunikaty portu szeregowego
	dl_kom = sprintf((char *)komunikat, "RAW ACCEL DATA: X = %d, Y = %d, Z = %d\n\r",MPU6050A_RAW->x,MPU6050A_RAW->y,MPU6050A_RAW->z);
	HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
	dl_kom = sprintf((char *)komunikat, "RAW GYRO DATA: X = %d, Y = %d, Z = %d\n\r",MPU6050G_RAW->x,MPU6050G_RAW->y,MPU6050G_RAW->z);
	HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
	dl_kom = sprintf((char *)komunikat, "SCALED ACCEL DATA: X = %d, Y = %d, Z = %d\n\r",(int)MPU6050A_Scaled->x,(int)MPU6050A_Scaled->y,(int)MPU6050A_Scaled->z); // @suppress("Float formatting support")
	HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
	dl_kom = sprintf((char *)komunikat, "SCALED GYRO DATA: X = %d, Y = %d, Z = %d\n\n\r",(int)MPU6050G_Scaled->x,(int)MPU6050G_Scaled->y,(int)MPU6050G_Scaled->z); // @suppress("Float formatting support")
	HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
	HAL_Delay(500);

}

void MPU6050_Update(RawData_Def *MPU6050A_RAW,RawData_Def *MPU6050G_RAW,ScaledData_Def *MPU6050A_Scaled,ScaledData_Def *MPU6050G_Scaled){
	MPU6050_Get_Accel_RawData(MPU6050A_RAW);
	MPU6050_Get_Gyro_RawData(MPU6050G_RAW);
	MPU6050_Get_Accel_Scale(MPU6050A_Scaled);
	MPU6050_Get_Gyro_Scale(MPU6050G_Scaled);
}

void RGB_LED_Level_Notificator(RawData_Def *MPU6050A_RAW){
	//Obsługa LED

	//Lewy LED RGB
	//Na podstawie danych RAW z akcelerometru w osi Y (zgodnie z oznaczeniami na module MPU6050)
	float duty_g;
	float duty_r;
	float correction_variable;

	if(MPU6050A_RAW->y > level_correction){
		//Obliczenie wypełnienia PWM
		correction_variable = (6100 - level_correction)/255;
		duty_g = abs(MPU6050A_RAW->y - level_correction) / correction_variable;
		duty_r = 255 - (abs(MPU6050A_RAW->y - level_correction) / correction_variable);

		//Ustawienie koloru
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,duty_g);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1,duty_r);
	}
	else if(MPU6050A_RAW->y < level_correction){
		duty_r = 255;
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1,duty_r);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,1);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1,255);
		HAL_Delay(200);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,255);
		HAL_Delay(200);
	}


	//Prawy LED RGB
	//Na podstawie danych RAW z akcelerometru w osi Y (zgodnie z oznaczeniami na module MPU6050)
	if(MPU6050A_RAW->y < level_correction){
			//Obliczenie wypełnienia PWM
			correction_variable = abs((5700 - level_correction)/255);
			duty_g = abs(MPU6050A_RAW->y - level_correction) / correction_variable;
			duty_r = 255 - (abs(MPU6050A_RAW->y - level_correction) / correction_variable);

			//Ustawienie koloru
			__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,duty_g);
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1,duty_r);
		}
		else if(MPU6050A_RAW->y > level_correction){
			duty_r = 255;
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1,duty_r);
			__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,1);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,255);
			HAL_Delay(200);
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1,255);
			HAL_Delay(200);
		}
}

void RGB_Standby_Animation(){
	if(rgb_animation_counter <= 255){
		//Animowanie lewej diody RGB
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1,abs(rgb_animation_counter));
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,255 - abs(rgb_animation_counter));
		//Animowanie prawej diody RGB
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1,abs(rgb_animation_counter));
			__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,255 - abs(rgb_animation_counter));

		//Inkrementacja zmiennej rgb_animation_counter
			HAL_Delay(2);
			rgb_animation_counter++;
	}
	else{
		rgb_animation_counter = -255;
	}
}

void Stop_BLDCs (){
	//Zatrzymywanie silników (lewy+prawy)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,min_duty_cycle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,min_duty_cycle);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  //Struktury danych MPU
  MPU_ConfigTypeDef MPU6050_Config_DATA; //Dane konfiguracyjne
  RawData_Def MPU6050A_RAW, MPU6050G_RAW; //Dane z czujnika (nieprzetworzone)
  ScaledData_Def MPU6050A_Scaled, MPU6050G_Scaled; //Dane z czujnika (przetworzone filtrem Kalmana)

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  //Inicjalizacja modułu MPU6050 i połączenia I2C
  MPU6050_Init(&hi2c1);

  //Konfiguracja modułu MPU6050
  MPU6050_Config_DATA.Accel_Full_Scale = AFS_SEL_4g;
  MPU6050_Config_DATA.ClockSource = Internal_8MHz;
  MPU6050_Config_DATA.CONFIG_DLPF = DLPF_184A_188G_Hz;
  MPU6050_Config_DATA.Gyro_Full_Scale = FS_SEL_500;
  MPU6050_Config_DATA.Sleep_Mode_Bit = 0;
  MPU6050_Config(&MPU6050_Config_DATA);

  //Inicjalizacja generatora sygnału PWM oraz wyłączenie silników BLDC
  HAL_UART_Receive_IT(&huart2, &znak, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // Lewa G
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); // Lewa R
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // Prawa G
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); // Prawa R
  Stop_BLDCs();
  pid_init(&pid_regulator, Kp, Ki, Kd, windup_correction);
  PID_Motor_Speed_Calculation_Init(&PID_Motor, maximum_motor_power_while_balancing, max_duty_cycle, min_duty_cycle_for_balancing, min_pid_correction_value, max_pid_correction_value);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Pętla nieskończona wykonywana jest na podstawie wybranej opcji z menu. Wybór odbywa się w trybie przerwaniowym.
	  switch(wybrana_opcja){
	  case 1:
		  // Obsługa opcji numer 1 z menu: "1.Control the left BLDC motor manually"
		  Option_1_performer(throttle_manual);
		  break;
	  case 2:
		  // Obsługa opcji numer 2 z menu: "2.Control the right BLDC motor manually"
		  Option_2_performer(throttle_manual);
		  break;
	  case 3:
		  // Obsługa opcji numer 3 z menu: "3.Start balancing"
		  MPU6050_Update(&MPU6050A_RAW, &MPU6050G_RAW, &MPU6050A_Scaled, &MPU6050G_Scaled);
		  Option_3_performer(&MPU6050A_RAW);
		  RGB_LED_Level_Notificator(&MPU6050A_RAW);
		  break;
	  case 4:
		  // Obsługa opcji numer 4 z menu: "4.Check Kp value"
		  if (check_var_value_once == 0){
			  Option_4_performer();
			  check_var_value_once = 1;
		  }
		  break;
	  case 5:
		  // Obsługa opcji numer 5 z menu: "5.Check Ki value"
		  if (check_var_value_once == 0){
			  Option_5_performer();
			  check_var_value_once = 1;
		  }
		  break;
	  case 6:
		  // Obsługa opcji numer 6 z menu: "6.Check Kd value"
		  if (check_var_value_once == 0){
			  Option_5_performer();
			  check_var_value_once = 1;
		  }
		  break;
	  case 7:
		  // Obsługa opcji numer 7 z menu: "7.Live MPU values check"
		  Option_7_performer(&MPU6050A_RAW, &MPU6050G_RAW, &MPU6050A_Scaled, &MPU6050G_Scaled);
		  RGB_LED_Level_Notificator(&MPU6050A_RAW);
		  break;
	  case 'c':
		  // Obsługa powrotu do menu głównego
		  Stop_BLDCs();
		  RGB_Standby_Animation();
		  check_var_value_once = 0;
		  pid_reset(&pid_regulator);
		  break;
	  case 'h':
		  // Obsługa awaryjnego zatrzymania silników
		  Stop_BLDCs();
		  break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM17;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 71;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 255;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 71;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 255;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 71;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 255;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART2){
		if(znak == 'c'){
			for (uint8_t i=0; i<number_of_menu_lines;i++){
				dl_kom = sprintf((char *)komunikat,menu_screen[i]);
				HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				exit_switch_options = 0;
				wybrana_opcja = znak;
			}
		}
		else if(znak =='h'){
			dl_kom = sprintf((char *)komunikat, "\n\n\rPerforming EMERGENCY STOP!\n\n\r");
			HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
			wybrana_opcja = znak;
		}
		else if(exit_switch_options == 0)
		{
			switch(znak){
			case 1:
				throttle_manual = 0;
				Choosen_USART_Option(znak);
				dl_kom = sprintf((char *)komunikat, "Control the throttle of the left motor by sending integer values (0-100).\n\r To exit this mode type 'c'\n\n\r");
				HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				dl_kom = sprintf((char *)komunikat, "Starting the left motor...\n\r");
				HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				dl_kom = sprintf((char *)komunikat, "Use numbers to control the throttle\n\r");
				HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				exit_switch_options = 1;
				break;
			case 2:
				throttle_manual = 0;
				Choosen_USART_Option(znak);
				dl_kom = sprintf((char *)komunikat, "Control the throttle of the right motor by sending integer values (0-100).\n\r To exit this mode type 'c'\n\n\r");
				HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				dl_kom = sprintf((char *)komunikat, "Starting the right motor...\n\r");
				HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				dl_kom = sprintf((char *)komunikat, "Use numbers to control the throttle\n\r");
				HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				exit_switch_options = 1;
				break;
			case 3:
				Choosen_USART_Option(znak);
				break;
			case 4:
				Choosen_USART_Option(znak);
				break;
			case 5:
				Choosen_USART_Option(znak);
				break;
			case 6:
				Choosen_USART_Option(znak);
				break;
			case 7:
				Choosen_USART_Option(znak);
				break;
			default:
				dl_kom = sprintf((char *)komunikat, "\n\n\r The given option is not available in the menu scope. Please try again.\n\n\r");
				HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
			}
		}
		else
		{
			switch (wybrana_opcja){
			case 1:
				if(znak >= 0 && znak <= 100){
					throttle_manual = znak;
					dl_kom = sprintf((char *)komunikat, "Setting throttle value to %d!\n\r",znak);
					HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				}
				else{
					dl_kom = sprintf((char *)komunikat, "Wrong value!\n\r");
					HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				}
			case 2:
				if(znak >= 0 && znak <= 100){
					throttle_manual = znak;
					dl_kom = sprintf((char *)komunikat, "Setting throttle value to %d!\n\r",znak);
					HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				}
				else{
					dl_kom = sprintf((char *)komunikat, "Wrong value!\n\r");
					HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);
				}
			}

		}
	}
	HAL_UART_Receive_IT(&huart2, &znak, 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == B1_Pin){
		wybrana_opcja = 'h';
		dl_kom = sprintf((char *)komunikat, "\n\n\rPerforming EMERGENCY STOP!\n\n\r");
		HAL_UART_Transmit(&huart2, komunikat, dl_kom, HAL_MAX_DELAY);

	}
}
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

