/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	 #include "MPU6050.h"
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
	 uint8_t buffer[2];
	 HAL_StatusTypeDef Status;
	 uint16_t size;
	 uint8_t Data[64];
	 // rotation angle of the sensor
	 unsigned long last_read_time;
	 float         last_gyro_x_angle;  // Store the gyro angles to compare drift
	 float         last_gyro_y_angle;
	 float         last_gyro_z_angle;
	 unsigned long t_now;
	 float dt = 0;
	 float gyro_angle_x;
	 float gyro_angle_y;
	 float gyro_angle_z;
	 float accel_vel_x;
	 float accel_vel_y;
	 float accel_vel_z;
	 float last_accel_vel_x;
	 float last_accel_vel_y;
	 float last_accel_vel_z;
	 float accel_pos_x;
	 float accel_pos_y;
	 float accel_pos_z;
	 float last_accel_pos_x;
	 float last_accel_pos_y;
	 float last_accel_pos_z;
	 float Accel_samples_x[20];
	 uint8_t sample_count = 0;
	 uint8_t prumer;
	 
void set_last_read_angle_data(unsigned long time, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float x_gyro, float y_gyro, float z_gyro) {
   last_read_time = time;
	 last_accel_vel_x = x_vel;
	 last_accel_vel_y = y_vel;
	 last_accel_vel_z = z_vel;
	 last_accel_pos_x = x_pos;
	 last_accel_pos_y = y_pos;
	 last_accel_pos_z = z_pos;
   last_gyro_x_angle = x_gyro;
   last_gyro_y_angle = y_gyro;
   last_gyro_z_angle = z_gyro;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
	 void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	 RawData_Def myAccelRaw, myGyroRaw;                           
 	 ScaledData_Def myAccelScaled, myGyroScaled;                  
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;                 
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	Status = HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDR<<1, 10, 100);               //Kontrola komunikace STM s MPU
	HAL_UART_Transmit(&huart2, (uint8_t *)"MPU6050 is ready\r\n", 18, 100);     //"potvrzeni" ze UART jede - v Term. se obj. pouze pokud je nejd. zap. Term. a pot. STM
	MPU6050_Init(&hi2c1); 
	
	//2. Configure Accel and Gyro parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_16g;      //nastaveni citlivosti Accel na 4g
	myMpuConfig.ClockSource = Internal_8MHz;        //zdroj hodin interni - 8MHz
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;    //Filtr 
	myMpuConfig.Gyro_Full_Scale = FS_SEL_2000;       //Nastaveni citlivosti Gyro na 500
	myMpuConfig.Sleep_Mode_Bit = 0;                 //1: sleep mode, 0: normal mode
	MPU6050_Config(&myMpuConfig);
	set_last_read_angle_data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	gyro_angle_x = 0;
	gyro_angle_y = 0;
	gyro_angle_z = 0;
	last_gyro_x_angle = 0;
	last_gyro_y_angle = 0;
	last_gyro_z_angle = 0;
//	accel_vel_x = 0;
//	accel_vel_y = 0;
//	accel_vel_z = 0;
//	accel_pos_x = 0;
//	accel_pos_y = 0;
//	accel_pos_z = 0;
//	last_accel_vel_x = 0;
//	last_accel_vel_y = 0;
//	last_accel_vel_z = 0;
//	last_accel_pos_x = 0;
//	last_accel_pos_y = 0;
//	last_accel_pos_z = 0;
//	prumer = 0;
  /* USER CODE END 2 */
 


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	MPU6050_Cali();
  while (1)
  {				
		HAL_Delay(2);
		Prumer_hodnot();
		Cali_Podminky();
		
		dt =(t_now -  last_read_time)/1000.0; //delí se kvuli "prechodu" na mikro sec.	
		
		gyro_angle_x = gyro_angle_x + (((gyrox + last_gyro_x_angle)*dt)/2);
    gyro_angle_y = gyro_angle_y + (((gyroy+ last_gyro_y_angle)*dt)/2);
		gyro_angle_z = gyro_angle_z + (((gyroz + last_gyro_z_angle)*dt)/2);

//		Accel_samples_x [sample_count]= accelerationx;
//		sample_count = (sample_count+1)%count;
//		
//		for (uint16_t i = 0; i <count; i++)
//		{
//			prumer = prumer + Accel_samples_x [sample_count];
//		}
//	
//		prumer = prumer / count;
//		//if (prumer == 0)
//			//accel_vel_x = 0;
// 		

//		accel_vel_x = accel_vel_x + (((accelerationx + last_accel_vel_x)*dt)/2);
//		if (accel_vel_x !=0)
//		accel_pos_x = accel_pos_x + (((accel_vel_x + last_accel_pos_x)*dt)/2);
//		
//		accel_vel_y = accel_vel_y + (((accelerationy + last_accel_vel_y)*dt)/2);
//		if (accel_vel_y !=0)
//		accel_pos_y = accel_pos_y + (((accel_vel_y + last_accel_pos_y)*dt)/2);
//		
//		accel_vel_z = accel_vel_z + (((accelerationz + last_accel_vel_z)*dt)/2);
//		if (accel_vel_z !=0)
//		accel_pos_z = accel_pos_z + (((accel_vel_z + last_accel_pos_z)*dt)/2);
			
		set_last_read_angle_data(t_now, accelerationx, accelerationy, accelerationz, accel_vel_x, accel_vel_y, accel_vel_z, gyrox, gyroy, gyroz);
				
		size = sprintf((char *)Data,"Data:%.2f:%.2f:%.2f\r\n", gyro_angle_x, gyro_angle_y, gyro_angle_z);      
		
		HAL_UART_Transmit(&huart2, Data, size, 100);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(10);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
