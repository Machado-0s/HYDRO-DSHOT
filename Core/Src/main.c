/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.

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
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "dshot.h"
#include <string.h>
#include <stdio.h>
#include "pwm.h"
#include "usart.h"
#include "uart_cmd.h" // For UART command functions and variables


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void Error_Handler(void);
/* USER CODE BEGIN PFP */
// Add this function somewhere globally

// Redirect printf to software UART

// Add DWT delay utility at the top of main.c:
#define DWT_CYCCNT ((volatile uint32_t *)0xE0001004)
void DWT_Delay(uint32_t microseconds) {
    uint32_t start = *DWT_CYCCNT;
    uint32_t cycles = microseconds * (SystemCoreClock / 1000000);
    while((*DWT_CYCCNT - start) < cycles);
}

static inline void short_delay_us(uint32_t us)
{
    volatile uint32_t count = us * (84); // Roughly 1 cycle per loop at 84 MHz
    while (count--) {
        __NOP();
    }
}

/* USER CODE END PFP */
volatile uint16_t current_motor_rpms[MOTORS_COUNT];
extern UART_HandleTypeDef huart1;
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Define global flags (no extern here)
volatile uint8_t dshot_send_flag = 0;
volatile bool uart_tx_ready = true; // TX flag

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
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  UART_CMD_Init(&huart1); // Pass your UART handle
  PWM_Init();



  // --- DShot Initialization ---
  setup_Dshot_Tx_Only();
  preset_bb_Dshot_buffers();

  // Initialize all motor target RPMs to 0.0 (stop)
    for (int i = 0; i < MOTORS_COUNT; i++) {
        pid_target_speed_rpms[i] = 1000.0f; // All motors initially stopped
    }

      for (int i = 0; i < MOTORS_COUNT; i++) {
           motor_values[i] = prepare_Dshot_package(0, false); // Send 0 throttle (disarmed)
       }

  // Send this 0 throttle for 200ms
  uint32_t calibration_start_time = HAL_GetTick();
  while (HAL_GetTick() - calibration_start_time < 2000) {
      update_motors_Tx_Only();
      // Keep the small delay to ensure signal integrity during calibration phase too
      for (volatile int i = 0; i < 100; i++);
  }


  for (int i = 0; i < MOTORS_COUNT; i++) {
	  motor_values[i] = prepare_Dshot_package(10, true); // Send 0 throttle (disarmed)
     }

   for (int t = 0; t < 10; t++){
   update_motors_Tx_Only();
   }

   Debug_Send_DMA("--- STM32 DShot Controller Started ---\r\n");



  /* Infinite loop */
     /* USER CODE BEGIN WHILE */
     //uint32_t last_servo_time = HAL_GetTick();      // For 20ms servo updates
     uint32_t last_50hz_time = 0;
     uint32_t last_100hz_time = 0;
     uint32_t now2 = HAL_GetTick();


     uint32_t last_telemetry_timestamp = 0;
     bool telemetry_active = false; // Start with telemetry inactive until first successful read
     uint16_t current_rpm = 0;

     uint32_t last_periodic_print_time = HAL_GetTick();
      const uint32_t PERIODIC_PRINT_INTERVAL_MS = 100; // 5 seconds
     float  lastTarget[10] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

  /* Infinite loop */
  while (1) {


	  // Check if new telemetry data is ready
	      if (telemetry_done_flag){
	          // Clear the flag immediately to prepare for the next cycle
	          telemetry_done_flag = 0;
	          last_telemetry_timestamp = HAL_GetTick(); // Update timestamp on success

	          // Step 1: Process the telemetry responses from the previous cycle
	          process_telemetry_with_new_method();


	          // Step 3: Loop through each motor to calculate a new command using PID
	          for (int m = 0; m < MOTORS_COUNT; m++){
	             // uint16_t current_rpm = 0;

	        	  // Use received_numeric_value from UART as the PID target RPM
	        	   float pid_target_rpm_from_uart = pid_target_speed_rpms[m];

	              if (motor_telemetry_data[m].valid_rpm) {
	                  current_rpm = motor_telemetry_data[m].raw_rpm_value;
	              }else {
	            	  //current_rpm_for_pid = (uint32_t)fabsf(current_pid_target);
	              }
	              //if(current_rpm < 800 || current_rpm == 0)current_rpm = pid_target_speed_rpm ;
	             // uint16_t new_command = pid_calculate_command(current_rpm);
	              // Calculate PID command. Pass the current RPM and the target RPM from UART.
	               uint16_t new_command = pid_calculate_command(current_rpm, pid_target_rpm_from_uart);
	              motor_values[m] = prepare_Dshot_package(new_command, false);


	             // Debug_Send_DMA("VALUE: %d %d \r\n",m, new_command);



	          }

	          // Step 4: Send the new DShot commands and start the next telemetry reception
	          for (volatile int i = 0; i < 100; i++);
	          update_motors_Tx_Only();

	          telemetry_active = true;
	      }

	      // Add a check for telemetry timeout
	      if (telemetry_active && (HAL_GetTick() - last_telemetry_timestamp > 2000)) {
	          // Telemetry has failed, enter a safe open-loop mode
	          telemetry_active = false;

	          // For all motors, set a safe, constant throttle
	          for (int m = 0; m < MOTORS_COUNT; m++) {
	              motor_values[m] = prepare_Dshot_package(DSHOT_BASE_COMMAND, false);

	             // Debug_Send_DMA("FAILED: %d %d %d",m,motor_telemetry_data[m].raw_rpm_value , motor_telemetry_data[m].voltage);
	          }
	          for (volatile int i = 0; i < 100; i++);
	          update_motors_Tx_Only();
	      }


	          // --- Periodic RPM and DShot Command Output ---
	             if ((HAL_GetTick() - last_periodic_print_time) >= PERIODIC_PRINT_INTERVAL_MS ) {
	                 last_periodic_print_time = HAL_GetTick(); // Reset the timer

	                 //Debug_Send_DMA("--- Current Motor Status (%lu ms) ---\r\n", HAL_GetTick());
	                 for (int m = 0; m < MOTORS_COUNT; m++) {
	                	 if(lastTarget[m] != pid_target_speed_rpms[m]){
	                     float current_target_rpm = pid_target_speed_rpms[m];
	                     uint32_t current_telemetry_rpm = motor_telemetry_data[m].raw_rpm_value;
	                     float current_physical_rpm = current_telemetry_rpm / (14.0f / 2.0f); // Assuming 14 poles / 7 pole pairs
	                     uint16_t last_dshot_command_sent = (uint16_t)(motor_values[m] >> 4); // Extract throttle from DShot package

	                     Debug_Send_DMA("Motor: %d | Tgt RPM: %.0f | Curr ERPM: %lu | Curr Phys RPM: %.2f | Last DShot Cmd: %u | Valid Telemetry: %s\r\n",
	                                    m,
	                                    current_target_rpm,
	                                    current_telemetry_rpm,
	                                    current_physical_rpm,
	                                    last_dshot_command_sent,
	                                    motor_telemetry_data[m].valid_rpm ? "Yes" : "No");
	                 }
	                // Debug_Send_DMA("---\r\n");
	                 lastTarget[m] = pid_target_speed_rpms[m];
	                 }
	             }

	             // Check if new UART data is available and process it
	         	          if (uart_new_data_available) {
	         	              process_uart_command(); // This will parse and update received_numeric_value
	         	          }


	         	         // ---- SERVO PWM UPDATE - Different frequencies
	         	         now2 = HAL_GetTick();

	         	         // Update 50Hz motors (every 20ms)
	         	         if (now2 - last_50hz_time >= 20) {
	         	             static uint16_t angle_50hz = 500;
	         	             angle_50hz += 5;
	         	             if (angle_50hz > 2500) angle_50hz = 500;

	         	             PWM_SetDuty(&htim4, TIM_CHANNEL_1, angle_50hz); // PB6 - 50Hz
	         	             PWM_SetDuty(&htim4, TIM_CHANNEL_2, angle_50hz); // PB7 - 50Hz

	         	             last_50hz_time = now2;
	         	         }

	         	         // Update 100Hz motors (every 10ms)
	         	         if (now2 - last_100hz_time >= 10) {
	         	             static uint16_t angle_100hz = 500;
	         	             angle_100hz += 5;
	         	             if (angle_100hz > 2500) angle_100hz = 500;

	         	             PWM_SetDuty(&htim3, TIM_CHANNEL_1, angle_100hz); // e.g., PB4 - 100Hz
	         	             PWM_SetDuty(&htim3, TIM_CHANNEL_2, angle_100hz); // e.g., PB5 - 100Hz

	         	             last_100hz_time = now2;
	         	         }

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
  }





// --- UART Transmit Complete Callback ---
// This function is called by HAL when a UART DMA transmission is complete.
// It sets the uart_tx_ready flag back to true, allowing the next Debug_Send_DMA call.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // Check if it's our USART1
    {
        uart_tx_ready = true; // Mark TX buffer as ready for next transmission
    }

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
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 25;
	  RCC_OscInitStruct.PLL.PLLN = 168;
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
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
