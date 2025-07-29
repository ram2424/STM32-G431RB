/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @brief          : Cooperative Scheduler Implementation
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
	* Features:
  * 				- 3 Tasks: LED Blink, Button Handler, Idle
  * 				- 10ms system tick using SysTick
  * 				- Round-robin scheduling with yield points
  * 				- Button interrupt for immediate response
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Task States
typedef enum{
	TASK_READY,
	TASK_RUNNING,
	TASK_BLOCKED,
	TASK_SUSPENDED
}Taskstate_t;

//Task Control Block TCB_t
typedef struct{
	uint8_t task_id;
	Taskstate_t state;
	uint8_t priority;
	uint32_t stack_pointer;
	void (*task_function)(void);
	uint32_t wake_time;
	char task_name[16];
}TCB_t;

// Scheduler Stats
typedef struct {
    uint32_t task_switches;
    uint32_t system_ticks;
    uint8_t current_task;  
} SchedulerStats_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_TASKS           3
#define SYSTICK_FREQ_MS     10      // 10ms system tick
#define LED_BLINK_PERIOD    50      // 50 * 10ms = 500ms

// Task IDs
#define TASK_LED_BLINK      0
#define TASK_BUTTON_HANDLER 1  
#define TASK_IDLE           2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

TCB_t tasks[MAX_TASKS];
uint8_t current_task = 0;
uint8_t task_count = 0;

// Scheduler variables
volatile uint32_t system_tick = 0;
volatile uint8_t schedule_flag = 0;
SchedulerStats_t scheduler_stats;

// Application variables
volatile uint8_t button_pressed = 0;
volatile uint8_t blink_enabled = 1;
volatile uint8_t led_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

// Scheduler functions
void Scheduler_Init(void);
void Scheduler_Start(void);
void Scheduler_Tick(void);
void Task_Yield(void);
void Task_Delay(uint32_t delay_ms);

// Task functions
void Task_LED_Blink(void);
void Task_Button_Handler(void);
void Task_Idle(void);

// Utility functions
void Task_Create(uint8_t id, void (*function)(void), uint8_t priority, const char* name);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Initialize the cooperative scheduler
  * @retval None
  */
void Scheduler_Init(void)
{
    // Clear all tasks
    memset(tasks, 0, sizeof(tasks));
    memset(&scheduler_stats, 0, sizeof(scheduler_stats));
    
    task_count = 0;
    current_task = 0;
    
    // Create tasks
    Task_Create(TASK_LED_BLINK, Task_LED_Blink, 2, "LED_Blink");
    Task_Create(TASK_BUTTON_HANDLER, Task_Button_Handler, 1, "Button_Handler");
    Task_Create(TASK_IDLE, Task_Idle, 3, "Idle_Task");
    
    // Configure SysTick for 10ms interrupts
    // SystemCoreClock = 170MHz, we want 100Hz (10ms)
    if (SysTick_Config(SystemCoreClock / 100) != 0) {
        Error_Handler();
    }
    
    // Set SysTick priority (should be lower than button interrupt)
    NVIC_SetPriority(SysTick_IRQn, 1);
}

/**
  * @brief  Create a new task
  * @param  id: Task ID
  * @param  function: Task function pointer
  * @param  priority: Task priority (1=highest, 3=lowest)
  * @param  name: Task name string
  * @retval None
  */
void Task_Create(uint8_t id, void (*function)(void), uint8_t priority, const char* name)
{
    if (id < MAX_TASKS) {
        tasks[id].task_id = id;
        tasks[id].state = TASK_READY;
        tasks[id].priority = priority;
        tasks[id].task_function = function;
        tasks[id].wake_time = 0;
        strncpy(tasks[id].task_name, name, sizeof(tasks[id].task_name) - 1);
        
        if (id >= task_count) {
            task_count = id + 1;
        }
    }
}

/**
  * @brief  Start the scheduler (never returns)
  * @retval None
  */
void Scheduler_Start(void)
{
    while (1) {
        // Wait for scheduler tick
        if (schedule_flag) {
            schedule_flag = 0;
            Scheduler_Tick();
        }
        
        // Run current task
        if (tasks[current_task].state == TASK_READY) {
            tasks[current_task].state = TASK_RUNNING;
            tasks[current_task].task_function();
            tasks[current_task].state = TASK_READY;
        }
        
        // Simple round-robin scheduling
        current_task = (current_task + 1) % task_count;
        scheduler_stats.task_switches++;
    }
}

/**
  * @brief  Scheduler tick handler (called every 10ms)
  * @retval None
  */
void Scheduler_Tick(void)
{
    system_tick++;
    scheduler_stats.system_ticks++;
    
    // Wake up delayed tasks
    for (uint8_t i = 0; i < task_count; i++) {
        if (tasks[i].state == TASK_BLOCKED && 
            tasks[i].wake_time <= system_tick) {
            tasks[i].state = TASK_READY;
            tasks[i].wake_time = 0;
        }
    }
}

/**
  * @brief  Cooperative task yield (voluntarily give up CPU)
  * @retval None
  */
void Task_Yield(void)
{
    // In cooperative scheduler, this just returns
    // The main scheduler loop will switch to next task
    return;
}

/**
  * @brief  Delay current task for specified milliseconds
  * @param  delay_ms: Delay in milliseconds
  * @retval None
  */
void Task_Delay(uint32_t delay_ms)
{
    uint32_t delay_ticks = delay_ms / SYSTICK_FREQ_MS;
    if (delay_ticks == 0) delay_ticks = 1;
    
    tasks[current_task].state = TASK_BLOCKED;
    tasks[current_task].wake_time = system_tick + delay_ticks;
}

/**
  * @brief  LED Blink Task - toggles LED every 500ms
  * @retval None
  */
void Task_LED_Blink(void)
{
    static uint32_t led_counter = 0;
    
    led_counter++;
    
    if (led_counter >= LED_BLINK_PERIOD) {  // 50 * 10ms = 500ms
        led_counter = 0;
        
        if (blink_enabled) {
            led_state = !led_state;
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 
                            led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }
    
    // Cooperative yield
    Task_Yield();
}

/**
  * @brief  Button Handler Task - processes button presses
  * @retval None
  */
void Task_Button_Handler(void)
{
    static uint32_t last_press_time = 0;
    
    if (button_pressed) {
        uint32_t current_time = system_tick * SYSTICK_FREQ_MS;
        
        // Debouncing: 200ms between presses
        if ((current_time - last_press_time) > 200) {
            button_pressed = 0;
            
            // Toggle blink enable
            blink_enabled = !blink_enabled;
            
            // If disabled, turn off LED
            if (!blink_enabled) {
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
                led_state = 0;
            }
            
            last_press_time = current_time;
        } else {
            button_pressed = 0;  // Ignore bounced press
        }
    }
    
    // Cooperative yield
    Task_Yield();
}

/**
  * @brief  Idle Task - runs when no other tasks are ready
  * @retval None
  */
void Task_Idle(void)
{
    // CPU usage monitoring or low power mode
    static uint32_t idle_counter = 0;
    idle_counter++;
    
    // Could add sleep mode here
    // __WFI();  // Wait for interrupt
    
    // Cooperative yield
    Task_Yield();
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
  /* USER CODE BEGIN 2 */
	
	// Initialize the scheduler
	Scheduler_Init();
	
	// Initialize LED state
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	
	Scheduler_Start();

  /* USER CODE END 2 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
