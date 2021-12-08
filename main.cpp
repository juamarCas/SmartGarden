/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "includes.h"
#include "SHTC3.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
//turn this define to 0 when you want to upload the final code
#define DEBUG 1

#if DEBUG
#include "UART2.h"
#endif

#if DEBUG
#endif

//buffer length is 2 times dma adc buffer legnth due to each DMA_ADC data consist od two bytes of data
#define DMA_ADC_BUFFER_LENGTH 2
#define BUFFER_LENGTH DMA_ADC_BUFFER_LENGTH * 2

/*sensors information*/

#define READ_ADDR            (0x70U << 1) | 0x01
#define START_ADC_CONVERTION ADC1->CR |= ADC_CR_ADSTART
#define STOP_ADC_CONVERTION  ADC1->CR |= ADC_CR_ADSTP
#define SHCT_ADDR            (0x70U << 1)

volatile int counter = 0;

/*Payload contains all the sensor information*/
typedef struct Payloads{
	std::uint16_t moist_1;
	std::uint16_t moist_2;
	float         temp;
	float         hum;
}Payload;


/*each state measures some variables depending of teh sensor*/
typedef struct SM_State{
	void (*state_action_ptr)(void);
    struct SM_State * next_state[2];
}SM_States_TypeDef;

/*Measurement functions*/
void Measure_Temp_Hum(void);
void Measure_Moist(void);
void Init_Timer(void);

/*esp related functions*/
void init_esp(void);
void Send_Data(void);


enum States {TempHum = 0, Moist , Send, Sleep, WaitingESP ,Error};

States state;
Payload payload;

SM_States_TypeDef sm_states[4] = {
		{
				&Measure_Temp_Hum,
				{&sm_states[Moist], &sm_states[Moist]} //go to measure moist
		},
		{
				&Measure_Moist,
				{&sm_states[Send], &sm_states[Send]} // go to send data
		},
		{
				&Send_Data,
				{&sm_states[Sleep], &sm_states[Sleep]} // start over
		},
		{
				&Init_Timer,
				{&sm_states[TempHum], &sm_states[TempHum]}
		}
};


SM_States_TypeDef * state_ptr;

/* USER CODE END PD */

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
#if DEBUG
/*debugging variables*/
volatile std::uint16_t data1 = 0;
volatile std::uint16_t data2 = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Initializate_UART3();
void Initializate_ADC1();
void Initializate_UART2();
void Initializate_TIMER4();
void Initializate_Peripherals();


/*
	separate a variable in two (16 bit variable into two 8 bit spaces)
	@param buffer, array of bytes that are going to be sent
	@param value, variable to be separated
	@param offset, starting position of the buffer you want to place your value
*/
void assemble_buffer(std::uint8_t * buffer, std::uint16_t value,std::uint8_t offset);

/*Interrupt functions*/
std::uint16_t moist1;
std::uint16_t moist2;
std::uint16_t adc_dma_buffer[DMA_ADC_BUFFER_LENGTH] = {0, 0};
std::uint8_t buffer[BUFFER_LENGTH] = {0};

SHTC3 shtc(&hi2c1);

/*variable to control the change between states*/
bool passNextState = true;

extern "C"{
	void DMA1_Channel1_IRQHandler(void){

		if(DMA1->ISR & DMA_ISR_TCIF1){
			payload.moist_1 = adc_dma_buffer[0];
			payload.moist_2 = adc_dma_buffer[1];
			STOP_ADC_CONVERTION;
			passNextState = true;
			DMA1->IFCR |= DMA_IFCR_CTCIF1;
			GPIOA->ODR ^= (1 << 5);
		}

	}

}



/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	//__WFI();
  /* USER CODE BEGIN 1 */
	__disable_irq();
  /* USER CODE END 1 */

  HAL_Init();
#if DEBUG
  /*this methods of UART2 are hardcoded UART in order to use printf function*/
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  UART2_init();
  printf("initializing system...\r\n");
#endif
  utils::delay::Init();
  /* USER CODE BEGIN Init */
  /*configure clock for ADC*/
  	RCC->CR |= RCC_CR_PLLON;
  	RCC->CFGR2 |= (16U << 4U);

  	/*Peripheral activation*/
  	Initializate_Peripherals();

  	periph::GPIO::set_pin(GPIOA, 5, OUTPUT);

  	Initializate_UART3();
  	Initializate_ADC1();


  	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


   if(!shtc.begin()){
	   GPIOA->ODR |= (1 << 5);
   }
   state = TempHum;
   utils::delay::ms(2500);

  __enable_irq();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  state_ptr = &sm_states[state];
  while (1)
  {
    /* USER CODE END WHILE */
	  (state_ptr->state_action_ptr)();
	  while(!(passNextState)){}
	  state_ptr = state_ptr->next_state[0];

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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


  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
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


}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void Initializate_Peripherals(){
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
	RCC->AHBENR  |= RCC_AHBENR_ADC12EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

void Initializate_ADC1(){

	periph::GPIO::set_pin(GPIOA, 0, ANALOG);
	periph::GPIO::set_pin(GPIOA, 1, ANALOG);
	/*configuring dma for the ADC*/
	DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 ;
	//inside the adc DMA function it activates the sha channel
	//Configure length and channels
	DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;

	//length of convertions
	ADC1->SQR1 = 0;
	ADC1->SQR1 |= ADC_LENGTH_2;

	periph::ADC::SetChannelSequence(ADC1_SQR1, ADC_CH_1, ADC_SQ1_1);
	periph::ADC::SetChannelSequence(ADC1_SQR1, ADC_CH_2, ADC_SQ1_2);
	periph::ADC::DMA_Init(ADC1, DMA1_Channel1, adc_dma_buffer, DMA_ADC_BUFFER_LENGTH,ADC_LENGTH_2);


}

void Initializate_UART3(){
	periph::GPIO::set_pin(GPIOB, 10, ALTERNATE);
	periph::GPIO::set_pin(GPIOB, 11, ALTERNATE);
	periph::GPIO::set_afrh(GPIOB, 7U, 8U);
	periph::GPIO::set_afrh(GPIOB, 7U, 12U);

	periph::UART::Init(USART3, 9600, 8000000);
}

/*Measurement functions*/
void Measure_Temp_Hum(void){
	 if(!shtc.Read_sensor(payload.temp, payload.hum, SHTC3_NORMAL_MEASUREMENT_CMD )){
			  GPIOA->ODR |= (1 << 5);
	 }
}

void Measure_Moist(void){
	passNextState = false;
	START_ADC_CONVERTION;
}

/*esp related functions*/
void init_esp(void){

}

void Init_Timer(void){
	/*temporal implementation*/
	utils::delay::ms(2500);
}

void Send_Data(void){
#if DEBUG
	int temp = static_cast<int>(payload.temp);
	int hum = static_cast<int>(payload.hum);
	printf("tempt: %d, hum: %d, moist1: %d, moist2: %d\r\n", temp, hum, payload.moist_1, payload.moist_2);
#endif
}

//as you can clearly see, only works with two bytes (16 bits) variables :*
void assemble_buffer(std::uint8_t * buffer, std::uint16_t value, std::uint8_t offset){
	std::uint8_t lsb = value & 0xFFU;
	std::uint8_t msb = value >> 8U;
	buffer[offset] = lsb;
	buffer[offset + 1] = msb;
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
