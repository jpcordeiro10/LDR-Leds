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
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <math.h>

#define 	DELAY_LEDS								50
#define 	DELAY_LEDS_DURANTE_O_TESTE				100
#define 	DELAY_ENTRE_LEDS						3000
#define 	DELAY_ENTRE_LEDS_DURANTE_O_TESTE		300
#define 	ON										1
#define 	OFF										0
#define		TempoResultado							5000
#define 	TempoTeste        						3000 			// Representa quantos segundos o teste fará de leitura (5 segundos)
#define 	QtdLeitura       						50	 			// Representa quantas leituras serao feitas
#define		numeroMedidas 							50				// Baseado na amostragem com tempo de teste
#define 	padraoLDR								240			// Valor medio do ADC do LDR (alterar conforme valores padroes medidos na tampa e num ambiente escuro)
#define 	Aprovado								1			
#define 	Reprovado								0

ADC_HandleTypeDef hadc1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

int i = 0;
int lock_proc = 0;
int lock_test = 0;
int led1_state = 0;
int led2_state = 0;
int testePronto = 0;
int Resultado;
int TestePronto = 0;
int LockLedResultado;
float valor_bot = 0;
float leituraLDR;
float delayLeitura = (TempoTeste/QtdLeitura); // Amostragem Ts
float	Tolerancia = 0.1; 		// Tolerancia de 10%
float LimiteSuperior = 0;
float LimiteInferior = 0;


uint32_t tempoLeitura = 0;
uint32_t adc_value = 0;
uint32_t voltage;
uint32_t millis, millis_teste, millis_resultado = 0;
uint32_t leituraAnterior = 0;
uint32_t Acumulador = 0;
uint32_t MediaLDR = 0;
uint32_t delayPiscada = 0;
uint32_t delayPiscadaDuranteTeste = 0;
uint32_t tempo_desligar_led1 = 0;
uint32_t tempo_desligar_led2 = 0;
uint32_t tempo_desligar_led1_dt = 0;
uint32_t tempo_desligar_led2_dt = 0;
uint32_t delayTeste = 0;
uint32_t delayResultado = 0;

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
	
LimiteSuperior = (padraoLDR + (padraoLDR * Tolerancia));
LimiteInferior = (padraoLDR - (padraoLDR * Tolerancia));
  while (1)
  {
		millis = HAL_GetTick();
		valor_bot = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12); // Leitura do botao
		// rotina para piscar os leds em stand by (valid)
		if(millis > delayPiscada && led1_state == OFF && lock_proc == 0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			led1_state = ON;
			tempo_desligar_led1 = millis + DELAY_LEDS;
			lock_proc = 1;
			delayPiscada = millis + DELAY_ENTRE_LEDS;
		}else if(millis > tempo_desligar_led1 && led1_state == ON){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			led1_state = OFF;
			tempo_desligar_led2 = millis + DELAY_LEDS;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2	, GPIO_PIN_SET);
			led2_state = ON;
			lock_proc = 2;
		}else if(millis > tempo_desligar_led2 && led1_state == OFF){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2	, GPIO_PIN_RESET);
			led2_state = OFF;
			lock_proc = 0;
		}
		if(valor_bot){
			lock_test = 1; // Travar as rotinas enquanto o teste é feito
		}
		while(lock_test){
			millis_teste = HAL_GetTick();
			delayTeste = millis + TempoTeste;
			// ------------------------------------------------
			if(millis_teste > delayPiscadaDuranteTeste && led1_state == OFF && lock_proc == 0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			led1_state = ON;
			tempo_desligar_led1_dt = millis_teste + DELAY_LEDS_DURANTE_O_TESTE;
			lock_proc = 1;
			delayPiscadaDuranteTeste = millis_teste + DELAY_ENTRE_LEDS_DURANTE_O_TESTE;
		}else if(millis_teste > tempo_desligar_led1_dt && led1_state == ON){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			led1_state = OFF;
			tempo_desligar_led2_dt = millis_teste + DELAY_LEDS_DURANTE_O_TESTE;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2	, GPIO_PIN_SET);
			led2_state = ON;
			lock_proc = 2;
		}else if(millis_teste > tempo_desligar_led2_dt && led1_state == OFF){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2	, GPIO_PIN_RESET);
			led2_state = OFF;
			lock_proc = 0;
		}
		// piscar os leds durante o teste
			
			// Leitura ADC
			// são 5 segundos de teste, durante o teste pegar amostras a cada 0,1s e realizar a média com a quantidade
			// Realizar leituras a cada 0,1 segundos e acumular 10
			if(millis_teste >= tempoLeitura){		
			// ---------------Leitura do LDR-----------------------------------------
				HAL_ADC_Start(&hadc1);
				if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
					adc_value = HAL_ADC_GetValue(&hadc1);
				}
				HAL_ADC_Stop(&hadc1);
				Acumulador += adc_value; // Acumula o valor do ADC	
			// ------------------------------------------------------------
				tempoLeitura = millis_teste + delayLeitura;
				//tempoLeitura = millis + 1000;
				i++;
			}
			
			if(i >= numeroMedidas){ // Se i alcançou 50 leituras quer dizer que se passaram 5 segundos
				i = 0; // Zera o contador
				MediaLDR = 0;
				MediaLDR = Acumulador / numeroMedidas;	
				Acumulador = 0; // Zera o acumulador
				
			//}
			// --------------------
			
			//if(millis_teste > delayTeste){ // após 5 segundos sai do while
				lock_test = 0;
				delayResultado = millis_teste + TempoResultado;
				millis_teste = 0;
				testePronto = 1;
				LockLedResultado = 0;
				if(MediaLDR > LimiteInferior && MediaLDR < LimiteSuperior){
					Resultado = Aprovado;
				} else {
					Resultado = Reprovado;
				}
			}
		}
		
		while(testePronto){ // Piscar o led de resultado do teste durante 3 segundos 
			// Verde aprovado
			// Vermelho reprovado
			millis_resultado = HAL_GetTick();
			
			
			if(!Resultado && !LockLedResultado){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
				LockLedResultado = 1;
			} 
			if(Resultado && !LockLedResultado){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
				LockLedResultado = 1;
			}
			
			if(millis_resultado > delayResultado){
				testePronto = 0;
				millis_resultado = 0;
				LockLedResultado = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			}
		}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
