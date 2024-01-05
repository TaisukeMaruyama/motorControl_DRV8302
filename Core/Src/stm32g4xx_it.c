/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GlogalVariables.h"
#include "SixsStep.h"
#include "VectorControl.h"
#include "SignalReadWrite.h"
#include "GeneralFunctions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_dac1_ch1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac1_ch1);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
  int8_t rotDir;
	float ErectFreqRef = 100.0f;
	float ErectFreqErr;
	uint8_t voltageMode_tmp;
	float theta_tmp;
	float electAngVelo_tmp;
	float Idq_ref[2];
	uint8_t leadAngleModeFlg;
	uint8_t flgFB;
  int8_t outputMode[3];
  
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	//read IO signals
	gButton1 = readButton1();
	gVolume = readVolume();
	readCurrent(gIuvw_AD, gIuvw);
	gVdc = 12.0f;//readVdc();
	gTwoDivVdc = gfDivideAvoidZero(2.0f, gVdc, 1.0f);

	// Sequence Control
	
	//DutyRef Calculation
	//if ( gButton1 == 1 )
	  rotDir = 1;
	//else
	//  rotDir = -1;


	  Idq_ref[0] = 0.0f; //gVolume * 2;//-0.0f;//gVolume;//0.05f;
    // Idq_ref[0] = -1.0f * (gPropoDuty * 10); //field weak
	  Idq_ref[1] = 10.0f * gPropoDuty;
	/*// Speed Control
	ErectFreqRef = 200.0f * gVolume;
	ErectFreqErr = ErectFreqRef - gElectFreq;
	gDutyRef += ErectFreqErr * 0.0000001f;
	*/

	// Sequence Control
	if(gInitCnt < 500){
		gInitCnt++;
		gPosMode = POSMODE_HALL;
		gDrvMode = DRVMODE_OFF;
		leadAngleModeFlg = 0;
		flgFB = 0;
	}
	else if (gElectFreq < 10.0f){
		gPosMode = POSMODE_HALL;
		gDrvMode = DRVMODE_OPENLOOP;
		leadAngleModeFlg = 0;
		flgFB = 0;
	}
	else if(gElectFreq < 20.0f){
		gPosMode = POSMODE_HALL_PLL;
		gDrvMode = DRVMODE_OPENLOOP;
		leadAngleModeFlg = 1;
		flgFB = 0;
	}
	else{
		gPosMode = POSMODE_HALL_PLL;
		gDrvMode = DRVMODE_VECTORCONTROL;
		leadAngleModeFlg = 1;
		flgFB = 1;
	}

	// MotorDrive
	if(gDrvMode == DRVMODE_OFF){
		outputMode[0] = OUTPUTMODE_OPEN;
		outputMode[1] = OUTPUTMODE_OPEN;
		outputMode[2] = OUTPUTMODE_OPEN;
		gDuty[0] = 0.0f;
		gDuty[1] = 0.0f;
		gDuty[2] = 0.0f;

	}
	else{
		gDutyRef = 0.0f;
		//sixStepTasks(gDutyRef, leadAngleModeFlg, 0.0f, &theta_tmp, &electAngVelo_tmp, gDuty, outputMode);
		calcElectAngle(leadAngleModeFlg, &voltageMode_tmp, &theta_tmp, &electAngVelo_tmp);
		gTheta = theta_tmp;
		gElectAngVelo = electAngVelo_tmp;
		//gTheta_DAC = 1000;//(gTheta + PI) * ONEDIVTWOPI * 4096;

		//write IO signals
		//gTheta = gTheta + 100.0f * CARRIERCYCLE;
		//gTheta = gfWrapTheta(gTheta);
		VectorControlTasks(Idq_ref, gTheta, gElectAngVelo, gIuvw, gVdc, gTwoDivVdc, flgFB, gDuty, outputMode);
		//OpenLoopTasks(1.5f, gTheta, gIuvw, gTwoDivVdc, gDuty, outputMode);
	}

	writeOutputMode(outputMode);
	writeDuty(gDuty);




	//if ( gButton1 == 0 )
	//	OpenLoopTasks(gDutyRef * 8.0f, gTheta, gIuvw, gTwoDivVdc, gDuty, outputMode);
	//else

//
//VectorControlTasks(Idq_ref, gTheta, gIuvw, gVdc, gDuty);



	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
