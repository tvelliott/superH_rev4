/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32h7xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_spi2_tx;
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit( void )
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit( ADC_HandleTypeDef *hadc )
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if( hadc->Instance == ADC1 ) {
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if( HAL_RCC_ADC12_CLK_ENABLED == 1 ) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PF11     ------> ADC1_INP2
    PF12     ------> ADC1_INN2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOF, &GPIO_InitStruct );

    hdma_adc1.Instance = DMA1_Stream0;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if( HAL_DMA_Init( &hdma_adc1 ) != HAL_OK ) {
      _Error_Handler( __FILE__, __LINE__ );
    }

    __HAL_LINKDMA( hadc, DMA_Handle, hdma_adc1 );

    HAL_NVIC_SetPriority( ADC_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( ADC_IRQn );
  } else if( hadc->Instance == ADC2 ) {
    HAL_RCC_ADC12_CLK_ENABLED++;
    if( HAL_RCC_ADC12_CLK_ENABLED == 1 ) {
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOF, &GPIO_InitStruct );



    HAL_NVIC_SetPriority( ADC_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( ADC_IRQn );

  } else if( hadc->Instance == ADC3 ) {
    /* Peripheral clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PF10     ------> ADC3_INP6
    PC2_C     ------> ADC3_INN1
    PC2_C     ------> ADC3_INP0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOF, &GPIO_InitStruct );

    HAL_SYSCFG_AnalogSwitchConfig( SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_OPEN );

    /* ADC3 DMA Init */
    /* ADC3 Init */
    hdma_adc3.Instance = DMA2_Stream0;
    hdma_adc3.Init.Request = DMA_REQUEST_ADC3;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if( HAL_DMA_Init( &hdma_adc3 ) != HAL_OK ) {
      _Error_Handler( __FILE__, __LINE__ );
    }

    __HAL_LINKDMA( hadc, DMA_Handle, hdma_adc3 );

    /* ADC3 interrupt Init */
    HAL_NVIC_SetPriority( ADC3_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( ADC3_IRQn );

    /* USER CODE END ADC3_MspInit 1 */
  }

}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit( ADC_HandleTypeDef *hadc )
{
  if( hadc->Instance == ADC1 ) {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if( HAL_RCC_ADC12_CLK_ENABLED == 0 ) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PF11     ------> ADC1_INP2
    PF12     ------> ADC1_INN2
    */
    HAL_GPIO_DeInit( GPIOF, GPIO_PIN_11 | GPIO_PIN_12 );

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit( hadc->DMA_Handle );

    /* ADC1 interrupt DeInit */
    /* USER CODE BEGIN ADC1:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
    /* USER CODE END ADC1:ADC_IRQn disable */

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  } else if( hadc->Instance == ADC2 ) {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */

    /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if( HAL_RCC_ADC12_CLK_ENABLED == 0 ) {
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PF13     ------> ADC2_INP2
    PF14     ------> ADC2_INN2
    */
    HAL_GPIO_DeInit( GPIOF, GPIO_PIN_13 | GPIO_PIN_14 );

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit( hadc->DMA_Handle );

    HAL_NVIC_DisableIRQ( ADC_IRQn );

    /* ADC2 interrupt DeInit */
    /* USER CODE BEGIN ADC2:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
    /* USER CODE END ADC2:ADC_IRQn disable */

    /* USER CODE BEGIN ADC2_MspDeInit 1 */

    /* USER CODE END ADC2_MspDeInit 1 */
  } else if( hadc->Instance == ADC3 ) {
    /* USER CODE BEGIN ADC3_MspDeInit 0 */

    /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PF10     ------> ADC3_INP6
    PC2_C     ------> ADC3_INN1
    PC2_C     ------> ADC3_INP0
    */
    HAL_GPIO_DeInit( GPIOF, GPIO_PIN_10 );

    HAL_DMA_DeInit( hadc->DMA_Handle );

    HAL_NVIC_DisableIRQ( ADC3_IRQn );

    /* USER CODE BEGIN ADC3_MspDeInit 1 */

    /* USER CODE END ADC3_MspDeInit 1 */
  }

}
/**
* @brief I2S MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2s: I2S handle pointer
* @retval None
*/
void HAL_I2S_MspInit( I2S_HandleTypeDef *hi2s )
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if( hi2s->Instance == SPI2 ) {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**I2S2 GPIO Configuration
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB15     ------> I2S2_SDO
    PC6     ------> I2S2_MCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

    /* I2S2 DMA Init */
    /* SPI2_TX Init */
    hdma_spi2_tx.Instance = DMA1_Stream1;
    hdma_spi2_tx.Init.Request = DMA_REQUEST_SPI2_TX;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if( HAL_DMA_Init( &hdma_spi2_tx ) != HAL_OK ) {
      _Error_Handler( __FILE__, __LINE__ );
    }

    __HAL_LINKDMA( hi2s, hdmatx, hdma_spi2_tx );

    /* I2S2 interrupt Init */
    HAL_NVIC_SetPriority( SPI2_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( SPI2_IRQn );


    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */
  }

}

/**
* @brief I2S MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2s: I2S handle pointer
* @retval None
*/
void HAL_I2S_MspDeInit( I2S_HandleTypeDef *hi2s )
{
  if( hi2s->Instance == SPI2 ) {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**I2S2 GPIO Configuration
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB15     ------> I2S2_SDO
    PC6     ------> I2S2_MCK
    */
    HAL_GPIO_DeInit( GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15 );

    HAL_GPIO_DeInit( GPIOC, GPIO_PIN_6 );

    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }

}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit( SPI_HandleTypeDef *hspi )
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if( hspi->Instance == SPI3 ) {
    /* USER CODE BEGIN SPI3_MspInit 0 */

    /* USER CODE END SPI3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

    HAL_NVIC_SetPriority( SPI3_IRQn, 3, 3 );
    HAL_NVIC_EnableIRQ( SPI3_IRQn );

    /* USER CODE BEGIN SPI3_MspInit 1 */

    /* USER CODE END SPI3_MspInit 1 */
  }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit( SPI_HandleTypeDef *hspi )
{
  if( hspi->Instance == SPI3 ) {
    /* USER CODE BEGIN SPI3_MspDeInit 0 */

    /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit( GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 );

    /* USER CODE BEGIN SPI3_MspDeInit 1 */

    /* USER CODE END SPI3_MspDeInit 1 */
  }

}
/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit( TIM_HandleTypeDef *htim_base )
{
  if( htim_base->Instance == TIM2 ) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  } else if( htim_base->Instance == TIM3 ) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority( TIM3_IRQn, 3, 0 );
    HAL_NVIC_EnableIRQ( TIM3_IRQn );
  }

}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit( TIM_HandleTypeDef *htim_base )
{
  if( htim_base->Instance == TIM2 ) {
    __HAL_RCC_TIM2_CLK_DISABLE();
  } else if( htim_base->Instance == TIM3 ) {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ( TIM3_IRQn );
  }

}
