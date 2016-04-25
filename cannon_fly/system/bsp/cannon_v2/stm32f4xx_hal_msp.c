/**
  ******************************************************************************
  * @file    PWR/PWR_CurrentConsumption/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    09-October-2015
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_msp.h"
#include "stm32f4xx_hal.h"
#include "bluenrg_sdk_api.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @defgroup PWR_CurrentConsumption
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

UART_HandleTypeDef UartHandle;
TIM_HandleTypeDef TimHandle;
TIM_OC_InitTypeDef sConfigOC;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_StatusTypeDef status = HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

    if (status != HAL_OK) {
        //while (1);
        return 0;
    }
    return ch;
}


/**
  * @brief  UART初始化函数.
  * @param  None
  * @retval None
  */
void UART_Init(void)
{
    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART1 configured as follow:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = ODD parity
        - BaudRate = 115200 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance          = USARTx;

    UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX ;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	

    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        while (1);
    }
		
#ifdef FLY_CONTROL

		__HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);
	  HAL_NVIC_SetPriority(USART2_IRQn, 0x0D, 0);    
    HAL_NVIC_EnableIRQ(USART2_IRQn);
		
#endif
		
    printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");
}

#ifdef FLY_CONTROL

/**
  * @brief  TIM10初始化函数.(计时中断)
  * @param  None
  * @retval None
  */
void TIM10_Init(void)
{
	
	TimHandle.Instance = TIM10;
	
	TimHandle.Init.Prescaler = 83;
	TimHandle.Init.Period = 4999;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	
	if(HAL_TIM_Base_Init(&TimHandle)!= HAL_OK)
	{
		/* Initialization Error */
		while(1);
	}
	
}

/**
  * @brief  TIM_PWM初始化函数.
  * @param  None
  * @retval None
  */
void TIM_PWM_Init(void)
{
	TIM_HandleTypeDef TimPwmHandle;
	
	
	/************TIM2初始化**************/
	TimPwmHandle.Instance = TIM2;
	
	TimPwmHandle.State = 0;
	TimPwmHandle.Init.Prescaler =6-1;
	TimPwmHandle.Init.Period = 1000-1;
	TimPwmHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	
	if(HAL_TIM_PWM_Init(&TimPwmHandle)!= HAL_OK)
	{
		/* Initialization Error */
		while(1);
	}
	
/************TIM3初始化**************/
	TimPwmHandle.Instance = TIM3;
	
	TimPwmHandle.State = HAL_TIM_STATE_RESET;
	TimPwmHandle.Init.Prescaler = 6-1;
	TimPwmHandle.Init.Period = 1000-1;
	TimPwmHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	
	if(HAL_TIM_PWM_Init(&TimPwmHandle)!= HAL_OK)
	{
		/* Initialization Error */
		while(1);
	}
}

#endif

/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();

    /* Enable USARTx clock */
    USARTx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USARTx_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;

    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USARTx_RX_PIN;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;

    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
		
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    /*##-1- Reset peripherals ##################################################*/
    USARTx_FORCE_RESET();
    USARTx_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /* Configure UART Tx as alternate function  */
    HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
    /* Configure UART Rx as alternate function  */
    HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
	
		HAL_NVIC_DisableIRQ(USART2_IRQn);

}


/**
  * @brief  Initializes the RTC MSP.
  * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
//__weak void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
//{
//  /* NOTE : This function Should not be modified, when the callback is needed,
//            the HAL_RTC_MspInit could be implemented in the user file
//   */
//}

void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
    RCC_OscInitTypeDef        RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

    /*##-1- Configue LSI as RTC clock soucre ###################################*/
#ifdef RTC_LSI
    RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
#endif

#ifdef RTC_LSE
    RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
#endif
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
#ifdef RTC_LSI
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
#endif

#ifdef RTC_LSE
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
#endif
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-2- Enable RTC peripheral Clocks #######################################*/
    /* Enable RTC Clock */
    __HAL_RCC_RTC_ENABLE();

    /*##-3- Configure the NVIC for RTC WakeUp Timer ############################*/
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

/**
  * @brief RTC MSP De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
    /*##-1- Reset peripherals ##################################################*/
    __HAL_RCC_RTC_DISABLE();
}

/**
  * @brief  Initializes the TIM_Base_MSP.
  * @param  hrtc: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM.
  * @retval None
  */
//__weak void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
//{
//  /* NOTE : This function Should not be modified, when the callback is needed,
//            the HAL_TIM_Base_MspInit could be implemented in the user file
//   */
//}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10)
	{
		__HAL_RCC_TIM10_CLK_ENABLE();
		
		__HAL_TIM_ENABLE(htim);
		__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
		HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0x0E, 0);    
		HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	}
	
}

/**
  * @brief TIM BASE De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
    /*##-1- Reset peripherals ##################################################*/
	if(htim->Instance==TIM10)
	{
		__HAL_RCC_TIM10_CLK_DISABLE();
		
		__HAL_TIM_DISABLE(htim);
		__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
	
		HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
	}
}

/**
  * @brief  Initializes the TIM_PWM_MSP.
  * @param  hrtc: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM.
  * @retval None
  */
//__weak void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
//{
//  /* NOTE : This function Should not be modified, when the callback is needed,
//            the HAL_TIM_PWM_MspInit could be implemented in the user file
//   */
//}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	if(htim->Instance == TIM2)
	{
		/****时钟开启****/
		__HAL_RCC_TIM2_CLK_ENABLE();
		__GPIOA_CLK_ENABLE();
		
		/****GPIO初始化****/
		GPIO_InitStruct.Pin       = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		/****输出通道设置****/
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);
		
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	}
	
	else if(htim->Instance == TIM3)
	{
		/****时钟开启****/
		__HAL_RCC_TIM3_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();
		__GPIOC_CLK_ENABLE();
		
		/****GPIO初始化****/
		GPIO_InitStruct.Pin       = GPIO_PIN_0;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin       = GPIO_PIN_7;
		
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		/****输出通道设置****/
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3);
		
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	}
}

/**
  * @brief TIM_PWM De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
    /*##-1- Reset peripherals ##################################################*/
	if(htim->Instance == TIM2)
	{
		__HAL_RCC_TIM2_CLK_DISABLE();
	}
	
	if(htim->Instance == TIM3)
	{
		__HAL_RCC_TIM3_CLK_DISABLE();
	}
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
