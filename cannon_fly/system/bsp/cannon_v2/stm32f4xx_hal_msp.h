 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4XX_HAL_MSP_H
#define __STM32F4XX_HAL_MSP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"

void UART_Init(void);
void TIM10_Init(void);
void TIM_PWM_Init(void);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2
  
/* Exported macro ------------------------------------------------------------*/

extern UART_HandleTypeDef UartHandle;
extern TIM_HandleTypeDef TimHandle;

/* Exported functions ------------------------------------------------------- */

void sendware(uint8_t *wareaddr,uint32_t waresize);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw);
void ANO_DT_Send_Senser(float a_x,float a_y,float a_z,float g_x,float g_y,float g_z,float m_x,float m_y,float m_z,int32_t bar);

#endif /* __STM32F4XX_HAL_MSP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
