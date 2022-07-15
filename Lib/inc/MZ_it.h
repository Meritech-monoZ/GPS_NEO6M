/**
 * @file MZ_it.h
 * @date Apr 21, 2021
 * @author SKM
 * @brief This file include all Interrupt Handler Requests
 */ 


#ifndef INC_MZ_IT_H_
#define INC_MZ_IT_H_                        ///< Defines INC Monoz IT

#include "MZ_BoardCfg.h"
#include "MZ_tool_config.h"

/** 
 * @defgroup INTERRUPT_HANDLER INTERRUPT HANDLER
 * Monoz Interupt Handler
 * @{
 * 
 * @fn void MZ_USART1_IRQHandler(void)
 * @brief Monoz USART1 Interrupt Handler
 */ 
void MZ_USART1_IRQHandler(void); 

/**
 * @fn void MZ_USART2_IRQHandler(void)
 * @brief Monoz USART2 Interrupt Handler
 */
void MZ_USART2_IRQHandler(void);

/**
 * @fn void MZ_USART3_IRQHandler(void)
 * @brief Monoz USART3 Interrupt Handler
 */
void MZ_USART3_IRQHandler(void);

/**
 * @fn void MZ_UART4_IRQHandler(void)
 * @brief Monoz UART4 Interrupt Handler
 */
void MZ_UART4_IRQHandler(void);

/**
 * @fn void MZ_UART5_IRQHandler(void)
 * @brief Monoz UART5 Interrupt Handler
 */
void MZ_UART5_IRQHandler(void);

/**
 * @fn void MZ_LPUART1_IRQHandler(void)
 * @brief Monoz LPUART1 Interrupt Handler
 */
void MZ_LPUART1_IRQHandler(void);

/**
 * @fn void MZ_I2C1_EV_IRQHandler(void)
 * @brief Monoz I2C1 event Interrupt Handler
 */
void MZ_I2C1_EV_IRQHandler(void);

/**
 * @fn void MZ_I2C2_EV_IRQHandler(void)
 * @brief Monoz I2C2 event Interrupt Handler
 */
void MZ_I2C2_EV_IRQHandler(void);

/**
 * @fn void MZ_I2C3_EV_IRQHandler(void)
 * @brief Monoz I2C3 event Interrupt Handler
 */
void MZ_I2C3_EV_IRQHandler(void);

/**
 * @fn void MZ_I2C4_EV_IRQHandler(void)
 * @brief Monoz I2C4 event Interrupt Handler
 */
void MZ_I2C4_EV_IRQHandler(void);

/**
 * @fn void MZ_I2C1_ER_IRQHandler(void)
 * @brief Monoz I2C1 error Interrupt Handler
 */
void MZ_I2C1_ER_IRQHandler(void);

/**
 * @fn void MZ_I2C2_ER_IRQHandler(void)
 * @brief Monoz I2C2 error Interrupt Handler
 */
void MZ_I2C2_ER_IRQHandler(void);

/**
 * @fn void MZ_I2C3_ER_IRQHandler(void)
 * @brief Monoz I2C3 error Interrupt Handler
 */
void MZ_I2C3_ER_IRQHandler(void);

/**
 * @fn void MZ_I2C4_ER_IRQHandler(void)
 * @brief Monoz I2C4 error Interrupt Handler
 */
void MZ_I2C4_ER_IRQHandler(void);

/** @} */
#endif /* INC_MZ_IT_H_ */
