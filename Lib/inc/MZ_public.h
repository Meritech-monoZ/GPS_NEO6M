/**
 * @file MZ_Public.h
 * @author SKM
 * @date Apr 2 2021 
 * @brief Monoz Public 
 * All peripheral Configurations will send to Monoz library through this file
 */

#ifndef MZ_PUBLIC_H_
#define MZ_PUBLIC_H_  															///< Defines Monoz Public file
#ifdef __cplusplus
extern "C" {
#endif
#include "MZ_common.h"

/**
 * @struct MZ_UART_INIT_ST
 * @brief Structure for Monoz UART Initialization parameters
 */
typedef struct
{
	  USART_TypeDef            		*Instance;                					/*!< UART registers base address */
	  UART_InitTypeDef         		Init;                     					/*!< UART communication parameters */
	  UART_AdvFeatureInitTypeDef 	AdvancedInit;           					/*!< UART Advanced Features initialization parameters */
}MZ_UART_INIT_ST;

typedef	MZ_UART_INIT_ST * MZ_UART_INIT_PTR;										/*!< UART User input structure pointer */

/** @struct MZ_I2C_INIT_ST
 * @brief Structure for I2C Initialization
 */
typedef struct
{
	  I2C_TypeDef            		*Instance;                					/*!< I2C registers base address */
	  I2C_InitTypeDef         		Init;                     					/*!< I2C communication parameters */
//	  UART_AdvFeatureInitTypeDef 	AdvancedInit;           					/*!< I2C Advanced Features initialization parameters */
}MZ_I2C_INIT_ST;

typedef	MZ_I2C_INIT_ST * MZ_I2C_INIT_PTR;										/*!< I2C User input structure pointer */

/**
 * @struct uart_enable
 * Structure for UART Initialization and configuration
 */
typedef struct
{
#if(MZ_MODEM != MZ_ENABLE)&&(MZ_UART1 == MZ_ENABLE)
	uint8_t u1:1;																/*!< UART1 Enable field */
#endif
#if(MZ_CLI != MZ_ENABLE)&&(MZ_UART2 == MZ_ENABLE)
	uint8_t u2:1;																/*!< UART2 Enable field */
#endif
#if(MZ_UART3 == MZ_ENABLE)
	uint8_t u3:1;																/*!< UART3 Enable field */
#endif
#if(MZ_UART4 == MZ_ENABLE)
	uint8_t u4:1;																/*!< UART4 Enable field */
#endif
#if(MZ_UART5 == MZ_ENABLE)
	uint8_t u5:1;																/*!< UART5 Enable field */
#endif
#if(MZ_LPUART1 == MZ_ENABLE)
	uint8_t lu1:1;																/*!< LPUART1 Enable field */
#endif
#if(MZ_MODEM != MZ_ENABLE)&&(MZ_UART1 == MZ_ENABLE)
	MZ_UART_INIT_PTR u1p;														/*!< Give configurations for UART1 */
#endif
#if(MZ_CLI != MZ_ENABLE)&&(MZ_UART2 == MZ_ENABLE)
	MZ_UART_INIT_PTR u2p;														/*!< Give configurations for UART2 */
#endif
#if(MZ_UART3 == MZ_ENABLE)
	MZ_UART_INIT_PTR u3p;														/*!< Give configurations for UART3 */
#endif
#if(MZ_UART4 == MZ_ENABLE)
	MZ_UART_INIT_PTR u4p;														/*!< Give configurations for UART4 */
#endif
#if(MZ_UART5 == MZ_ENABLE)
	MZ_UART_INIT_PTR u5p;														/*!< Give configurations for UART5 */
#endif
#if(MZ_LPUART1 == MZ_ENABLE)
	MZ_UART_INIT_PTR lu1p;														/*!< Give configurations for LPUART1 */
#endif
}uart_enable;


/**
 * @struct i2c_enable
 * Structure for I2C Initialization and configuration
 */
typedef struct
{
#if(MZ_I2C1 == MZ_ENABLE)
	uint8_t i1:1;																/*!< I2C1 Enable field */
#endif
#if(MZ_I2C2 == MZ_ENABLE)
	uint8_t i2:1;																/*!< I2C2 Enable field */
#endif
#if(MZ_I2C3 == MZ_ENABLE)
	uint8_t i3:1;																/*!< I2C3 Enable field */
#endif
#if(MZ_I2C4 == MZ_ENABLE)
	uint8_t i4:1;																/*!< I2C4 Enable field */
#endif
#if(MZ_I2C1 == MZ_ENABLE)
	MZ_I2C_INIT_PTR i1p;														/*!< Give configurations for I2C1 */
#endif
#if(MZ_I2C2 == MZ_ENABLE)
	MZ_I2C_INIT_PTR i2p;														/*!< Give configurations for I2C2 */
#endif
#if(MZ_I2C3 == MZ_ENABLE)
	MZ_I2C_INIT_PTR i3p;														/*!< Give configurations for I2C3 */
#endif
#if(MZ_I2C4 == MZ_ENABLE)
	MZ_I2C_INIT_PTR i4p;														/*!< Give configurations for I2C4 */
#endif
}i2c_enable;


/*
 * Sample Template for the hardware context variable to initialize to use
 * peripherals. The name uart_enable_cfg and i2c_enable_cfg are fixed.
 * User need to define the context with this exact name.
 *
 * extern MZ_UART_INIT_ST sample_uart_instance;
 * uart_enable uart_enable_cfg =
 * {
 * .u3 = MZI_UART3,
 * .u3p = &sample_uart_instance,
 * .lu1 = MZI_LPUART1,
 * .lu1p = 0,
 * };
 *
 * extern MZ_I2C_INIT_ST sample_i2c_instance;
 *
 * i2c_enable i2c_enable_cfg =
 * {
 * .i2 = MZI_I2C2,
 * .i2p = &sample_i2c_instance,
 * .i4 = MZI_I2C4,
 * .i4p = 0,
 * };
 */


#ifdef __cplusplus
}
#endif
#endif /* MZ_PUBLIC_H_ */

