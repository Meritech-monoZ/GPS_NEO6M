/**
 * @file MZ_uart.h
 * @author SKM
 * @date Mar 3 2021 
 * @brief This file includes Monoz Uart Related APIs
 */

#ifndef MZ_UART_H_
#define MZ_UART_H_														///< Defines Inc Monoz UART file
#ifdef __cplusplus
extern "C" {
#endif

//#include "MZ_common.h"
#include "MZ_public.h"
#include "MZ_error_handler.h"

/** 
 * @enum en_uart_no
 * @brief Define UART on its state
 */  
typedef enum
{
#if(MZ_UART1 == 1)
	_USART1,						/*!< Enables the USART1 */
#endif
#if(MZ_UART2 == 1)
	_USART2,						/*!< Enables the USART2 */
#endif
#if(MZ_UART3 == 1)
	_USART3,						/*!< Enables the USART3 */
#endif
#if(MZ_UART4 == 1)
	_UART4,							/*!< Enables the USART4 */
#endif
#if(MZ_UART5 == 1)
	_UART5,							/*!< Enables the USART5 */
#endif
#if(MZ_LPUART1 == 1)
	_LPUART1,						/*!< Enables the LPUART1 */
#endif
	UART_MAX_COUNT					/*!< UART Maximum Count */
}en_uart_no;

#define UART_MAX_COUNT_SIZE UART_MAX_COUNT										///< UART Maximum Count Size

typedef _mz_fp _uart_api;														/*!< Monoz Function Pointer */								

/**
 * @struct MZ_UARTX
 * @brief Structure for UART Handling and API events
 */ 
typedef struct
{
	mzReg					_reg_status;										/*!< UART registration status */
	UART_HandleTypeDef 		_handler;											/*!< UART low level handler */
	mzUint8 				_enable;											/*!< UART enable flag */
	mzUint8 				*_rx_buffer;										/*!< UART RX buffer address */
	mzUint32				_rx_buffer_size;									/*!< UART RX buffer size */
	mzUint8 				*_tx_buffer;										/*!< UART TX buffer address */
	mzUint32				_tx_buffer_size;									/*!< UART TX buffer size */
	mzUint8 				_rx_intr;											/*!< UART RX interrupt enable flag */
	mzUint8 				_tx_intr;											/*!< UART TX interrupt enable flag */
	_uart_api 				read;												/*!< UART Read API */
	_uart_api 				write;												/*!< UART Write API */
	_uart_api 				rx_intr_api;										/*!< UART RX interrupt handler API */
	_uart_api 				tx_intr_api;										/*!< UART TX interrupt handler API */
//	_uart_api 				UART_process_api;									/*!< UART Custom processing API */
	_uart_api 				UART_ev_call_back;									/*!< UART event callback API */
}MZ_UARTX;

/**
 * @struct st_uart_intr
 * @brief Structure for UART state Handler
 */ 
typedef struct
{
	UART_HandleTypeDef	*uart_ptr;                                              /*!< Monoz UART handler */
	enum
	{
		FULL_COMPLETE,
		HALF_COMPLETE,
	}event;
}st_uart_intr;

typedef	MZ_UARTX MZ_UART_BTYPE;													/*!< UART Base Type */
typedef	MZ_UARTX * MZ_UART_BTYPE_PTR;											/*!< UART Base Type pointer */

/** 
 * @defgroup UART UART
 * UART handler
 * @{
 * @fn MZ_UART_BTYPE_PTR MZ_UART_get_base_address(void)
 * @brief Get the base address of the UART internal context 
 * @return Base address of UART
 */ 
MZ_UART_BTYPE_PTR MZ_UART_get_base_address(void);

/**
 * @fn MZ_UART_BTYPE_PTR MZ_UART_reference(en_uart_no uart_no)
 * @brief Monoz UART reference 
 * @param uart_no 
 * @return Uart Reference
 */ 
MZ_UART_BTYPE_PTR MZ_UART_reference(en_uart_no uart_no);

/**
 * @fn mz_error_t MZ_UART_init(MZ_UART_INIT_PTR _cfg)
 * @brief Register a UART. Need to call from Upper layer 
 * @param _cfg MZ_UART_INIT_PTR
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_UART_init(MZ_UART_INIT_PTR _cfg);

/**
 * @fn mz_error_t MZ_UART_register_callback(uint8_t uart_no, _uart_api cb)
 * @brief Monoz UART registering by a Callback
 * @param uart_no uint8_t
 * @param cb _uart_api
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_UART_register_callback(uint8_t uart_no, _uart_api cb);

/**
 * @fn mz_error_t MZ_UART_register_intr_cb_tx(uint8_t uart_no, _uart_api cb)
 * @brief Monoz UART registering by Interupt Callback transmission
 * @param uart_no uint8_t
 * @param cb _uart_api
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_UART_register_intr_cb_tx(uint8_t uart_no, _uart_api cb);

/**
 * @fn mz_error_t MZ_UART_register_intr_cb_rx(uint8_t uart_no, _uart_api cb)
 * @brief Monoz UART registering by Interupt Callback Reception
 * @param uart_no uint8_t
 * @param cb _uart_api
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_UART_register_intr_cb_rx(uint8_t uart_no, _uart_api cb);

/**
 * @fn mz_error_t MZ_UART_Transmit(uint8_t uart_no,uint8_t *pData,uint16_t Size,uint32_t Timeout)
 * @brief Transmit UART data
 * @param uart_no uint8_t
 * @param pData pData
 * @param Size uint16_t
 * @param Timeout uint32_t
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t MZ_UART_Transmit(uint8_t uart_no,uint8_t *pData,uint16_t Size,uint32_t Timeout);

/**
 * @fn mz_error_t MZ_UART_Receive(uint8_t uart_no,uint8_t *pData,uint16_t Size,uint32_t Timeout)
 * @brief Recive UART data
 * @param uart_no uint8_t
 * @param pData pData
 * @param Size uint16_t
 * @param Timeout uint32_t
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t MZ_UART_Receive(uint8_t uart_no,uint8_t *pData,uint16_t Size,uint32_t Timeout);

/**
 * @fn mz_error_t MZ_UART_Transmit_IT(uint8_t uart_no, uint8_t *pData, uint16_t Size)
 * @brief UART Transmit Interupt
 * @param uart_no uint8_t
 * @param pData pData
 * @param Size uint16_t
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_UART_Transmit_IT(uint8_t uart_no, uint8_t *pData, uint16_t Size);

/**
 * @fn mz_error_t MZ_UART_Receive_IT(uint8_t uart_no, uint8_t *pData, uint16_t Size)
 * @brief UART Receive Interupt
 * @param uart_no uint8_t
 * @param pData pData
 * @param Size uint16_t
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_UART_Receive_IT(uint8_t uart_no, uint8_t *pData, uint16_t Size);

/**
 * @fn void MZ_HAL_UART_IRQHandler(uint8_t uart_no)
 * @brief Global Interrupt handler functions
 * @param uart_no uint8_t
 */
void MZ_HAL_UART_IRQHandler(uint8_t uart_no);									
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_UART_H_ */
