/** @file  MZ_i2c.h
 * @date Jun 8, 2021
 * @author Mahendra
 * @brief This file contains all the functionalities for I2C protocol
 */

#ifndef MZ_I2C_H_
#define MZ_I2C_H_
#ifdef __cplusplus
extern "C" {
#endif

//#include "MZ_common.h"
#include "MZ_public.h"
#include "MZ_error_handler.h"
#include "MZ_i2c.h"

/** @enum en_i2c_no
 * @brief Enumeration for Monoz I2C
 * Enable or disable the I2C instance
 */
typedef enum
{
#if(MZ_I2C1 == 1)
	_I2C1,											/*!< i2c1 is enable */
#endif
#if(MZ_I2C2 == 1)
	_I2C2,											/*!< i2c2 is enable */
#endif
#if(MZ_I2C3 == 1)
	_I2C3,											/*!< i2c3 is enable */
#endif
#if(MZ_I2C4 == 1)
	_I2C4,											/*!< i2c4 is enable */
#endif
	I2C_MAX_COUNT									/*!< i2c maximum count */
}en_i2c_no;

#define I2C_MAX_COUNT_SIZE I2C_MAX_COUNT

/**
 * @typedef _i2c_api
 * @brief use _i2c_api as monoz function pointer
 */
typedef _mz_fp _i2c_api;

/** @struct MZ_I2CX
 * @brief Structure for Monoz I2C events
 */
typedef struct
{
	mzReg					_reg_status;										/*!< I2C registration status */
	I2C_HandleTypeDef 		_handler;											/*!< I2C low level handler */
	mzUint8 				_enable;											/*!< I2C enable flag */
	mzUint8 				*_rx_buffer;										/*!< I2C RX buffer address */
	mzUint32				_rx_buffer_size;									/*!< I2C RX buffer size */
	mzUint8 				*_tx_buffer;										/*!< I2C TX buffer address */
	mzUint32				_tx_buffer_size;									/*!< I2C TX buffer size */
	mzUint8 				_rx_intr;											/*!< I2C RX interrupt enable flag */
	mzUint8 				_tx_intr;											/*!< I2C TX interrupt enable flag */
	mzUint8					_dev_add;											/*!< I2C Slave Address*/
	_i2c_api 				read;												/*!< I2C Read API */
	_i2c_api 				write;												/*!< I2C Write API */
	_i2c_api 				rx_intr_api;										/*!< I2C RX interrupt handler API */
	_i2c_api 				tx_intr_api;										/*!< I2C TX interrupt handler API */
//	_i2c_api 				I2C_process_api;									/*!< I2C Custom processing API */
	_i2c_api 				I2C_ev_call_back;									/*!< I2C event callback API */
}MZ_I2CX;

/**
 * @struct st_i2c_intr
 * @brief Structure for I2C interrupts
 */
typedef struct
{
	I2C_HandleTypeDef	*i2c_ptr;
	enum
	{
		FULL_COMPLETE1,
		HALF_COMPLETE1,
	}event;
}st_i2c_intr;


typedef	MZ_I2CX MZ_I2C_BTYPE;													/*!< I2C Base Type */
typedef	MZ_I2CX * MZ_I2C_BTYPE_PTR;												/*!< I2C Base Type pointer */

/**
 * @defgroup I2C I2C
 * Monoz I2C
 * @{
 *
 * @fn MZ_I2C_BTYPE_PTR MZ_I2C_get_base_address(void)
 * @brief Get the base address of the I2C internal context
 * @return Base address of I2C
 */
MZ_I2C_BTYPE_PTR MZ_I2C_get_base_address(void);								// Get the base address of the I2C internal context */

/**
 * @fn MZ_I2C_BTYPE_PTR MZ_I2C_reference(en_i2c_no i2c_no)
 * @brief Get the reference of the I2C internal context
 * @param i2c_no en_i2c_no
 * @return Reference of I2C
 */
MZ_I2C_BTYPE_PTR MZ_I2C_reference(en_i2c_no i2c_no);

/**
 * @fn mz_error_t MZ_I2C_init(MZ_I2C_INIT_PTR _cfg)
 * @brief Register and initialize the I2C
 *
 * Register a I2C. Need to call from Upper layer
 * @param _cfg MZ_I2C_INIT_PTR
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_I2C_init(MZ_I2C_INIT_PTR _cfg);									// Register a I2C. Need to call from Upper layer */

/**
 * @fn mz_error_t MZ_I2C_register_callback(uint8_t I2C_no, _i2c_api cb)
 * @brief Register the callback of I2C
 *
 * Register a I2C. Need to call from Upper layer
 * @param I2C_no uint8_t
 * @param cb _i2c_api
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_I2C_register_callback(uint8_t I2C_no, _i2c_api cb);

/**
 * @fn mz_error_t MZ_I2C_register_intr_cb_tx(uint8_t i2c_no, _i2c_api cb)
 * @brief Register the callback Interrupt for transmission in I2C
 * @param i2c_no uint8_t
 * @param cb _i2c_api
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_I2C_register_intr_cb_tx(uint8_t i2c_no, _i2c_api cb);

/**
 * @fn mz_error_t MZ_I2C_register_intr_cb_rx(uint8_t i2c_no, _i2c_api cb)
 * @brief Register the callback Interrupt for reception in I2C
 * @param i2c_no uint8_t
 * @param cb _i2c_api
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_I2C_register_intr_cb_rx(uint8_t i2c_no, _i2c_api cb);

/**
 * @fn mz_error_t MZ_I2C_Master_Transmit(uint8_t i2c_no,uint16_t DevAddress,uint8_t *pData,uint16_t Size,uint32_t Timeout)
 * @brief Monoz I2C Master Transmittion
 * @param i2c_no uint8_t
 * @param DevAddress uint16_t
 * @param *pData uint8_t
 * @param Size uint16_t
 * @param Timeout uint32_t
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_I2C_Master_Transmit(uint8_t i2c_no,uint16_t DevAddress,uint8_t *pData,uint16_t Size,uint32_t Timeout);

/**
 * @fn mz_error_t MZ_I2C_Master_Receive(uint8_t i2c_no,uint16_t DevAddress,uint8_t *pData,uint16_t Size,uint32_t Timeout)
 * @brief Monoz I2C Master Reception
 * @param i2c_no uint8_t
 * @param DevAddress uint16_t
 * @param *pData uint8_t
 * @param Size uint16_t
 * @param Timeout uint32_t
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_I2C_Master_Receive(uint8_t i2c_no,uint16_t DevAddress,uint8_t *pData,uint16_t Size,uint32_t Timeout);

/**
 * @fn mz_error_t MZ_I2C_Master_Transmit_IT(uint8_t i2c_no,uint16_t DevAddress,uint8_t *pData,uint16_t Size)
 * @brief Monoz I2C Interrupt for Master Transmittion
 * @param i2c_no uint8_t
 * @param DevAddress uint16_t
 * @param *pData uint8_t
 * @param Size uint16_t
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_I2C_Master_Transmit_IT(uint8_t i2c_no,uint16_t DevAddress,uint8_t *pData,uint16_t Size);

/**
 * @fn mz_error_t MZ_I2C_Master_Receive_IT(uint8_t i2c_no,uint16_t DevAddress,uint8_t *pData,uint16_t Size)
 * @brief Monoz I2C Interrupt for Master Reception
 * @param i2c_no uint8_t
 * @param DevAddress uint16_t
 * @param *pData uint8_t
 * @param Size uint16_t
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t MZ_I2C_Master_Receive_IT(uint8_t i2c_no,uint16_t DevAddress,uint8_t *pData,uint16_t Size);

/**
 * @fn void MZ_HAL_I2C_EV_IRQHandler(uint8_t i2c_no)
 * @brief Monoz HAL evnet Interrupt Handler Request
 * @param i2c_no uint8_t
 */
void MZ_HAL_I2C_EV_IRQHandler(uint8_t i2c_no);									/*!< Global Interrupt handler function */

/**
 * @fn void MZ_HAL_I2C_ER_IRQHandler(uint8_t i2c_no)
 * @brief Monoz HAL I2C error Interrupt Handler Request
 * @param i2c_no uint8_t
 */
void MZ_HAL_I2C_ER_IRQHandler(uint8_t i2c_no);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_I2C_H_ */
