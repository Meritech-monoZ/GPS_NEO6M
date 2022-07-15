/** @file MZ_device_include.h
 *  @date Jul 27, 2021
 *  @author SKM
 *  @brief Monoz lib common Devices which are included
 */

#ifndef MZ_DEVICE_INCLUDE_H_
#define MZ_DEVICE_INCLUDE_H_                        ///< Monoz Device include
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @defgroup DEVICE_INCLUDE DEVICE INCLUDE
 * Monoz Device Include API
 * @{
 *
 * @fn char * mzd_read_imei(void)
 * @brief Read IMEI of Monoz Device
 * @return Device IMEI number
*/
char * mzd_read_imei(void);

/** @fn char * mzd_read_cimi(void)
 * @brief Read CIMI of Monoz Device
 * @return Device CIMI number
 */
char * mzd_read_cimi(void);

/**
 * @fn char * mzd_read_msisdn(void)
 * @brief Read MSISDN of Monoz Device
 * @return Device MSISDN number
 */
char * mzd_read_msisdn(void);

/**
 * @fn char * mzd_read_network(void)
 * @brief Read Network of Monoz Device
 * @return Device Network
 */
char * mzd_read_network(void);

/**
 * @fn char mzd_read_phone_function(void)
 * @brief Read phone functionality of Monoz Device
 * @return Device NetworkFunction of the phone
 */
char mzd_read_phone_function(void);

/**
 * @fn char mzd_read_cops(void)
 * @brief Read cops of Monoz Device
 * @return Device COPS value
 */
char mzd_read_cops(void);

/**
 * @fn char mzd_read_dev_reg_status(void)
 * @brief Read Monoz Device Registration status
 * @return Device Register status
 */
char mzd_read_dev_reg_status(void);

/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_DEVICE_INCLUDE_H_ */
