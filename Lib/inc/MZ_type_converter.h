/** @file MZ_type_converter.h
 *  @date Jun 10, 2021
 *  @author SKM
 * @brief Monoz Type Converter
 * This file contains the APIs used to hex to ascii  and ascii to hex conversion
 */


#ifndef MZ_TYPE_CONVERTER_H_
#define MZ_TYPE_CONVERTER_H_                        ///< Define Monoz Type Converter
#ifdef __cplusplus
extern "C" {
#endif
/** 
 * @defgroup TYPE_CONVERTION TYPE CONVERTION
 * Convert datatype of value
 * @{
 * @fn void mz_ascii_to_hex(void * in, void * out)
 * @brief Monoz ASCII to Hex Converter
 * @param in void
 * @param out void
 */ 
void mz_ascii_to_hex(void * in, void * out);

/**
 * @fn void mz_hex_to_ascii(void * in, void * out)
 * @brief Monoz Hex to ASCII Converter
 * @param in void
 * @param out void
 */ 
void mz_hex_to_ascii(void * in, void * out);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_TYPE_CONVERTER_H_ */
