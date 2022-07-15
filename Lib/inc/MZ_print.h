/** @file MZ_print.h
 *  @date Apr 27, 2021
 *  @author SKM
 * @brief Monoz Print
 */


#ifndef MZ_PRINT_H_
#define MZ_PRINT_H_                                     ///< Defines INC Monoz Print Header
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @defgroup PRINT PRINT
 * Monoz Print API
 * @{
 * @fn void mz_printf_ISR(void * __ch)
 * @brief Print a single character from Interrupt
 * @param __ch void
*/
void mz_printf_ISR(void * __ch);

/**
 * @fn void mz_printf(void * __ch)
 * @brief Print a single character from value
 * @param __ch void
*/
void mz_printf(void * __ch);

/**
 * @fn int mz_puts(void * __ch)
 * @brief Print character from string
 * @param __ch void
 * @return 1/0
*/
int mz_puts(void * __ch);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_PRINT_H_ */
