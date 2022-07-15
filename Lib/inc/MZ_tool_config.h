/** @file MZ_tool_config.h
 *  @date Mar 3, 2021
 *  @author SKM
 * @brief Configuration of Monoz Tool
 */


#ifndef MZ_TOOL_CONFIG_H_
#define MZ_TOOL_CONFIG_H_                       ///< Defines the monoz tool configuration
#ifdef __cplusplus
extern "C" {
#endif
#ifdef MZ_LIB_BUILD
// If SDK Internal build, Then include Dummy configuration file to avoid build fail
#include "MZ_tool_pseudo.h"
#else
// Include real tool generated header file
#include "MZ_tool_gen.h"
#endif
#ifdef __cplusplus
}
#endif
#endif /* MZ_TOOL_CONFIG_H_ */

