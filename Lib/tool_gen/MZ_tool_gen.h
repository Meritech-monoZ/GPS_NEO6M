/**
 * @file MZ_tool_gen.h
 * @date Mar 3  2021
 * @author MZ_click
 * @brief  This is a tool generated file. Do not edit manually
 */ 


#ifndef INC_MZ_TOOL_GEN_H_
#define INC_MZ_TOOL_GEN_H_                              						///< Defines Inc Monoz Tool Generator
#include"MZ_BoardCfg.h"

#if((MZ_UART1 == MZ_ENABLE)&&(MZ_MODEM != MZ_ENABLE))
#define MZI_UART1	(MZ_ENABLE)
#endif //(MZ_UART1 == MZ_ENABLE)

#if((MZ_UART2 == MZ_ENABLE)&&(MZ_CLI != MZ_ENABLE))
#define MZI_UART2	(MZ_ENABLE)
#endif //(MZ_UART2 == MZ_ENABLE)

#if(MZ_UART3 == MZ_ENABLE)
#define MZI_UART3	(MZ_DISABLE)
#endif //(MZ_UART3 == MZ_ENABLE)

#if(MZ_UART4 == MZ_ENABLE)
#define MZI_UART4	(MZ_ENABLE)
#endif //(MZ_UART4 == MZ_ENABLE)

#if(MZ_UART5 == MZ_ENABLE)
#define MZI_UART5	(1)
#endif //(MZ_UART5 == MZ_ENABLE)

#if(MZ_LPUART1 == MZ_ENABLE)
#define MZI_LPUART1	(MZ_ENABLE)
#endif //(MZ_LPUART1 == MZ_ENABLE)

#endif /* INC_MZ_TOOL_GEN_H_ */
