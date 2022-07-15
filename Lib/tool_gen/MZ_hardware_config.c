/*
 * MZ_hardware_config.c
 *
 *  Created on: Apr 2, 2021
 *      Author: MZ_click
 *  This is a tool generated file. Do not edit manually
 */

#include "MZ_public.h"

//extern MZ_UART_INIT_ST uart3_instance;
////extern MZ_UART_INIT_ST lpuart1_instance;
extern MZ_UART_INIT_ST gps_lpuart1_instance;

uart_enable uart_enable_cfg =
{
.u3 = MZI_UART3,
.u3p = 0,
.lu1 = MZI_LPUART1,
.lu1p = &gps_lpuart1_instance,
};

//extern MZ_I2C_INIT_ST i2c2_instance;
extern MZ_I2C_INIT_ST uv_i2c2_instance;

i2c_enable i2c_enable_cfg =
{
.i2 = MZI_I2C2,
.i2p = 0,//&uv_i2c2_instance,
.i4 = MZI_I2C4,
.i4p = 0,
};

