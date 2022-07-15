/*
 * MZ_modem_config.c
 *
 *  Created on: Jun 29, 2021
 *      Author: MZ_click
 */

#include "MZ_timer.h"
#include "MZ_Modem_public.h"
#include "MZ_Simcom_AT_include.h"

#if(MZ_SDK_ADVANCE_AT_INJECT == 1)

// Please refer the Advance documentation.

#else
char buff[] = {'{','"','h','e','l','l','o','"',':','"','1','0','"','}',26,'\0'};
void bg96_setup(void)
{
	mz_raw_at_exe_csq();
	mz_raw_at_read_cops();
	MZ_init_cmd_direct("AT\r\n",AT_TIME_1SEC, AT_TIME_1SEC);
//	MZ_init_cmd_direct("AT+QMTDISC=0\r\n",AT_TIME_15SEC, AT_TIME_15SEC);
//	MZ_init_cmd_direct("AT+QMTOPEN=0,\"cloud.monoz.io\",1883\r\n",AT_TIME_15SEC, AT_TIME_15SEC);
//	MZ_init_cmd_direct("AT+QMTCONN=0,\"test999\",\"test999\",\"test999\"\r\n",AT_TIME_15SEC, AT_TIME_15SEC);
//	MZ_init_cmd_direct("AT+QMTPUB=0,0,0,0,\"v1/devices/me/telemetry\"\r\n",AT_TIME_15SEC, 0);
//	MZ_init_cmd_direct(buff,AT_TIME_15SEC, AT_TIME_15SEC);

}


void sim7080g_setup(void)
{
	mz_raw_at_exe_csq();
	mz_raw_at_read_cops();
	mz_raw_at_exe_CGNAPN();
	mz_raw_at_write_CNACT(0,0);
	mz_raw_at_write_CNCFG(0,1,"\"iot.1nce.net\"","\"\"","\"\"",0);
	mz_raw_at_write_CNACT(0,1);
	mz_raw_at_read_CNACT();
}
void mz_reset_sequence(void * arg)
{
	(void)arg;
	bg96_setup();
}

void mz_reboot_sequence(void * arg)
{
	(void)arg;
	bg96_setup();
	//sim7080g_setup();
}

#endif //(MZ_SDK_ADVANCE_AT_INJECT == 1)
