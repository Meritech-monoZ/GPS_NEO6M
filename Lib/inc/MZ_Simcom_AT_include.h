/*
 * MZ_Simcom_AT_include.h
 *
 *  Created on: Oct 6, 2021
 *      Author: SKM
 */

#ifndef MZ_SIMCOM_AT_INCLUDE_H_
#define MZ_SIMCOM_AT_INCLUDE_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "MZ_error_handler.h"

#if(MZ_MODEM_SIMCOM == MZ_ENABLE)
// Standard AT CMD's

/**
 * @defgroup MURATA_AT_COMMAND MURATA AT COMMAND
 * AT commands of MURATA
 * @{
 * @fn mz_error_t mz_raw_at_exe_at(void)
 * @brief This API issues AT command. Expected output is OK/Error
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_at(void);

/**
 * @fn mz_error_t mz_raw_at_test_cgmi(void)
 * @brief Issues AT+CGMI=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cgmi(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cgmi(void)
 * @brief Issues AT+CGMI? command to request the manufacturer identification
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cgmi(void);

/**
 * @fn mz_error_t mz_raw_at_test_gmi(void)
 * @brief Issues AT+GMI=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_gmi(void);

/**
 * @fn mz_error_t mz_raw_at_exe_gmi(void)
 * @brief Issues AT+GMI? command to request the TA manufacturer identification
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_gmi(void);

/**
 * @fn mz_error_t mz_raw_at_test_cgmm(void)
 * @brief Issues AT+CGMM=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cgmm(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cgmm(void)
 * @brief Monoz Standard  AT Command for Requests Model Identification
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cgmm(void);

/**
 * @fn mz_error_t mz_raw_at_test_gmm(void)
 * @brief Issues AT+GMM=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_gmm(void);

/**
 * @fn mz_error_t mz_raw_at_exe_gmm(void)
 * @brief Monoz Standard  AT Command for Requests TA Model Identification
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_gmm(void);

/**
 * @fn mz_error_t mz_raw_at_test_cgmr(void)
 * @brief Issues AT+CGMR=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cgmr(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cgmr(void)
 * @brief Monoz Standard  AT Command for Requests revision Identification
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cgmr(void);

/**
 * @fn mz_error_t mz_raw_at_test_gmr(void)
 * @brief Issues AT+GMR=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_gmr(void);

/**
 * @fn mz_error_t mz_raw_at_exe_gmr(void)
 * @brief Monoz Standard  AT Command for Requests TA revision Identification
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_gmr(void);

/**
 * @fn mz_error_t mz_raw_at_test_cgsn(void)
 * @brief Issues AT+CGSN=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cgsn(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cgsn(void)
 * @brief Monoz Standard  AT Command for Requests serial number Identification
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cgsn(void);

/**
 * @fn mz_error_t mz_raw_at_test_gsn(void)
 * @brief Issues AT+GSN=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_gsn(void);

/**
 * @fn mz_error_t mz_raw_at_exe_gsn(void)
 * @brief Monoz Standard  AT Command for Requests TA serial number Identification
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_gsn(void);

/**
 * @fn mz_error_t mz_raw_at_test_cimi(void)
 * @brief Issues AT+CIMI=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cimi(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cimi(void)
 * @brief Monoz Standard  AT Command for Requests  international mobile subscriber
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cimi(void);

/**
 * @fn mz_error_t mz_raw_at_test_cnum(void)
 * @brief Issues AT+CNUM=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cnum(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cnum(void)
 * @brief Monoz Standard  AT Command for subscriber number
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cnum(void);

/**
 * @fn mz_error_t mz_raw_at_exe_z(void)
 * @brief AT sets all parameters to their defaults as specified by a user memory profile or by the manufacturer,and resets AT
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_z(void);

/**
 * @fn mz_error_t mz_raw_at_exe_i(void)
 * @brief Monoz Standard  AT Command for Request manufacturer specific information about the TA
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_i(void);

/**
 * @fn mz_error_t mz_raw_at_write_cfun(mzUint8 setval)
 * @brief This command is used to change the mode of phone functionality
 * @param setval mzUint8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cfun(mzUint8 setval);

/**
 * @fn mz_error_t mz_raw_at_test_cfun(void)
 * @brief Issues AT+CFUN=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cfun(void);

/**
 * @fn mz_error_t mz_raw_at_read_cfun(void)
 * @brief Read  phone functionality
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cfun(void);

/**
 * @fn mz_error_t mz_raw_at_test_cereg(void)
 * @brief Issues AT+CEREG=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cereg(void);

/**
 * @fn mz_error_t mz_raw_at_write_cereg(mzUint8 setval)
 * @brief This command is used to subscribe or unsubscibe unsolicited network status notification
 * @param setval mzUint8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cereg(mzUint8 setval);

/**
 * @fn mz_error_t mz_raw_at_read_cereg(void)
 * @brief EPS network registration status
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cereg(void);

/**
 * @fn mz_error_t mz_raw_at_exe_e(mzUint8 setval)
 * @brief Monoz Standard Echo AT command
 * @param setval mzUint8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_e(mzUint8 setval);

/**
 * @fn mz_error_t mz_raw_at_exe_e0(void)
 * @brief Command Echo disabled
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_e0(void);

/**
 * @fn mz_error_t mz_raw_at_exe_e1(void)
 * @brief Command Echo enabled
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_e1(void);

/**
 * @fn mz_error_t mz_raw_at_write_ipr(mzUint32 band)
 * @brief This command is used to set the baud rate of the serial interface of the devive
 * @param band mzUint32
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_ipr(mzUint32 band);

/**
 * @fn mz_error_t mz_raw_at_test_cpsms(void)
 * @brief Issues AT+CPSMS=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cpsms(void);

/**
 * @fn mz_error_t mz_raw_at_write_cpsms(mzUint8 mode, char * RPRAU,char * RGRT,char * RPTAU,char * RAT)
 * @brief This command is used to set the power saving mode
 * @param mode mzUint8
 * @param RPRAU char
 * @param RGRT char
 * @param RPTAU char
 * @param RAT char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cpsms(mzUint8 mode, char * RPRAU,char * RGRT,char * RPTAU,char * RAT);

/**
 * @fn mz_error_t mz_raw_at_read_cpsms(void)
 * @brief Power Saving Mode Setting
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cpsms(void);

/**
 * @fn mz_error_t mz_raw_at_test_cedrxs(void)
 * @brief Issues AT+CEDRXS=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cedrxs(void);

/**
 * @fn mz_error_t mz_raw_at_write_cedrxs(mzUint8 mode, mzUint8 actiontype,char * ReDRXv)
 * @brief This command is used to controls  the setting of eDRX parameter
 * @param mode mzUint8
 * @param actiontype mzUint8
 * @param ReDRXv char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cedrxs(mzUint8 mode, mzUint8 actiontype,char * ReDRXv);

/**
 * @fn mz_error_t mz_raw_at_read_cedrxs(void)
 * @brief Controls the setting of  eDRX parameters
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cedrxs(void);

/**
 * @fn mz_error_t mz_raw_at_test_cedrxrdp(void)
 * @brief Issues AT+CEDRXRDP=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cedrxrdp(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cedrxrdp(void)
 * @brief Retrieves eDRX parameters
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cedrxrdp(void);

/**
 * @fn mz_error_t mz_raw_at_write_cmgd(mzInt8 range , mzInt8 delflag)
 * @brief Monoz Standard  AT Command for Delete Messages
 * @param range mzInt8
 * @param delflag mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cmgd(mzInt8 range , mzInt8 delflag);

/**
 * @fn mz_error_t mz_raw_at_test_cmgd(void)
 * @brief Issues AT+CMGD=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cmgd(void);

/**
 * @fn mz_error_t mz_raw_at_write_cmgf(mzInt8 stat)
 * @brief This command is used to select between protocol data unit and message format
 * @param stat mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cmgf(mzInt8 stat);

/**
 * @fn mz_error_t mz_raw_at_test_cmgf(void)
 * @brief Issues AT+CMGF=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cmgf(void);

/**
 * @fn mz_error_t mz_raw_at_read_cmgf(void)
 * @brief Monoz Standard  AT Command for Message Format
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cmgf(void);

/**
 * @fn mz_error_t mz_raw_at_write_cmgl(mzInt8 setvalue)
 * @brief This command is used to list messages from prefered storage
 * @param setvalue mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cmgl(mzInt8 setvalue);

/**
 * @fn mz_error_t mz_raw_at_test_cmgl(void)
 * @brief Issues AT+CMGL=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cmgl(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cmgl(void)
 * @brief Monoz Standard  AT Command for List Messages
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cmgl(void);

/**
 * @fn mz_error_t mz_raw_at_write_cmgr(mzInt8 setvalue)
 * @brief This command is used to read messages from prefered storage
 * @param setvalue mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cmgr(mzInt8 setvalue);

/**
 * @fn mz_error_t mz_raw_at_test_cmgr(void)
 * @brief Issues AT+CMGR=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cmgr(void);

/**
 * @fn mz_error_t mz_raw_at_write_cmgs(mzInt8 setvalue)
 * @brief This command is used to send the message from host
 * @param setvalue mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cmgs(mzInt8 setvalue);

/**
 * @fn mz_error_t mz_raw_at_test_cmgs(void)
 * @brief Issues AT+CMGS=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cmgs(void);

/**
 * @fn mz_error_t mz_raw_at_write_cmgw(mzInt8 setvalue)
 * @brief This command is used to write the message to the memory
 * @param setvalue mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cmgw(mzInt8 setvalue);

/**
 * @fn mz_error_t mz_raw_at_test_cmgw(void)
 * @brief Issues AT+CMGW=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cmgw(void);

/**
 * @fn mz_error_t mz_raw_at_write_cmss(mzInt8 setvalue, char * da , mzInt8 toda)
 * @brief This command is used to Send SMS from storage
 * @param setvalue mzInt8
 * @param da char
 * @param toda  mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cmss(mzInt8 setvalue, char * da , mzInt8 toda);

/**
 * @fn mz_error_t mz_raw_at_test_cmss(void)
 * @brief Issues AT+CMSS=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cmss(void);

/**
 * @fn mz_error_t mz_raw_at_write_cnma(mzInt8 setvalue)
 * @brief This command is used to send a new message or delivery status acknowledgement
 * @param setvalue mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cnma(mzInt8 setvalue);

/**
 * @fn mz_error_t mz_raw_at_test_cnma(void)
 * @brief Issues AT+CNMA=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cnma(void);

/**
 * @fn mz_error_t mz_raw_at_exe_cnma(void)
 * @brief New Message Acknowledgement to ME/TA
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_cnma(void);

/**
 * @fn mz_error_t mz_raw_at_write_cnmi(mzInt8 setvalue, mzInt8 mt, mzInt8 bm, mzInt8 ds, mzInt8 bfr)
 * @brief This command is used to registers and unregisters and SMS clients
 * @param setvalue mzInt8 -(1,2)
 * @param mt mzInt8 -(0-2)
 * @param bm mzInt8 -(0,2)
 * @param ds mzInt8 -(0-2)
 * @param bfr mzInt8 -(0-1)
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cnmi(mzInt8 setvalue, mzInt8 mt, mzInt8 bm, mzInt8 ds, mzInt8 bfr);

/**
 * @fn mz_error_t mz_raw_at_test_cnmi(void)
 * @brief Issues AT+CNMI=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cnmi(void);

/**
 * @fn mz_error_t mz_raw_at_read_cnmi(void)
 * @brief It request to read New Message Indications
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cnmi(void);
/**
 * @fn mz_error_t mz_raw_at_write_cpms(char *mem1, char *mem2, char * mem3)
 * @brief The +CPMS command selects the memory storage "ME/SM"
 * @param mem1 Char
 * @param mem2 char
 * @param mem3 char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cpms(char *mem1, char *mem2, char * mem3);

/**
 * @fn mz_error_t mz_raw_at_test_cpms(void)
 * @brief Issues AT+CPMS=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cpms(void);

/**
 * @fn mz_error_t mz_raw_at_read_cpms(void)
 * @brief It request to read Preferred Message Storage
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cpms(void);

/**
 * @fn mz_error_t mz_raw_at_write_csca(char *sca, mzInt8 tosca)
 * @brief Set the SMS service center address
 * @param sca Char -<phone_number>
 * @param tosca mzInt8 -<address_type(129/145)>
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_csca(char *sca, mzInt8 tosca);

/**
 * @fn mz_error_t mz_raw_at_test_csca(void)
 * @brief Issues AT+CSCA=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_csca(void);

/**
 * @fn mz_error_t mz_raw_at_read_csca(void)
 * @brief It request to read Service Centre Address
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_csca(void);

/**
 * @fn mz_error_t mz_raw_at_write_cscs(char *setvalue)
 * @brief This command is used to select the character set
 * @param setvalue Char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cscs(char *setvalue);

/**
 * @fn mz_error_t mz_raw_at_test_cscs(void)
 * @brief Issues AT+CSCS=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cscs(void);

/**
 * @fn mz_error_t mz_raw_at_read_cscs(void)
 * @brief It request to read Select TE (Terminal) character set
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cscs(void);

/**
 * @fn mz_error_t mz_raw_at_write_csmp(mzInt8 fo, mzInt8 vp, mzInt8 pid, mzInt8 dcs)
 * @brief Used to set text mode parameters
 * @param fo mzInt8 -(1,17,33,49,65,81,97,113)
 * @param vp mzInt8 -(0-255)
 * @param pid mzInt8 -(0)
 * @param dcs mzInt8 -(0,4,8)
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_csmp(mzInt8 fo, mzInt8 vp, mzInt8 pid, mzInt8 dcs);

/**
 * @fn mz_error_t mz_raw_at_test_csmp(void)
 * @brief Issues AT+CSMP=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_csmp(void);

/**
 * @fn mz_error_t mz_raw_at_read_csmp(void)
 * @brief It request to read Set Text Mode Parameters
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_csmp(void);

/**
 * @fn mz_error_t mz_raw_at_write_csms(mzInt8 setvalue)
 * @brief This command is used to select the message service
 * @param setvalue mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_csms(mzInt8 setvalue);

/**
 * @fn mz_error_t mz_raw_at_test_csms(void)
 * @brief Issue AT+CSMS=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_csms(void);

/**
 * @fn mz_error_t mz_raw_at_read_csms(void)
 * @brief It request to read Select Message Service
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_csms(void);

/**
 * @fn mz_error_t mz_raw_at_write_cmee(mzInt8 setvalue)
 * @brief This command is used to enable or disable the use of result code
 * @param setvalue mzInt8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_cmee(mzInt8 setvalue);

/**
 * @fn mz_error_t mz_raw_at_test_cmee(void)
 * @brief Issue AT+CMEE=? command to test this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_cmee(void);

/**
 * @fn mz_error_t mz_raw_at_read_cmee(void)
 * @brief It request to read Report mobile termination error
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cmee(void);

/**
 * @fn mz_error_t mz_raw_at_test_csq(void)
 * @brief Issue AT+CSQ=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_csq(void);

/**
 * @fn mz_error_t mz_raw_at_exe_csq(void)
 * @brief Issue AT+CSQ=? command to request the signal quality
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_csq(void);

/**
 * @fn mz_error_t mz_raw_at_test_CCID(void)
 * @brief Issue AT+CCID=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_CCID(void);

/**
 * @fn mz_error_t mz_raw_at_read_CCID(void)
 * @brief It request to read Reads the ICCID from SIM EFICCID
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_CCID(void);

/**
 * @fn mz_error_t mz_raw_at_exe_CCID(void)
 * @brief Reads the ICCID from SIM EFICCID
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_exe_CCID(void);

/**
 * @fn mz_error_t mz_raw_at_read_cops(void)
 * @brief Meritech Specific AT Command
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cops(void);
mz_error_t mz_raw_at_write_cops(int mode, int access_technology, char * network);
mz_error_t mz_raw_at_test_cops(void);

/**
 * @fn mz_error_t mz_raw_at_read_cclk(void)
 * @brief Meritech Specific AT Command
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_cclk(void);

/** Simcom specific AT CMD's */
mz_error_t mz_raw_at_write_CNSMOD(int n);
mz_error_t mz_raw_at_test_CNSMOD(void);
mz_error_t mz_raw_at_read_CNSMOD(void);
typedef enum
{
	CNMP_AUTOMATIC = 2,
	CNMP_GSM_ONLY = 13,
	CNMP_LTE_ONLY = 38,
	CNMP_GSM_AND_LTE_ONLY = 51,
}CNMP_mode;

mz_error_t mz_raw_at_write_CNMP(CNMP_mode mode);
mz_error_t mz_raw_at_test_CNMP(void);
mz_error_t mz_raw_at_read_CNMP(void);

typedef enum
{
	CMNB_CATM = 1,
	CMNB_NBIOT = 2,
	CMNB_CATM_AND_NBIOT = 3,
}CMNB_mode;

mz_error_t mz_raw_at_write_CMNB(CMNB_mode mode);
mz_error_t mz_raw_at_test_CMNB(void);
mz_error_t mz_raw_at_read_CMNB(void);

mz_error_t mz_raw_at_test_CPSI(void);
mz_error_t mz_raw_at_read_CPSI(void);

mz_error_t mz_raw_at_exe_CGNAPN(void);
mz_error_t mz_raw_at_test_CGNAPN(void);

typedef enum
{
	CBANDCFG_CATM = 1,
	CBANDCFG_NBIOT = 2,
}CBANDCFG_mode;
typedef enum
{
	CBANDCFG_BAND_1 = 1,
	CBANDCFG_BAND_2 = 2,
	CBANDCFG_BAND_3 = 3,
	CBANDCFG_BAND_4 = 4,
	CBANDCFG_BAND_5 = 5,
	CBANDCFG_BAND_8 = 8,
	CBANDCFG_BAND_12 = 12,
	CBANDCFG_BAND_13 = 13,
	CBANDCFG_BAND_14 = 14,
	CBANDCFG_BAND_18 = 18,
	CBANDCFG_BAND_19 = 19,
	CBANDCFG_BAND_20 = 20,
	CBANDCFG_BAND_25 = 25,
	CBANDCFG_BAND_26 = 26,
	CBANDCFG_BAND_27 = 27,
	CBANDCFG_BAND_28 = 28,
	CBANDCFG_BAND_66 = 66,
	CBANDCFG_BAND_71 = 71,
	CBANDCFG_BAND_85 = 85,
}CBANDCFG_band;
mz_error_t mz_raw_at_write_CBANDCFG(CBANDCFG_mode mode,CBANDCFG_band band);
mz_error_t mz_raw_at_test_CBANDCFG(void);
mz_error_t mz_raw_at_read_CBANDCFG(void);

typedef enum
{
	CNBS_UE_SNR_LVL_0_BAND_SCAN = 1,
	CNBS_UE_SNR_LVL_0_1_BAND_SCAN = 2,
	CNBS_UE_SNR_LVL_0_1_2_BAND_SCAN = 3,
	CNBS_UE_SNR_LVL_2_BAND_SCAN = 5,
}
CNBS_mode;

mz_error_t mz_raw_at_write_CNBS(CNBS_mode mode);
mz_error_t mz_raw_at_test_CNBS(void);
mz_error_t mz_raw_at_read_CNBS(void);


// MQTT related
mz_error_t mz_raw_at_write_SMCONF(char * MQTTParamTag,char * MQTTParamValue);
mz_error_t mz_raw_at_test_SMCONF(void);
mz_error_t mz_raw_at_read_SMCONF(void);
//[TODO] mz_error_t mz_raw_at_write_SMSSL(void);
mz_error_t mz_raw_at_test_SMSSL(void);
mz_error_t mz_raw_at_read_SMSSL(void);
mz_error_t mz_raw_at_exe_SMCONN(void);
mz_error_t mz_raw_at_write_SMPUB(char * topic,int content_len, int qos, int retain);
mz_error_t mz_raw_at_test_SMPUB(void);
mz_error_t mz_raw_at_write_SMSUB(char * topic, int qos);
mz_error_t mz_raw_at_test_SMSUB(void);
mz_error_t mz_raw_at_write_SMUNSUB(char * topic);
mz_error_t mz_raw_at_test_SMUNSUB(void);
mz_error_t mz_raw_at_test_SMSTATE(void);
mz_error_t mz_raw_at_read_SMSTATE(void);
mz_error_t mz_raw_at_write_SMPUBHEX(int status);
mz_error_t mz_raw_at_test_SMPUBHEX(void);
mz_error_t mz_raw_at_read_SMPUBHEX(void);
mz_error_t mz_raw_at_exe_SMDISC(void);
//[TODO]mz_error_t mz_raw_at_write_SMALIAUTH(void);
mz_error_t mz_raw_at_test_SMALIAUTH(void);
mz_error_t mz_raw_at_read_SMALIAUTH(void);
//[TODO]mz_error_t mz_raw_at_write_SMALIDYNA(void);
mz_error_t mz_raw_at_test_SMALIDYNA(void);
mz_error_t mz_raw_at_read_SMALIDYNA(void);



// IP related
mz_error_t mz_raw_at_write_CNACT(int pdpidx, int action);
mz_error_t mz_raw_at_test_CNACT(void);
mz_error_t mz_raw_at_read_CNACT(void);
mz_error_t mz_raw_at_write_CNCFG(int pdpidx, int ip_type, char * APN, char * usename, char * password, int authentication);
mz_error_t mz_raw_at_test_CNCFG(void);
mz_error_t mz_raw_at_read_CNCFG(void);


#endif // (MZ_MODEM_SIMCOM == MZ_ENABLE)
#ifdef __cplusplus
}
#endif
#endif /* MZ_SIMCOM_AT_INCLUDE_H_ */
