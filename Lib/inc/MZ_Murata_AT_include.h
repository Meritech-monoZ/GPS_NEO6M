/**
 * @file MZ_Murata_AT_include.h
 * @author SKM
 * @date Jun 4 2021 
 * @brief Monoz Murata AT Include
 */

#ifndef MZ_MURATA_AT_INCLUDE_H_
#define MZ_MURATA_AT_INCLUDE_H_                             ///< Define Monoz Murata AT Command Include
#ifdef __cplusplus
extern "C" {
#endif
#include "MZ_error_handler.h"

#if(MZ_MODEM_MURATA == MZ_ENABLE)
/** Standard AT CMD's */

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

/** Murata specific AT CMD's */

/**
 * @fn mz_error_t mz_raw_at_test_MEAS(void)
 * @brief Issue AT+MEAS=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_MEAS(void);

/**
 * @fn mz_error_t mz_raw_at_read_MEAS(void)
 * @brief It request to read Returns measurement for specified measurement type
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_MEAS(void);

/**
 * @fn mz_error_t mz_raw_at_write_MEAS(char * meas_type)
 * @brief This command is used to select measurement for specified measurement type
 * @param meas_type char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_MEAS(char * meas_type);

/**
 * @fn mz_error_t mz_raw_at_test_SETACFG(void)
 * @brief Issue AT+SETACFG=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_SETACFG(void);

/**
 * @fn mz_error_t mz_raw_at_read_SETACFG(void)
 * @brief It request to read Set a configuration parameter
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_SETACFG(void);

/**
 * @fn mz_error_t mz_raw_at_write_SETACFG(char *param_file_name,char *param_section, char *param_name, char *param_value)
 * @brief Set a configuration parameter
 * @param param_file_name char
 * @param param_section char
 * @param param_name char 
 * @param param_value char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_SETACFG(char *param_file_name,char *param_section, char *param_name, char *param_value);

/**
 * @fn mz_error_t mz_raw_at_test_GETACFG(void)
 * @brief Issue AT+GETACFG=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_GETACFG(void);

/**
 * @fn mz_error_t mz_raw_at_read_GETACFG(void)
 * @brief It request to read Get a configuration parameter
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_GETACFG(void);

/**
 * @fn mz_error_t mz_raw_at_write_GETACFG(char *param_file_name, char *param_section, char *param_name)
 * @brief Gets a configuration field
 * @param param_file_name char -<pm>
 * @param param_section char -<conf>
 * @param param_name char -<sleep_mode>
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_GETACFG(char *param_file_name, char *param_section, char *param_name);

/**
 * @fn mz_error_t mz_raw_at_test_LWM2MCMD(void)
 * @brief Issue AT+LWM2MCMD=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_LWM2MCMD(void);

/**
 * @fn mz_error_t mz_raw_at_read_LWM2MCMD(void)
 * @brief It request to read Control the LWM2M client
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_LWM2MCMD(void);

/**
 * @fn mz_error_t mz_raw_at_write_LWM2MCMD(char *str1,mzUint8 val1, mzUint8 val2, mzUint8 val3,mzUint8 val4 , char *val5)
 * @brief This command is used to control the LWM2M client
 * @param str1 char
 * @param val1 mzUint8
 * @param val2 mzUint8
 * @param val3 mzUint8
 * @param val4 mzUint8
 * @param val5 char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_LWM2MCMD(char *str1,mzUint8 val1, mzUint8 val2, mzUint8 val3,mzUint8 val4 , char *val5);

/**
 * @fn mz_error_t mz_raw_at_test_LWM2MOPEV(void)
 * @brief Issue AT+LWM2MOPEV=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_LWM2MOPEV(void);

/**
 * @fn mz_error_t mz_raw_at_read_LWM2MOPEV(void)
 * @brief It request to read Control LWM2M URC
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_LWM2MOPEV(void);

/**
 * @fn mz_error_t mz_raw_at_write_LWM2MOPEV(mzUint8 event, mzUint8 mode)
 * @brief Control LWM2M URC
 * @param event mzUint8 -(0-25,99,100)
 * @param mode mzUint8 -(0,1)
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_LWM2MOPEV(mzUint8 event, mzUint8 mode);

/**
 * @fn mz_error_t mz_raw_at_test_LWM2MOBJCMD(void)
 * @brief Issue AT+LWM2MOBJCMD=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_LWM2MOBJCMD(void);

/**
 * @fn mz_error_t mz_raw_at_read_LWM2MOBJCMD(void)
 * @brief It request to read Manage %LWM2MCMD URC for LWM2M host objects
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_LWM2MOBJCMD(void);

/**
 * @fn mz_error_t mz_raw_at_write_LWM2MOBJCMD(mzUint8 mode)
 * @brief This command is used to control the LWM2M client
 * @param mode mzUint8
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_LWM2MOBJCMD(mzUint8 mode);

/**
 * @fn mz_error_t mz_raw_at_test_LWM2MOBJRSP(void)
 * @brief Issue AT+LWM2MOBJRSP=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_LWM2MOBJRSP(void);

/**
 * @fn mz_error_t mz_raw_at_read_LWM2MOBJRSP(void)
 * @brief It request to read Provide host application response for %LWM2MCMD URC
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_LWM2MOBJRSP(void);

/**
 * @fn mz_error_t mz_raw_at_write_LWM2MOBJRSP(mzUint8 retcode, mzUint8 seq_code,char *uri,char *val)
 * @brief Command is used to provide Host application response for %LWM2MCMDU
 * @param retcode mzUint8
 * @param seq_code mzUint8
 * @param uri char
 * @param val char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_LWM2MOBJRSP(mzUint8 retcode, mzUint8 seq_code,char *uri,char *val);

/**
 * @fn mz_error_t mz_raw_at_test_LWM2MOBJEV(void)
 * @brief Issue AT+LWM2MOBJEV=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_LWM2MOBJEV(void);

/**
 * @fn mz_error_t mz_raw_at_read_LWM2MOBJEV(void)
 * @brief It request to read Send notification with resource value for LWM2M host objects
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_LWM2MOBJEV(void);

/**
 * @fn mz_error_t mz_raw_at_write_LWM2MOBJEV(char token,mzUint16 server_id,char format,char fragment_info,char *uri,char *value)
 * @brief This command is used to control the LWM2M client
 * @param token char
 * @param server_id mzUint16
 * @param format char
 * @param fragment_info char
 * @param uri char
 * @param value char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_LWM2MOBJEV(char token,mzUint16 server_id,char format,char fragment_info,char *uri,char *value);

/**
 * @fn mz_error_t mz_raw_at_test_DNSRSLV(void)
 * @brief Issue AT+DNSRSLV=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_DNSRSLV(void);

/**
 * @fn mz_error_t mz_raw_at_read_DNSRSLV(void)
 * @brief It request to read Resolve specific domain name
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_DNSRSLV(void);

/**
 * @fn mz_error_t mz_raw_at_write_DNSRSLV(char Rsvd,char *domain_name)
 * @brief Resolve Specific Domain name
 * @param Rsvd char -<Rsvd(1)>
 * @param domain_name char -<domain_name()>
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_DNSRSLV(char Rsvd,char *domain_name);

/**
 * @fn mz_error_t mz_raw_at_test_PINGCMD(void)
 * @brief Issue AT+PINGCMD=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_PINGCMD(void);

/**
 * @fn mz_error_t mz_raw_at_read_PINGCMD(void)
 * @brief It request to read Ping specified address
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_PINGCMD(void);

/**
 * @fn mz_error_t mz_raw_at_write_PINGCMD(char id,char ip_type,char dst_ip,char count,int packetsize,mzUint8 timeout)
 * @brief This command is used for executing PING services
 * @param id char -(1-20)
 * @param ip_type char -(0,1)
 * @param dst_ip char -<"dest ip address>
 * @param count char -(1-20)
 * @param packetsize int -56
 * @param timeout int -(1-60)
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_PINGCMD(char id,char ip_type,char dst_ip,char count,int packetsize,mzUint8 timeout);

/**
 * @fn mz_error_t mz_raw_at_test_SOCKETCMD(void)
 * @brief Issue AT+SOCKETCMD=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_SOCKETCMD(void);

/**
 * @fn mz_error_t mz_raw_at_read_SOCKETCMD(void)
 * @brief It request to read Create and maintain sockets
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_SOCKETCMD(void);

/**
 * @fn mz_error_t mz_raw_at_test_SOCKETDATA(void)
 * @brief Issue AT+SOCKETDATA=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_SOCKETDATA(void);

/**
 * @fn mz_error_t mz_raw_at_read_SOCKETDATA(void)
 * @brief It request to read Send/receive data to/from the socket
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_SOCKETDATA(void);

//mz_error_t mz_raw_at_write_SOCKETDATA(void);

/**
 * @fn mz_error_t mz_raw_at_test_SOCKETEV(void)
 * @brief Issue AT+SOCKETREV=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_SOCKETEV(void);

/**
 * @fn mz_error_t mz_raw_at_read_SOCKETEV(void)
 * @brief It request to read Handle socket events
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_SOCKETEV(void);

//mz_error_t mz_raw_at_write_SOCKETEV(void);

/**
 * @fn mz_error_t mz_raw_at_test_CERTCMD(void)
 * @brief Issue AT+CERTCMD=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_CERTCMD(void);

/**
 * @fn mz_error_t mz_raw_at_read_CERTCMD(void)
 * @brief It request to read Manage user certificates in NV
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_CERTCMD(void);

/**
 * @fn mz_error_t mz_raw_at_write_CERTCMD(char *cmd,char *filename,char type,char *data)
 * @brief Manage [read/write/delete/list] user certificates in NV
 * @param cmd char
 * @param filename char
 * @param type char
 * @param data char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_CERTCMD(char *cmd,char *filename,char type,char *data);

/**
 * @fn mz_error_t mz_raw_at_test_CERTCFG(void)
 * @brief Issue AT+CERTCFG=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_CERTCFG(void);

/**
 * @fn mz_error_t mz_raw_at_read_CERTCFG(void)
 * @brief It request to read Manage certificate profile configuration file
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_CERTCFG(void);

/**
 * @fn mz_error_t mz_raw_at_write_CERTCFG(char *op,char profile_id,char *ca_file,char *ca_path,char *dev_cert,char *dev_key)
 * @brief Meritech Specific AT Command
 * @param op char
 * @param profile_id char
 * @param ca_file char
 * @param ca_path char
 * @param dev_cert char
 * @param dev_key char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_CERTCFG(char *op,char profile_id,char *ca_file,char *ca_path,char *dev_cert,char *dev_key);

/**
 * @fn mz_error_t mz_raw_at_test_PDNSET(void)
 * @brief Issue AT+PDNSET=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_PDNSET(void);

/**
 * @fn mz_error_t mz_raw_at_read_PDNSET(void)
 * @brief It request to read Set the run-time PDN parameters for PDNs that are exposed to host
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_PDNSET(void);

/**
 * @fn mz_error_t mz_raw_at_write_PDNSET(char ext_session_id,char *apnname,char *ip_type)
 * @brief set the run-time PDN parameters for data PDNs that are exposed to host
 * @param ext_session_id char -(1)
 * @param apnname char -(APP)
 * @param ip_type char -(IP,IPV6,IPV4V6)
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_PDNSET(char ext_session_id,char *apnname,char *ip_type);

/**
 * @fn mz_error_t mz_raw_at_test_PDNRDP(void)
 * @brief Issue AT+PDNRDP=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_PDNRDP(void);

/**
 * @fn mz_error_t mz_raw_at_read_PDNRDP(void)
 * @brief It request to read Get the relevant information for an active PDN
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_PDNRDP(void);

/**
 * @fn mz_error_t mz_raw_at_write_PDNRDP(char ext_session_id);
 * @brief This command is used to get the relevant information for an active PDN
 * @param ext_session_id char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_PDNRDP(char ext_session_id);

/**
 * @fn mz_error_t mz_raw_at_test_TESTCFG(void)
 * @brief Issue AT+TESTCFG=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t mz_raw_at_test_TESTCFG(void);

/**
 * @fn mz_error_t mz_raw_at_read_TESTCFG(void)
 * @brief It request to read Configure LWM2M server parameters
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_TESTCFG(void);

/**
 * @fn mz_error_t mz_raw_at_write_TESTCFG(char *test_mode,char *param1,char *param2,char *param3,char *param4)
 * @brief Meritech Specific AT Command
 * @param test_mode char
 * @param param1 char
 * @param param2 char
 * @param param3 char
 * @param param4 char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_write_TESTCFG(char *test_mode,char *param1,char *param2,char *param3,char *param4);

/**
 * @fn mz_error_t mz_raw_at_test_PDNACT(void)
 * @brief Issue AT+PDNACT=? command to test the this command is supported or not
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_test_PDNACT(void);

/**
 * @fn mz_error_t mz_raw_at_read_PDNACT(void)
 * @brief It request to read Activate specified PDN defined by %PDNSET
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_raw_at_read_PDNACT(void);

/**
 * @fn mz_error_t mz_raw_at_write_PDNACT(char act,char ext_session_id,char *APN)
 * @brief  This command is used to active specified PDN
 * @param act char -(0,1)
 * @param ext_session_id char -0
 * @param APN char -<apname>
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t mz_raw_at_write_PDNACT(char act,char ext_session_id,char *APN);

/**
 * @fn mz_error_t mz_raw_at_test_DEVCFG(void)
 * @brief  AT command used to test the device configuration of Murata.
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t mz_raw_at_test_DEVCFG(void);

/**
 * @fn mz_error_t mz_raw_at_read_DEVCFG(void)
 * @brief  AT command used to read the device configuration of Murata.
 * The command supports the selection of IP and NIDD mode
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t mz_raw_at_read_DEVCFG(void);

/**
 * @fn mz_error_t mz_raw_at_write_DEVCFG(char *cmd,char *object,char *value1)
 * @brief This command is used tosupport the selection of IP and NIDD mode in NB-IOT
 * @param cmd char -(wr,rd)
 * @param object char -DDMODE
 * @param value1 char -(IP,NIDD)
 * @return MZ_OK/MZ_FAIL
 */ 
mz_error_t mz_raw_at_write_DEVCFG(char *cmd,char *object,char *value1);

/**
 * @fn void mz_create_cmd_timeout(mzInt32 _to)
 * @brief creates the command timeout
 * @param _to mzInt32
 */
void mz_create_cmd_timeout(mzInt32 _to);

/**
 * @fn void mz_create_cmd_time_wait_before_next(mzInt32 _twn)
 * @brief creates the command time wait before next command
 * @param _twn mzInt32
 */
void mz_create_cmd_time_wait_before_next(mzInt32 _twn);

/** @} */

#endif // (MZ_MODEM_MURATA == MZ_ENABLE)

#ifdef __cplusplus
}
#endif
#endif /* MZ_MURATA_AT_INCLUDE_H_ */
