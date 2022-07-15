/** 
 * @file MZ_Lwm2m_public.h
 *  @date Jun 7, 2021
 *  @author SKM
 * @brief Monoz LWM2M public Protocol
 */

#ifndef MZ_LWM2M_PUBLIC_H_
#define MZ_LWM2M_PUBLIC_H_									///< Defines Monoz LWM2M Public	
#ifdef __cplusplus
extern "C" {
#endif
#include "MZ_error_handler.h"

#if(MZ_LWM2M_ENABLE == MZ_ENABLE)

/** @enum en_lw_event
 * @brief Enumeration of LW Events
 */ 
typedef enum
{
	LW_EV_WRITE_DATA=0, 									/*!< Write operation was received */
	LW_EV_EXECUTE = 1, 										/*!< Execute operation was received */
	LW_EV_WRITE_ATTRIBUTE = 4,								/*!< Write Attributes operation was received */
	LW_EV_DISCOVER = 5, 									/*!< Discover operation was received */
	LW_EV_READ = 6, 										/*!< Read operation was received */
	LW_EV_OBSERVE = 7, 										/*!< Observe operation was received */
	LW_EV_OBSERVE_CANCEL = 8,								/*!< Cancel observation operation was received */
	LW_EV_CLIENT_OFFLINE = 9,								/*!< Client is offline */
	LW_EV_CLIENT_ONLINE = 10,								/*!< Client is online */
	LW_EV_CLIENT_NOTIFY_SEND = 11,							/*!< Client sent observation notification to a server */
	LW_EV_CLIENT_WKP_SMS = 12,								/*!< Client received wakeup SMS */
	LW_EV_CLIENT_NOTIFY_SEND_ACK = 13,						/*!< Client received notification acknowledge */
	LW_EV_CLIENT_ON = 14,									/*!< Client ON: LMM2M client exits Client OFF state and tries to re-connect server due to explicitly AT Command registration request */
	LW_EV_CLIENT_OFF = 15,									/*!< Client OFF: LWM2M client has exhausted server connection retries */
	LW_EV_NOTIFY_FAIL = 16,									/*!< Confirmable NOTIFY failed */
	LW_EV_BOOTSTRAP_COMPLETE = 20,							/*!< Bootstrap finished and completed successfully */
	LW_EV_REG_FINISH = 21,									/*!< Registration finished and completed successfully. All server observation requests are cleared in FW; the host need to clear host object observations */
	LW_EV_REG_UPDATE_FINISH = 22,							/*!< Register update finished and completed successfully */
	LW_EV_DEREG_FINISH = 23,								/*!< De-register finished and completed successfully */
	LW_EV_NOTIFY_NOT_SEND = 24, 							/*!< Notification was not saved and not sent to server */
	LW_EV_ENABLE_ALL = 100, 								/*!< enable all notifications */
}en_lw_event;


/** @struct st_lw_event
 * @brief Structure For LW Events
 */ 
typedef struct __attribute__((packed))
{
	en_lw_event 	lw_event;								/*!< unsolicited event */
	int				lw_ssc_id;								/*!< serverShortId */
	int				lw_Obj_id;								/*!< ObjectID */
	int				lw_Obj_Ins_id;							/*!<  ObjectInstanceID - Optional */
	int				lw_Res_id;								/*!< ResourceID - Optional */
	int				lw_Res_Ins_id;							/*!< ResourceInstanceID - Optional */
	char			lw_val[512];							/*!< Val - Optional - Max size 511 Bytes */
	int				lw_MsgId;								/*!< COAP message ID (for NOTIFY event) */
}st_lw_event;

/** 
 * @defgroup LWM2M_PROTOCOL LWM2M PROTOCOL
 * LWM2M Protocol
 * @{
 */  
/* @defgroup READ_TOKEN READ TOKEN
 * Read the Token value
 * @{
 */ 
/**
 * @fn void * mz_read_token_Ob19_0_0(mz_error_t * ret)
 * @brief Read the token value of 19/0/0.
 * @param ret mz_error_t
 * @note If no valid token, then ret = MZ_LW_INVALID_TOKEN
 * Optimized by returning the internal reference of token.
 * @warning DONOT change the values using the return pointer. Only use it for copy/compare purpose.
 */
void * mz_read_token_Ob19_0_0(mz_error_t * ret);

/** @fn void * mz_read_token_Ob19_1_0(mz_error_t * ret)
 * @brief Read the token value of 19/1/0.
 * @param ret mz_error_t
 * @note If no valid token, then ret = MZ_LW_INVALID_TOKEN
 * Optimized by returning the internal reference of token.
 * @warning DONOT change the values using the return pointer. Only use it for copy/compare purpose.
 */
void * mz_read_token_Ob19_1_0(mz_error_t * ret);
/* @} */

/*
 * @defgroup SET_VALUE  SET_VALUE
 * Set the Token value
 * @{
 */ 
/** @fn mz_error_t mz_set_value_Ob19_0_0(char * src)
 * @brief write 19/0/0 value to internal resource structure
 * @param src char
 * @return MZ_OK/MZ_FAIL
 * @note The value will be send to LWM2M server when LWM2M READ or NOTIFY event occurs
 */
mz_error_t mz_set_value_Ob19_0_0(char * src);

/** @fn mz_error_t mz_set_value_Ob19_1_0(char * src)
 * @brief write 19/1/0 value to internal resource structure
 * @param src char
 * @return MZ_OK/MZ_FAIL
 * @note The value will be send to LWM2M server when LWM2M READ or NOTIFY event occurs
 */
mz_error_t mz_set_value_Ob19_1_0(char * src);
/* @} */

/*
 * @defgroup READ_VALUE READ_VALUE
 * Read resource value
 * @{
 */
/** 
 * @fn void * mz_read_value_Ob19_0_0(mz_error_t * ret)
 * @brief Read internal resource structure value of 19/0/0
 * @param ret mz_error_t
 * @note Optimized by returning the internal reference of token.
 * @warning  DONOT change the values using the return pointer. Only use it for copy/compare purpose.
 */
void * mz_read_value_Ob19_0_0(mz_error_t * ret);

/**
 * @fn void * mz_read_value_Ob19_1_0(mz_error_t * ret) 
 * @brief Read internal resource structure value of 19/1/0
 * @param ret mz_error_t
 * @note NOTE : Optimized by returning the internal reference of token.
 * @warning Caution : DONOT change the values using the return pointer. Only use it for copy/compare purpose.
 */
void * mz_read_value_Ob19_1_0(mz_error_t * ret);
/* @} */

/*
 * @defgroup NOTIFY NOTIFY
 * Notify resource structure
 * @{
 */
/** 
 * @fn mz_error_t mz_notify_Ob19_0_0(void)
 * @brief Notify internal resource structure value of 19/0/0 to LWm2M server
 * @return MZ_OK/MZ_FAIL
 * @note  use mz_set_value_Ob19_0_0() API to set the values first before notify.
 * If values are not set then this API call will use old stored value to NOTIFY to the server.
 * The user can use mz_set_and_notify_Ob19_0_0() API to set the value and then notify to server.
 */
mz_error_t mz_notify_Ob19_0_0(void);

/** @fn mz_error_t mz_notify_Ob19_1_0(void) 
 * @brief Notify internal resource structure value of 19/1/0 to LWm2M server
 * @return MZ_OK/MZ_FAIL
 * @note  use mz_set_value_Ob19_1_0() API to set the values first before notify.
 * If values are not set then this API call will use old stored value to NOTIFY to the server.
 * The user can use mz_set_and_notify_Ob19_1_0() API to set the value and then notify to server.
 */
mz_error_t mz_notify_Ob19_1_0(void);
/* @} */

/* @defgroup SET_AND_NOTIFY SET_AND_NOTIFY
 * Set and Notify Resource structure
 * @{
 */
/** 
 * @fn mz_error_t mz_set_and_notify_Ob19_0_0(char * src)
 * @brief write 19/0/0 value to internal resource structure and then notify to server.
 * @param src char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_set_and_notify_Ob19_0_0(char * src);

/** @fn mz_error_t mz_set_and_notify_Ob19_1_0(char * src) 
 * @brief write 19/1/0 value to internal resource structure and then notify to server.
 * @param src char
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t mz_set_and_notify_Ob19_1_0(char * src);
/* @} */
/** @} */

#endif // (MZ_LWM2M_ENABLE == MZ_ENABLE)
#ifdef __cplusplus
}
#endif
#endif /* MZ_LWM2M_PUBLIC_H_ */
