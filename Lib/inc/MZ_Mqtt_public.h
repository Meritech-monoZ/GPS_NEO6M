/*
 * MZ_Mqtt_public.h
 *
 *  Created on: Oct 6, 2021
 *      Author: SKM
 */

#ifndef MZ_MQTT_PUBLIC_H_
#define MZ_MQTT_PUBLIC_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "MZ_common.h"
#include "MZ_error_handler.h"
#if(MZ_MQTT_ENABLE == MZ_ENABLE)

typedef enum
{
	MQTT_EV_DISCONNECT = 0,														/*!< Expression MQTT disconnect state */
	MQTT_EV_ON_LINE = 1,														/*!< Expression MQTT on-line state */
	MQTT_EV_ON_LINE_SESSION = 2,												/*!< Expression MQTT on-line state and SP(Session Present) flag is set */
}en_mqtt_state;

typedef enum
{
	MQTT_PUB_NORMAL = 0,														/*!< PUB data format is normal */
	MQTT_PUB_HEX = 1,															/*!< PUB data format is hex */
}en_mqtt_pubhex;

typedef enum
{
	MQTT_SESSION_OLD = 0,														/*!< Resume communication based on persent session */
	MQTT_SESSION_NEW = 1,														/*!< Resume communication with a new session */
}en_mqtt_cleanss;

typedef enum
{
	MQTT_QOS0 = 0,																/*!< At most once */
	MQTT_QOS1 = 1,																/*!< At lease once */
	MQTT_QOS2 = 2,																/*!< Only once */
}en_mqtt_qos;

typedef enum
{
	MQTT_RETAIN_OFF = 0,														/*!< Message will not be saved or removed or replaced */
	MQTT_RETAIN_ON = 1,															/*!< Message and its <qos> will be saved */
}en_mqtt_retain;

typedef enum
{
	MQTT_SUB_NORMAL = 0,														/*!< SUB data format is normal */
	MQTT_SUB_HEX = 1,															/*!< SUB data format is hex */
}en_mqtt_subhex;

typedef enum
{
	MQTT_SYNCMODE = 0,															/*!< The MQTT connection uses synchronous mode */
	MQTT_ASYNCMODE = 1,															/*!< The MQTT connection uses asynchronous mode */
}en_mqtt_conn_mode;

#define MZ_MQTT_CONFIG_CLIENT_ID_LEN			(128)
#define MZ_MQTT_CONFIG_URL_LEN					(246)
#define MZ_MQTT_CONFIG_USERNAME_LEN				(256)
#define MZ_MQTT_CONFIG_PASSWORD_LEN				(512)
#define MZ_MQTT_CONFIG_TOPIC_LEN				(128)
#define MZ_MQTT_CONFIG_MESSAGE_LEN				(1024)

typedef struct __attribute__((packed))
{
	mzUint8				config_id;												/*!< MQTT configuration id */
//	char 				client_id[MZ_MQTT_CONFIG_CLIENT_ID_LEN];				/*!< MQTT client id */
//	char				server_url[MZ_MQTT_CONFIG_URL_LEN];						/*!< MQTT server URL / IP */
	char 				*client_id;												/*!< MQTT client id */
	char				*server_url;											/*!< MQTT server URL / IP */
	mzUint16			server_port;											/*!< MQTT server port, Range: 0-65565 */
	mzUint16			keep_alive_time;										/*!< MQTT client server connection keep alive time, Range: 0-65565 */
	en_mqtt_cleanss		session_cleanss;										/*!< MQTT session cleanss */
	char 				*username;												/*!< MQTT server username */
	char				*password;												/*!< MQTT server password */
//	char 				username[MZ_MQTT_CONFIG_USERNAME_LEN];					/*!< MQTT server username */
//	char				password[MZ_MQTT_CONFIG_PASSWORD_LEN];					/*!< MQTT server password */
//	en_mqtt_qos			qos;													/*!< MQTT QOS */
//	char				topic[MZ_MQTT_CONFIG_TOPIC_LEN];						/*!< MQTT TOPIC */
//	char				message[MZ_MQTT_CONFIG_MESSAGE_LEN];					/*!< MQTT MESSAGE */
//	en_mqtt_retain		retain;													/*!< MQTT Retain Identification */
	en_mqtt_subhex		subhex;													/*!< MQTT SUB data format */
	en_mqtt_conn_mode	connect_mode;											/*!< MQTT Connection Mode */
}st_mqtt_client_config;

typedef struct __attribute__((packed))
{
	en_mqtt_qos			qos;													/*!< MQTT QOS */
//	char				topic[MZ_MQTT_CONFIG_TOPIC_LEN];						/*!< MQTT TOPIC */
//	char				message[MZ_MQTT_CONFIG_MESSAGE_LEN];					/*!< MQTT MESSAGE */
	char				*topic;													/*!< MQTT TOPIC */
	char				*message;												/*!< MQTT MESSAGE */
	en_mqtt_retain		retain;													/*!< MQTT Retain Identification */
}st_mqtt_message;

typedef struct __attribute__((packed))
{
	en_mqtt_state 			state;												/*!< MQTT state event */
	st_mqtt_message			msg;												/*!< Message associated by the even */
}st_mqtt_event;

mz_error_t mz_mqtt_client_configure(st_mqtt_client_config * config);

mz_error_t mz_mqtt_client_connect(mzUint8 config_id);

mz_error_t mz_mqtt_client_disconnect(mzUint8 config_id);

mz_error_t mz_mqtt_client_reset(mzUint8 config_id);

mz_error_t mz_mqtt_client_remove(mzUint8 config_id);

mz_error_t mz_mqtt_pub(st_mqtt_message * msg);

mz_error_t mz_mqtt_sub(st_mqtt_message * msg);

mz_error_t mz_mqtt_unsub(st_mqtt_message * msg);

#endif // (MZ_MQTT_ENABLE == MZ_ENABLE)
#ifdef __cplusplus
}
#endif
#endif /* MZ_MQTT_PUBLIC_H_ */
