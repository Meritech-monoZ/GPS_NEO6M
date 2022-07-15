/*
 * Objectives covered in this Example :
 * 1. Demonstrate how to create a UART instance, configure and integrate the
 *    UART instance with MonoZ_Lib
 * 2. Demonstrate how to use UART in interrupt mode for receiving data
 * 3. Demonstrate how to create a Application Thread using MonoZ_Lib
 * 4. Demonstrate how to create single-shot and recursive timers, and use them.
 * 5. Demonstrate how to use lwm2m API's defined in MonoZ_Lib to set
 * 	  Object/Resource values using mz_set_value_Ob19_X_X API,
 * 	  Read the new data received from server(in write event) using
 * 	  mz_read_value_Ob19_X_X API, Send the sensor data using
 * 	  mz_set_and_notify_Ob19_X_X API.
 * 6. Demonstrate how to handle post processing event from MonoZ_Lib lwm2m
 * 	  client.
 * 7. Demonstrate how to force modem re-initialization.
 * 8. Demonstrate how to use CLI for printing informations.
 *
 * Sources for this example :
 * MZ_GPSSensor.c	- Main Application, UART Driver and sensor Driver
 * MZ_GPSSensor.h
 *
 * [Other Linked files]
 * MZ_hardware_config.c - Hardware Driver integration file
 * MZ_modem_config.c 	- Modem command integration file
 */


/** @file MZ_GPSSensor.c
 * @date JULY 11, 2022
 * @author Meritech
 * @brief This is the example for read the GPS sensor data and send it to platform
 */

/* Include Header Files - START */
#include <MZ_GPSSensor.h>
#include "MZ_sys_cmsis_os2.h"
#include "MZ_error_handler.h"
#include "MZ_timer.h"
#include "MZ_print.h"
#include "MZ_Mqtt_public.h"
#include "MZ_type_converter.h"
#include "MZ_Modem_public.h"
#include "MZ_uart.h"
#include "MZ_main.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

/* Include Header Files - END */

/* Define some common use MACRO - START */
#define GPS_READ_TIMER_EXPIRE_SET				(1)						///< Set the Timer Expire for GPS read
#define GPS_READ_TIMER_EXPIRE_CLEAR				(0)						///< Clear the Timer Expire for GPS read
#define UART_RECEIVE_COMPLETE_SET				(0x00)					///< Sets the UART Receive 
#define UART_RECEIVE_COMPLETE_CLEAR				(0xFF)					///< Clear the UART Receive
#define TIME_180SEC								(pdMS_TO_TICKS(180000))	///< Timer is set for 180 seconds
#define TIME_90SEC								(pdMS_TO_TICKS(90000))	///< Timer is set for 90 seconds
#define TIMER_ID_CLEAR							(0)						///< Clear the timer id
/* Define some common use MACRO - END */

/* Thread related MACRO and variables - START */
#define GPS_APP_STACK_SIZE			(1024)										///< Stack size for the thread */
static mz_thread_t 					gps_thread_id = NULL;						/*!< Thread id handler */
static StaticTask_t 				gps_cb_mem;									/*!< Thread control block */
static StackType_t 					gps_stack[GPS_APP_STACK_SIZE];				/*!< Thread stack */
/* Thread related MACRO and variables - END */

/* Timer related MACRO and variables - START */
#define GPS_SENSOR_READ_TIME					(TIME_90SEC)			///< Set 90 seconds timer for read sensor data
static char gps_read_timer_expire_flag = GPS_READ_TIMER_EXPIRE_CLEAR;	/*!< Flag is created and cleared for sensor data read */
/* Timer related MACRO and variables - END */

/* GPS sensor related MACRO and variables - START */
static volatile char  final_lat_data[40] = {0};					/*!< Store final gps value after calculation from raw value and decimal place */
static volatile char  final_lon_data[40] = {0};					/*!< Store final gps value after calculation from raw value and decimal place */
static volatile char  final_pdop_data[10] = {0};				/*!< Store final gps value after calculation from raw value and decimal place */
static volatile char  final_hdop_data[10] = {0};				/*!< Store final gps value after calculation from raw value and decimal place */
static volatile char  final_vdop_data[10] = {0};				/*!< Store final gps value after calculation from raw value and decimal place */

//static volatile char dummy_data[20] = {0};
/* GPS sensor related MACRO and variables - END */

/* MQTT related MACRO and variables - START */
static char payload_string[200] = "";											/*!< payload string buffer to pass the final payload information to MonoZ_Lib */
static st_mqtt_message pmsg;
/* MQTT related MACRO and variables - END */

/* CRC related MACRO and variables - START */
#define CRC_TABLE_SIZE 256											///< Defines the crc table size
#define CRC16_POLY 0x8005											///< Defines the CRC16_POLY	value
#define CRC16_INIT_REM 0x0000										///< Defines the CRC16_INIT_REM value
#define CRC16_FINAL_XOR 0x0000										///< Defines the CRC16_FINAL_XOR value
extern const unsigned short crc16Table[CRC_TABLE_SIZE];				
/* CRC related MACRO and variables - END */

static mz_error_t gps_uart_init(void);
static void GPS_SENSOR_READ_TIMEr_cb(TimerHandle_t xTimer);
static void create_mqtt_payload(st_mqtt_message * pmsg , char * buff);
static void send_payload_to_server(st_mqtt_message * pmsg);
static void gps_app_thread(void * arg);
static unsigned short crc16MakeTableMethod(	unsigned short crc,
											const unsigned short *table,
											unsigned char *pbuffer,
											unsigned int length);
/* static function prototypes - END */

/*============================================================================*/
/* GPS Sensor board - UART related code - START                                */
/*============================================================================*/

/* GPS UART configuration related MACRO - START */
#define MZ_GPS_INSTANCE 					(LPUART1)						///< Defines the instance
#define MZ_GPS_INIT_BAUDRATE				(9600)//(115200) //(57600)		///< Defines the baud rate
#define MZ_GPS_INIT_WORDLENGTH				(UART_WORDLENGTH_8B)			///< Defines the initial word length
#define MZ_GPS_INIT_STOPBITS				(UART_STOPBITS_1)				///< Defines the stop bits
#define MZ_GPS_INIT_PARITY					(UART_PARITY_NONE)				///< Defines the initial parity
#define MZ_GPS_INIT_MODE 					(UART_MODE_TX_RX)				///< Defines the mode
#define MZ_GPS_INIT_HWFLOWCTL 				(UART_HWCONTROL_NONE)			///< Defines initial hardware flow control
#define MZ_GPS_INIT_OVERSAMPLING 			(UART_OVERSAMPLING_16)			///< Defines the initial oversampling
#define MZ_GPS_INIT_ONEBITSAMPLING 			(UART_ONE_BIT_SAMPLE_DISABLE)	///< Defines the initial one bit sampling
#define MZ_GPS_ADVANCEDINIT_ADVFEATUREINIT  (UART_ADVFEATURE_NO_INIT)		///< Defines the Uart advance features
#define MZ_GPS_UART_INSTANCE				(_LPUART1)						///< Defimes the UART instance
/* GPS UART configuration related MACRO - START */

#define MZ_MQTT_PUB_TOPIC 		"\"v1/devices/me/telemetry\""
#define MZ_MQTT_PUB_QOS			MQTT_QOS0
#define MZ_MQTT_SUB_TOPIC		"\"v1/devices/me/attributes\""
#define MZ_MQTT_SUB_QOS			MQTT_QOS2
#define MZ_MZTT_KEY1			"latitude"
#define MZ_MZTT_KEY2			"longitude"
#define MZ_MZTT_KEY3			"PDOP"
#define MZ_MZTT_KEY4			"HDOP"
#define MZ_MZTT_KEY5			"VDOP"

//#define MZ_MZTT_DUMMYKEY		"dummydata"
/* gps UART configuration structure - START */
/*
 * This configuration structure is used in MZ_hardware_config.c to initialize
 * the UART hardware interface.
 * In this example this Configuration is taken during the Initialization of
 * the MonoZ_Lib to load the settings prior to Application start.
 *
 * NOTE:
 * In-case Application want to perform UART initialization later after
 * application start or by itself during runtime, MZ_UART_init() API can be
 * used.
 * In such case remove the configuration from MZ_hardware_config.c.
 */
MZ_UART_INIT_ST gps_lpuart1_instance =
{
	.Instance = MZ_GPS_INSTANCE,
	.Init.BaudRate = MZ_GPS_INIT_BAUDRATE,
	.Init.WordLength = MZ_GPS_INIT_WORDLENGTH,
	.Init.StopBits = MZ_GPS_INIT_STOPBITS,
	.Init.Parity = MZ_GPS_INIT_PARITY,
	.Init.Mode = MZ_GPS_INIT_MODE,
	.Init.HwFlowCtl = MZ_GPS_INIT_HWFLOWCTL,
	.Init.OverSampling = MZ_GPS_INIT_OVERSAMPLING,
	.Init.OneBitSampling = MZ_GPS_INIT_ONEBITSAMPLING,
	.AdvancedInit.AdvFeatureInit = MZ_GPS_ADVANCEDINIT_ADVFEATUREINIT
};
/* GPS UART configuration structure - END */

/* GPS UART related variables - START */
static volatile char gps_uart_recv_complete_flag = UART_RECEIVE_COMPLETE_CLEAR; /*!< Flag to hold if the uart receive is complete or not */
/* GPS UART related variables - END */

/** @fn static void gps_lpuart1_rx_intr(void * arg)
 * @brief GPS UART related callback - START
 * This UART callback will be called after completion of uart receive.
 * @param arg void 
 */
static void gps_lpuart1_rx_intr(void * arg)
{
	(void)arg;

	/* Set gps sensor uart receive complete flag */
	gps_uart_recv_complete_flag = UART_RECEIVE_COMPLETE_SET;
}
/* GPS UART related callback - END */

/** @fn static mz_error_t gps_uart_init(void)
 *  @brief GPS UART related initialization - START
 * This Timer callback will be called after the server monitoring timer is
 * expired.The timer value is set as LWM2M_SERVER_MONITORING_TIME
 * @return MZ_OK/MZ_FAIL
 */
static mz_error_t gps_uart_init(void)
{
	/*
	 * Register the lpuart1 receive complete callback using
	 * MZ_UART_register_intr_cb_rx() API
	 * When receive is completed on the lpuart interface,
	 * the callback API gps_lpuart1_rx_intr() will be processed.
	 */
	return MZ_UART_register_intr_cb_rx(	MZ_GPS_UART_INSTANCE,
										gps_lpuart1_rx_intr);

	/*
	 * Based on the Application requirement register a transmit callback using
	 * MZ_UART_register_intr_cb_tx() API. Please refer MZ_uart.h for more
	 * details.
	 */
}
/* GPS uart related initialization - END */

/*============================================================================*/
/* gps sensor board - UART related code - END                                  */
/*============================================================================*/

/*============================================================================*/
/* gps sensor board - gps sensor read and send to MQTT server - START         */
/*============================================================================*/

/** @fn static void GPS_SENSOR_READ_TIMEr_cb(TimerHandle_t xTimer)
 * @brief gps sensor reading timer callback - START
 * This Timer callback will be called after the gps sensor reading timer is
 * expired.The timer value is set as GPS_SENSOR_READ_TIME
 * @param xTimer TimerHandle_t
 */
static void GPS_SENSOR_READ_TIMEr_cb(TimerHandle_t xTimer)
{
	/* Set the gps sensor timer expire flag */
	gps_read_timer_expire_flag = GPS_READ_TIMER_EXPIRE_SET;
}
/* gps sensor reading timer callback - END */

/** @fn static void create_mqtt_payload(void)
 * @brief MQTT Create payload API - START
 * This API will be used to create the payload string/buffer from gps final
 * value received after processing of raw value.
 */
static void create_mqtt_payload(st_mqtt_message * pmsg , char * buff)
{
	sprintf(buff,"{\"%s\":%s,\"%s\":%s,\"%s\":%s,\"%s\":%s,\"%s\":%s}%c",MZ_MZTT_KEY1,final_lat_data,MZ_MZTT_KEY2,final_lon_data,MZ_MZTT_KEY3,final_pdop_data,MZ_MZTT_KEY4,final_hdop_data,MZ_MZTT_KEY5,final_vdop_data,26);

	pmsg->topic = MZ_MQTT_PUB_TOPIC;
	pmsg->qos = MZ_MQTT_PUB_QOS;
	pmsg->retain = MQTT_RETAIN_OFF;
	pmsg->message = buff;
}
/* MQTT Create payload API - END */

/** @fn static void send_payload_to_server(void)
 * @brief MQTT send payload API - START
 * This API will be used to send the payload string/buffer to MonoZ_Lib.
 * It will also print if the sending of payload to MonoZ_Lib was successful or
 * any error occurred
 */
static void send_payload_to_server(st_mqtt_message * pmsg)
{
	//mz_error_t status = mz_mqtt_pub(pmsg);

	mz_error_t status = MZ_init_cmd_direct("AT+QMTDISC=0\r\n",AT_TIME_15SEC, AT_TIME_15SEC);
	status |= MZ_init_cmd_direct("AT+QMTOPEN=0,\"cloud.monoz.io\",1883\r\n",AT_TIME_15SEC, AT_TIME_15SEC);
	status |= MZ_init_cmd_direct("AT+QMTCONN=0,\"GPSTest\",\"GPSTest\",\"GPSTest\"\r\n",AT_TIME_15SEC, AT_TIME_15SEC);
	status |= MZ_init_cmd_direct("AT+QMTPUB=0,0,0,0,\"v1/devices/me/telemetry\"\r\n",AT_TIME_15SEC, 0);
	status |= MZ_init_cmd_direct(payload_string,AT_TIME_15SEC, AT_TIME_15SEC);

	/* Check the status of the request */
	if(MZ_OK == status)
	{
		/* print success on CLI */
		mz_puts("Data send to MonoZ_Lib\r\n");
	}
	else
	{
		/* print of error string on CLI */
		mz_puts("Data send to MonoZ_Lib FAILED\r\n");
		//mz_puts(mz_error_to_str(status));
	}
}
/* MQTT send payload API - END */

/** @fn static void gps_app_thread(void * arg)
 * @brief gps main Application thread.  START
 * 1. It creates all the timer
 * 		gps sensor reading timer,
 * 		mqtt server monitoring timer
 * @param arg void
 */


char rx1_char[255] = {0};
char respbuf[80] = {0};
char splitStrings[10][80];
char splitStrings2[21][12];

char __lat[11] = {0};
char __long[12] = {0};
char __PDOP[5] = {0};
char __HDOP[5] = {0};
char __VDOP[5] = {0};

float x1, x2;
//int cntr = 0;
static void gps_app_thread(void * arg)
{
	(void)arg;

	/*
	 * create the gps sensor reading timer.
	 * As per requirement, We are creating a recursive timer using
	 * mz_tm_create_start_recursive() API.
	 * In-case of other type of timer, please refer MZ_timer.h
	 * The expire time for this timer set to GPS_SENSOR_READ_TIME.
	 * "GPS_SENSOR_READ_TIMEr_cb" is passed as an argument.
	 * When the timer expires, the "GPS_SENSOR_READ_TIMEr_cb" API will be
	 * processed.
	 */
	if(MZ_OK == mz_tm_create_start_recursive("Uart Read timer",
											GPS_SENSOR_READ_TIME,
											GPS_SENSOR_READ_TIMEr_cb))
	{
		mz_puts("GPS sensor reading timer started\r\n");
	}

	st_mqtt_client_config mqtt_client =
	{
			.config_id = 1,
			.client_id = "GPSTest",
			.server_url = "cloud.monoz.io",
			.server_port = 1883,
			.keep_alive_time = 3600,
			.session_cleanss = MQTT_SESSION_NEW,
			.username = "GPSTest",
			.password = "GPSTest",
			.subhex = MQTT_SUB_NORMAL,
			.connect_mode = MQTT_ASYNCMODE
	};

	//mz_mqtt_client_configure(&mqtt_client);

	/*
	 * This is the infinite loop for this thread - the thread will execute this
	 * loop forever and not come outside of this loop
	 */
	while(1)
	{
		/* Need to write the periodic executing logic in this loop block */

		 /* Process the server re-registration based on registration required or
		 * not status - Check if the flag is set
		 */

		/* Read GPS sensor uart data in a buffer*/
		(void)MZ_UART_Receive_IT(MZ_GPS_UART_INSTANCE,(uint8_t *)&rx1_char,sizeof(rx1_char));

		int j = 0, cnt = 0;
	    for(int i = 0; i <= (strlen(rx1_char)); i++)
	    {
			/*  10:19:02  $GPRMC,101902.00,A,2951.91860,N,07752.38737,E,0.032,,300322,,,A*7C
				10:19:02  $GPVTG,,T,,M,0.032,N,0.060,K,A*24
				10:19:02  $GPGGA,101902.00,2951.91860,N,07752.38737,E,1,05,3.95,248.4,M,-36.3,M,,*7A
				10:19:02  $GPGSA,A,3,06,02,19,24,17,,,,,,,,4.73,3.95,2.60*05
				10:19:02  $GPGSV,3,1,09,02,62,243,34,03,00,033,,06,65,030,32,11,64,227,32*76
				10:19:02  $GPGSV,3,2,09,17,27,062,23,19,41,045,29,20,25,174,20,24,34,262,32*73
				10:19:02  $GPGSV,3,3,09,28,42,121,19*46
				10:19:02  $GPGLL,2951.91860,N,07752.38737,E,101902.00,A,A*64
			*/

	    	/* Split all new lines from received buffer */
	        if(rx1_char[i] == '\n')
	        {
	            splitStrings[cnt][j] = '\0';
	            cnt++;
	            j=0;
	        }
	        else
	        {
	            splitStrings[cnt][j] = rx1_char[i];
	            j++;
	        }
		}
	    for(int i = 0; i <= cnt; i++)
		{
			int jj = 0, cnt2 = 0;
			strcpy(respbuf, splitStrings[i]);

			for(int ii = 0; ii <= (strlen(respbuf)); ii++)
			{
				/* split all contents from line */
				if((respbuf[ii] == ',') || (respbuf[ii] == '*'))
				{
					splitStrings2[cnt2][jj] = '\0';
					cnt2++;
					jj = 0;
				}
				else
				{
					splitStrings2[cnt2][jj] = respbuf[ii];
					jj++;
				}
			}
		    for(int ii = 0; ii <= cnt2; ii++)
			{
				char latbuf1[10] = {0};
				char lonbuf1[10] = {0};
				char latbuf2[2] = {0};
				char lonbuf2[3] = {0};

	    		/* Data moved to the corresponding variable according to the GPS format */
				if((strcmp("$GPRMC",splitStrings2[ii])) == 0)
				{
					/*  $GPRMC, 123519, A, 4807.038, N, 01131.000, E,022.4, 084.4, 230394, 003.1, W*6A
						 $				Every NMEA sentence starts with $ character.
						GPRMC			Global Positioning Recommended Minimum Coordinates
						123519			Current time in UTC – 12:35:19
						A				Status A=active or V=Void.
						4807.038,N		Latitude 48 deg 07.038′ N
						01131.000,E		Longitude 11 deg 31.000′ E
						022.4			Speed over the ground in knots
						084.4			Track angle in degrees True
						220318			Current Date – 22rd of March 2018
						003.1,W			Magnetic Variation
						*6A				The checksum data, always begins with *
					 */

		    		for(int i = 0; i <= 1; i++){
		    			latbuf2[i] = splitStrings2[3][i];}

		    		for(int i = 0; i <= 2; i++){
		    			lonbuf2[i] = splitStrings2[5][i];}

		    		for(int i = 2; i <= strlen(splitStrings2[3]); i++){
		    			latbuf1[i - 2] = splitStrings2[3][i];}

		    		for(int i = 3; i <= strlen(splitStrings2[5]); i++){
		    			lonbuf1[i - 3] = splitStrings2[5][i];}

		    		x1 = (strtod(latbuf2, NULL)) + (strtod(latbuf1, NULL) / 60);
		    		x2 = (strtod(lonbuf2, NULL)) + (strtod(lonbuf1, NULL) / 60);

		    		sprintf (__lat, "%4.8f", x1);
		    		sprintf (__long, "%4.8f", x2);
				}
		    	else if((strcmp("$GPGSA",splitStrings2[ii])) == 0)
				{
		    		/*  $GPGSA,A,3,06,02,19,24,17,,,,,,,,4.73,3.95,2.60*05

						Parameter	Value	Unit			Description
						Op. Mode	A						M=Manual, A=Automatic 2D/3D
						Nav. Mode	3						1=No, 2=2D, 3=3D
						SVID		9(=G9)					Satellite ID
						SVID		8(=G8)					Satellite ID
						SVID		7(=G7)					Satellite ID
						SVID		4(=G4)					Satellite ID
						SVID		27(=G27)				Satellite ID
						SVID		30(=G30)				Satellite ID
						SVID								Satellite ID
						SVID								Satellite ID
						SVID								Satellite ID
						SVID								Satellite ID
						SVID								Satellite ID
						SVID								Satellite ID
						SVs Used	6						Number of SVs used for Navigation
						PDOP		4.73					Positional Dilution of Precision
						HDOP		3.95					Horizontal Dilution of Precision
						VDOP		2.60					Vertical Dilution of Precision
						GNSS System ID
					*/

					strcpy(__PDOP, splitStrings2[15]);
					strcpy(__HDOP, splitStrings2[16]);
					strcpy(__VDOP, splitStrings2[17]);
				}
			}
		}
	    //Moved the data to create the payload
		strcpy(final_lat_data, __lat);
		strcpy(final_lon_data, __long);
		strcpy(final_pdop_data, __PDOP);
		strcpy(final_hdop_data, __HDOP);
		strcpy(final_vdop_data, __VDOP);

		/* Create the payload from the received data */
		create_mqtt_payload(&pmsg, payload_string);

		/* send the payload to mqtt server */
		send_payload_to_server(&pmsg);


		/* Put a delay to avoid blocking on this thread */
		HAL_Delay(10);


	}//End of while(1) - Do not place any code after this.

}
/* gps main Application thread. - END */

/*
 * gps Application initialization API. - START
 *
 * 1. It call all necessary initializations before application start
 * 2. It create the main gps application
 * 3. It set all variables to its initialization states
 */
mz_error_t gps_app_init(void)
{
	mz_error_t _ret = MZ_OK;

	/* Initialize gps uart related functions */
	_ret = gps_uart_init();
	if(MZ_OK != _ret) goto clean;

	/* Create the gps application thread */
	if(!mz_thread_create(	&gps_thread_id,
							"gps Scheduler",
							gps_app_thread,
							NULL,
							osPriorityNormal,
							gps_stack,
							GPS_APP_STACK_SIZE,
							&gps_cb_mem,
							sizeof(gps_cb_mem)))
	{
		_ret = MZ_THREAD_CREATE_FAIL;
	}

	clean :
	return _ret;
}
/* gps Application initialization API. - END */

/*============================================================================*/
/* gps ADDON board - gps sensor read and send to Lwm2m server - END           */
/*============================================================================*/

/*============================================================================*/
/* gps ADDON board - LWM2M event related code - START                         */
/*============================================================================*/
/** @fn void lwm2m_event_process(void * event)
 * @brief LWM2M user defined callback function. - START
 * This Function will be called from MZ_callback.c mz_pro_default_callback().
 * User need to check the events interested based on specific application
 * requirement. If no application requirement exist for specific event, then
 * no action need to be taken by this function
 * @note MonoZ_Lib will call the mz_pro_default_callback() API after processing all
 * LWM2M server events. So the events received at this API are just to inform
 * user application to perform any post processing tasks. MonoZ_Lib maintain
 * server connection and Obj19 related data. The detailed event list refer
 * MZ_Lwm2m_public.h
 */
//void lwm2m_event_process(void * event)
//{
//	st_lw_event * e = event;
//
//	/* Check post-processing events is send by MonoZ_Lib to user */
//	switch(e->lw_event)
//	{
//		/*
//		 * This is a Observe start event. The event has already been processed
//		 * and the token is already stored inside MonoZ_Lib. If any user action
//		 * need to be taken then it can be performed here.
//		 */
//		case LW_EV_OBSERVE:
//			/* print the observe received on CLI */
//			mz_puts("Observe received\r\n");
//			/* Check if observe is started for target Object and resource */
//			if(e->lw_Obj_id == 19 && e->lw_Obj_Ins_id == 0 && e->lw_Res_id == 0)
//			{
//				/* Set the observed received flag */
//				observe_receive_flag = OBSERVE_COMPLETE_SET;
//			}
//		break;
//		case LW_EV_CLIENT_NOTIFY_SEND:
//			/* print when notify send to server */
//			mz_puts("Notify send to server\r\n");
//		break;
//		case LW_EV_CLIENT_NOTIFY_SEND_ACK:
//			/* print when notify acknowledgment is received from server */
//			mz_puts("Notify send acknowledge\r\n");
//		break;
//		case LW_EV_NOTIFY_FAIL:
//			/* print when notify failed received from server */
//			mz_puts("Notify send failed\r\n");
//		break;
//		case LW_EV_CLIENT_OFF:
//			/* Set server re-registration flag when received client off */
//			lwm2m_server_rereg_flag = SERVER_REG_NEED_SET;
//		break;
//
//#if 0
//		/* Some other sample cases */
//		case LW_EV_WRITE_DATA:
//			/* Do something */
//			;
//		break;
//		case LW_EV_EXECUTE:
//			/* Do something */
//			;
//		break;
//		case LW_EV_READ:
//			/* Do something */
//			;
//		break;
//		case LW_EV_OBSERVE_CANCEL:
//			/* Do something */
//			;
//		break;
//#endif
//
//		default:
//		break;
//	}
//}
/*============================================================================*/
/* gps ADDON board - LWM2M event related code - END                           */
/*============================================================================*/

/*============================================================================*/
/* gps ADDON board - CRC related code - START                                 */
/*============================================================================*/

/* CRC table - START */
/*
 *  CRC calculation used in validation of gps sensor data.
 *  This information is used directly as it is from the gps sensor datasheet.
 *  Please refer the datasheet for more detail information
 */
const unsigned short crc16Table[CRC_TABLE_SIZE] = {
	0x0000, 0x8005, 0x800F, 0x000A,
	0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039,
	0x0028, 0x802D, 0x8027, 0x0022,
	0x8063, 0x0066, 0x006C, 0x8069,
	0x0078, 0x807D, 0x8077, 0x0072,
	0x0050, 0x8055, 0x805F, 0x005A,
	0x804B, 0x004E, 0x0044, 0x8041,
	0x80C3, 0x00C6, 0x00CC, 0x80C9,
	0x00D8, 0x80DD, 0x80D7, 0x00D2,
	0x00F0, 0x80F5, 0x80FF, 0x00FA,
	0x80EB, 0x00EE, 0x00E4, 0x80E1,
	0x00A0, 0x80A5, 0x80AF, 0x00AA,
	0x80BB, 0x00BE, 0x00B4, 0x80B1,
	0x8093, 0x0096, 0x009C, 0x8099,
	0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189,
	0x0198, 0x819D, 0x8197, 0x0192,
	0x01B0, 0x81B5, 0x81BF, 0x01BA,
	0x81AB, 0x01AE, 0x01A4, 0x81A1,
	0x01E0, 0x81E5, 0x81EF, 0x01EA,
	0x81FB, 0x01FE, 0x01F4, 0x81F1,
	0x81D3, 0x01D6, 0x01DC, 0x81D9,
	0x01C8, 0x81CD, 0x81C7, 0x01C2,
	0x0140, 0x8145, 0x814F, 0x014A,
	0x815B, 0x015E, 0x0154, 0x8151,
	0x8173, 0x0176, 0x017C, 0x8179,
	0x0168, 0x816D, 0x8167, 0x0162,
	0x8123, 0x0126, 0x012C, 0x8129,
	0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A,
	0x810B, 0x010E, 0x0104, 0x8101,
	0x8303, 0x0306, 0x030C, 0x8309,
	0x0318, 0x831D, 0x8317, 0x0312,
	0x0330, 0x8335, 0x833F, 0x033A,
	0x832B, 0x032E, 0x0324, 0x8321,
	0x0360, 0x8365, 0x836F, 0x036A,
	0x837B, 0x037E, 0x0374, 0x8371,
	0x8353, 0x0356, 0x035C, 0x8359,
	0x0348, 0x834D, 0x8347, 0x0342,
	0x03C0, 0x83C5, 0x83CF, 0x03CA,
	0x83DB, 0x03DE, 0x03D4, 0x83D1,
	0x83F3, 0x03F6, 0x03FC, 0x83F9,
	0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9,
	0x03B8, 0x83BD, 0x83B7, 0x03B2,
	0x0390, 0x8395, 0x839F, 0x039A,
	0x838B, 0x038E, 0x0384, 0x8381,
	0x0280, 0x8285, 0x828F, 0x028A,
	0x829B, 0x029E, 0x0294, 0x8291,
	0x82B3, 0x02B6, 0x02BC, 0x82B9,
	0x02A8, 0x82AD, 0x82A7, 0x02A2,
	0x82E3, 0x02E6, 0x02EC, 0x82E9,
	0x02F8, 0x82FD, 0x82F7, 0x02F2,
	0x02D0, 0x82D5, 0x82DF, 0x02DA,
	0x82CB, 0x02CE, 0x02C4, 0x82C1,
	0x8243, 0x0246, 0x024C, 0x8249,
	0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A,
	0x826B, 0x026E, 0x0264, 0x8261,
	0x0220, 0x8225, 0x822F, 0x022A,
	0x823B, 0x023E, 0x0234, 0x8231,
	0x8213, 0x0216, 0x021C, 0x8219,
	0x0208, 0x820D, 0x8207, 0x0202
};
/* CRC table - END */

/* CRC method - START */
/** @fn static unsigned short crc16MakeTableMethod( unsigned short crc,const unsigned short *table,unsigned char *pbuffer,unsigned int length)
 * @brief calculating crc value 
 * @param crc unsigned short
 * @param table const unsigned short *,
 * @param pbuffer unsigned char *
 * @param length unsigned int
 * @return calculated crc value 
 */
static unsigned short crc16MakeTableMethod( unsigned short crc,
											const unsigned short *table,
											unsigned char *pbuffer,
											unsigned int length)
{
	while(length--)
	{
		crc = table[((crc >> 8) ^ *pbuffer++)] ^ (crc << 8); // normal
	}
	return(crc ^ CRC16_FINAL_XOR);
}

/* CRC method - END */

/*============================================================================*/
/* gps sensor board - CRC related code - END                                   */
/*============================================================================*/
