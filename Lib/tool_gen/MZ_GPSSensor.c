/*
 * MZ_GPS_Sensor.c
 *
 *  Created on: 11-July-2023
 *      Author: Piyush
 */


/* Include Header Files - START */

#include "MZ_GPSSensor.h"
#include "MZ_sys_cmsis_os2.h"
#include "MZ_error_handler.h"
#include "MZ_timer.h"
#include "MZ_print.h"
#include "MZ_Mqtt_public.h"
#include "MZ_type_converter.h"
#include "MZ_Modem_public.h"
#include "MZ_uart.h"
#include "MZ_main.h"
#include "main.h"

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
#define GPS_SENSOR_DATA_SEND_TIME				(pdMS_TO_TICKS(120000))	//< Timer is set for 120 seconds
#define GPS_SENSOR_READ_TIME					(TIME_90SEC)			///< Set 90 seconds timer for read sensor data */
/* Define some common use MACRO - END */

/* Thread related MACRO and variables - START */

#define GPS_APP_STACK_SIZE			(1024)										/* Stack size for the thread */
static mz_thread_t 					gps_thread_id = NULL;						/* Thread id handler */
static StaticTask_t 				gps_cb_mem;									/* Thread control block */
static StackType_t 					gps_stack[GPS_APP_STACK_SIZE];				/* Thread stack */

/* Thread related MACRO and variables - END */

/* Timer related MACRO and variables - START */
static char gps_read_timer_expire_flag = GPS_READ_TIMER_EXPIRE_CLEAR;			/* Flag is created and cleared for sensor data read */
static size_t gps_sensor_data_timer_id = TIMER_ID_CLEAR;					    /*!< gps_sensor timer id - Initialize it to 0 */
/* Timer related MACRO and variables - END */

/* GPS_SENSORS MACRO - START */

#define LAT_DATA_SIZE				20
#define LON_DATA_SIZE				20
#define PDOP_DATA_SIZE				8
#define HDOP_DATA_SIZE				8
#define VDOP_DATA_SIZE				8

#define PAYLOAD_STRING_SIZE			200

#define RX_BUF_SIZE					255
#define RESP_BUF_SIZE				80

#define LAT_BUFF_SIZE				11
#define LON_BUFF_SIZE				12
#define LAT_BUFF1_SIZE				10
#define LON_BUFF1_SIZE				10
#define LAT_BUFF2_SIZE				2
#define LON_BUFF2_SIZE				3
#define PDOP_BUFF_SIZE				5
#define HDOP_BUFF_SIZE				5
#define VDOP_BUFF_SIZE				5

#define SPLIT_STRING_SIZE1			10
#define SPLIT_STRING_SIZE1_2		80

#define SPLIT_STRING_SIZE2			21
#define SPLIT_STRING_SIZE2_2		12

#define MINUTE_DEVIDER				60

/* GPS_SENSORS MACRO - END */

/* GPS sensor related MACRO and variables - START */

static volatile long int final_gps_value = INIT_0;						/* Store final gps value after calculation from raw value and decimal place */

static char  final_lat_data[LAT_DATA_SIZE] = {0};				/* Store final gps value after calculation from raw value and decimal place */
static char  final_lon_data[LON_DATA_SIZE] = {0};				/* Store final gps value after calculation from raw value and decimal place */
static char  final_pdop_data[PDOP_DATA_SIZE] = {0};			/* Store final gps value after calculation from raw value and decimal place */
static char  final_hdop_data[HDOP_DATA_SIZE] = {0};			/* Store final gps value after calculation from raw value and decimal place */
static char  final_vdop_data[VDOP_DATA_SIZE] = {0};			/* Store final gps value after calculation from raw value and decimal place */

/* GPS sensor related MACRO and variables - END */

/* GPS UART related variables - START */
static volatile char gps_uart_recv_complete_flag = UART_RECEIVE_COMPLETE_CLEAR; /*!< Flag to hold if the uart receive is complete or not */
/* GPS UART related variables - END */

/* MQTT related MACRO and variables - START */
static char payload_string[PAYLOAD_STRING_SIZE] = "";					/* payload string buffer to pass the final payload information to MonoZ_Lib */
static st_mqtt_message pmsg;
/* MQTT related MACRO and variables - END */

/* static function prototypes - START */

static mz_error_t gps_uart_init(void);
static void gps_sensor_read_timer_cb(TimerHandle_t xTimer);
static void create_mqtt_payload(st_mqtt_message * pmsg , char * buff);
static void send_payload_to_server(st_mqtt_message * pmsg);
static void gps_app_thread(void * arg);

/* static function prototypes - END */

/* GPS sensor variable and buffers START*/
int8_t dataTxReady = INIT_0;  	//data transmit ready flag
int8_t timerCBFlag = INIT_0;  	//timer callback flag

char __lat[LAT_BUFF_SIZE] = {0};
char __long[LON_BUFF_SIZE] = {0};
char __PDOP[PDOP_BUFF_SIZE] = {0};
char __HDOP[HDOP_BUFF_SIZE] = {0};
char __VDOP[VDOP_BUFF_SIZE] = {0};

float __fLat = FLOAT_0;
float __fLon = FLOAT_0;

char rx1_char[RX_BUF_SIZE] = {0};
char respbuf[RESP_BUF_SIZE] = {0};

char splitStrings[SPLIT_STRING_SIZE1][SPLIT_STRING_SIZE1_2];
char splitStrings2[SPLIT_STRING_SIZE2][SPLIT_STRING_SIZE2_2];
/* GPS sensor variable and buffers END*/

/* GPS UART configuration related MACRO - START */
#define MZ_GPS_INSTANCE 					(LPUART1)					///< Defines the instance
#define MZ_GPS_INIT_BAUDRATE				(9600)						///< Defines the baud rate
#define MZ_GPS_INIT_WORDLENGTH				(UART_WORDLENGTH_8B)		///< Defines the initial word length
#define MZ_GPS_INIT_STOPBITS				(UART_STOPBITS_1)			///< Defines the stop bits
#define MZ_GPS_INIT_PARITY					(UART_PARITY_NONE)			///< Defines the initial parity
#define MZ_GPS_INIT_MODE 					(UART_MODE_TX_RX)			///< Defines the mode
#define MZ_GPS_INIT_HWFLOWCTL 				(UART_HWCONTROL_NONE)		///< Defines initial hardware flow control
#define MZ_GPS_INIT_OVERSAMPLING 			(UART_OVERSAMPLING_16)		///< Defines the initial oversampling
#define MZ_GPS_INIT_ONEBITSAMPLING 			(UART_ONE_BIT_SAMPLE_DISABLE)	///< Defines the initial one bit sampling
#define MZ_GPS_ADVANCEDINIT_ADVFEATUREINIT 	(UART_ADVFEATURE_NO_INIT)	///< Defines the Uart advance features
#define MZ_GPS_UART_INSTANCE				(_LPUART1)					///< Defimes the UART instance
/* GPS UART configuration related MACRO - END */

#define MZ_MQTT_PUB_TOPIC 		"\"v1/devices/me/telemetry\""
#define MZ_MQTT_PUB_QOS			MQTT_QOS0
#define MZ_MQTT_SUB_TOPIC		"\"v1/devices/me/attributes\""
#define MZ_MQTT_SUB_QOS			MQTT_QOS2
#define MZ_MZTT_KEY1			"latitude"
#define MZ_MZTT_KEY2			"longitude"
#define MZ_MZTT_KEY3			"PDOP"
#define MZ_MZTT_KEY4			"HDOP"
#define MZ_MZTT_KEY5			"VDOP"

/* GPS UART configuration structure - START */
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
/*GPS UART related callback - END */

/** @fn static void gps_sensor_data_timer_cb(TimerHandle_t xTimer)
 * @brief gps_sensor_data_timer_cb timer callback - START
 * This Timer callback will be called after the loadcell_data timer is
 * expired.The timer value is set as LOADCELL_DATA_SEND_TIME
 * @param xTimer TimerHandle_t
 * @retval None
 */
static void gps_sensor_data_timer_cb(TimerHandle_t xTimer)
{
	/* Stopping the loadcell data one time timer */
	if(MZ_OK != mz_tm_stop(gps_sensor_data_timer_id))
	{
		/* print of error starting on CLI */
		mz_puts("gps sensor data timer stopping failed\r\n");
	}
    else {} // Default waiting case.

	HAL_Delay(DELAY_100MS);
	dataTxReady = FLAG_SET;

	/* Print when the application is ready for data transmission */
	mz_puts("Ready for data Transmission\r\n");
}
/* Sensor transmission timer callback - END */

/** @fn static mz_error_t gps_uart_init(void)
 *  @brief GPS UART related initialization - START
 * This Timer callback will be called after the server monitoring timer is
 * expired.The timer value is set as MQTT_SERVER_MONITORING_TIME
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

/** @fn static void gps_sensor_read_timer_cb(TimerHandle_t xTimer)
 * @brief gps sensor reading timer callback - START
 * This Timer callback will be called after the gps sensor reading timer is
 * expired.The timer value is set as GPS_SENSOR_READ_TIME
 * @param xTimer TimerHandle_t
 */
static void gps_sensor_read_timer_cb(TimerHandle_t xTimer)
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
 * @brief GPS main Application thread.  START
 * 1. It creates all the timer
 * 		gps sensor reading timer,
 * 		mqtt server monitoring timer
 * 2. It process the responses to find gps raw value
 * @param arg void
 */

static void gps_app_thread(void * arg)
{
	(void)arg;

	char latbuf1[LAT_BUFF1_SIZE] = {0};
	char lonbuf1[LON_BUFF1_SIZE] = {0};
	char latbuf2[LAT_BUFF2_SIZE] = {0};
	char lonbuf2[LON_BUFF2_SIZE] = {0};

	int32_t wrdInLine = INIT_0;
	int32_t noLineCnt = INIT_0;
	int32_t wrdInLine2 = INIT_0;
	int32_t noLineCnt2 = INIT_0;

	/*
	 * create the gps sensor reading timer.
	 * As per requirement, We are creating a recursive timer using
	 * mz_tm_create_start_recursive() API.
	 * In-case of other type of timer, please refer MZ_timer.h
	 * The expire time for this timer set to GPS_SENSOR_READ_TIME.
	 * "gps_sensor_read_timer_cb" is passed as an argument.
	 * When the timer expires, the "gps_sensor_read_timer_cb" API will be
	 * processed.
	 */
	if(MZ_OK == mz_tm_create_start_recursive("Uart Read timer",
											GPS_SENSOR_READ_TIME,
											gps_sensor_read_timer_cb))
	{
		mz_puts("GPS sensor reading timer started\r\n");
	}

	/*
	 * This is the infinite loop for this thread - the thread will execute this
	 * loop forever and not come outside of this loop
	 */
	while(1)
	{
		/* Need to write the periodic executing logic in this loop block
		 * Process the server re-registration based on registration required or
		 * not status - Check if the flag is set
		 */

		/* Read GPS uart data in a buffer*/
		(void)MZ_UART_Receive_IT(MZ_GPS_UART_INSTANCE, (uint8_t *)&rx1_char, sizeof(rx1_char));

		wrdInLine = 0;
		noLineCnt = 0;
	    for(int16_t recCmptData = 0; recCmptData <= (strlen(rx1_char)); recCmptData++)
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
	        if(rx1_char[recCmptData] == '\n')
	        {
	            splitStrings[noLineCnt][wrdInLine] = '\0';
	            noLineCnt++;
	            wrdInLine = 0;
	        }
	        else
	        {
	            splitStrings[noLineCnt][wrdInLine] = rx1_char[recCmptData];
	            wrdInLine++;
	        }
		}
	    for(int16_t dataInLine = 0; dataInLine <= noLineCnt; dataInLine++)
		{
			wrdInLine2 = 0;
			noLineCnt2 = 0;
			strcpy(respbuf, splitStrings[dataInLine]);

			for(int16_t dataInLine2 = 0; dataInLine2 <= (strlen(respbuf)); dataInLine2++)
			{
				/* split all contents from line */
				if((respbuf[dataInLine2] == ',') || (respbuf[dataInLine2] == '*'))
				{
					splitStrings2[noLineCnt2][wrdInLine2] = '\0';
					noLineCnt2++;
					wrdInLine2 = 0;
				}
				else
				{
					splitStrings2[noLineCnt2][wrdInLine2] = respbuf[dataInLine2];
					wrdInLine2++;
				}
			}
		    for(int32_t ii = 0; ii <= noLineCnt2; ii++)
			{
				memset(latbuf1, 0, LAT_BUFF1_SIZE);
				memset(latbuf2, 0, LAT_BUFF2_SIZE);
				memset(lonbuf1, 0, LON_BUFF1_SIZE);
				memset(lonbuf2, 0, LON_BUFF2_SIZE);
				HAL_Delay(100);

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

		    		for(int16_t i = 0; i <= 1; i++){
		    			latbuf2[i] = splitStrings2[3][i];}

		    		for(int16_t i = 0; i <= 2; i++){
		    			lonbuf2[i] = splitStrings2[5][i];}

		    		for(int16_t i = 2; i <= strlen(splitStrings2[3]); i++){
		    			latbuf1[i - 2] = splitStrings2[3][i];}

		    		for(int16_t i = 3; i <= strlen(splitStrings2[5]); i++){
		    			lonbuf1[i - 3] = splitStrings2[5][i];}

		    		__fLat = (strtod(latbuf2, NULL)) + (strtod(latbuf1, NULL) / MINUTE_DEVIDER);
		    		__fLon = (strtod(lonbuf2, NULL)) + (strtod(lonbuf1, NULL) / MINUTE_DEVIDER);

		    		sprintf (__lat, "%4.8f", __fLat);
		    		sprintf (__long, "%4.8f", __fLon);
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
			HAL_Delay(10);
		}
	    //Move the data to create the payload
		strcpy(final_lat_data, __lat);
		strcpy(final_lon_data, __long);
		strcpy(final_pdop_data, __PDOP);
		strcpy(final_hdop_data, __HDOP);
		strcpy(final_vdop_data, __VDOP);

		if(timerCBFlag == FLAG_CLEAR)
		{
			timerCBFlag = FLAG_SET;
			/* Stopping the gps sensor data one time timer */
			if(gps_sensor_data_timer_id)
			{
				/* Starting the gps sensor data one time timer */
				if(MZ_OK != mz_tm_start(gps_sensor_data_timer_id))
				{
					/* print of error string on CLI */
					mz_puts("gps sensor data timer creation failed\r\n");
				}
				else {} // Default waiting case.
			}
			else
			{
				/* Create the gps sensor data timer.*/
				gps_sensor_data_timer_id = mz_tm_create_one("gps sensor data timer",
															GPS_SENSOR_DATA_SEND_TIME,
															gps_sensor_data_timer_cb);

				/* Starting the gps sensor data one time timer */
				if(MZ_OK != mz_tm_start(gps_sensor_data_timer_id))
				{
					/* print of error string on CLI */
					mz_puts("gps sensor data timer creation failed\r\n");
				}
				else {} // Default waiting case.
			}
		}
		else {} // Default waiting case.

		/* Send data to MQTT server */
		if(dataTxReady == FLAG_SET)
		{
			/* Create the payload from the received data */
			create_mqtt_payload(&pmsg, payload_string);

			/* send the payload to mqtt server */
			send_payload_to_server(&pmsg);

			dataTxReady = FLAG_CLEAR;
			timerCBFlag = FLAG_CLEAR;
		}
		else {} // Default waiting case.

		/* Put a delay to avoid blocking on this thread */
		HAL_Delay(10);

	}//End of while(1) - Do not place any code after this.

}
/* GPS main Application thread. - END */

/*
 * GPS Application initialization API. - START
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
/* GPS Application initialization API. - END */



