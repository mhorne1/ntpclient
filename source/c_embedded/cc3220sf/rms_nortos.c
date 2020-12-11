/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== rms_nortos.c ========
 *  Built with:
 *  Code Composer Studio 10.1.1.00004
 *  SimpleLink CC32xx SDK 4.30.0.06
 */

/* Standard includes */
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

/* TI-DRIVERS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/net/wifi/simplelink.h>


#include "common.h"
#include "platform.h"
#include "uart_term.h"

/* For usleep() */
#include <unistd.h>

/* Accelerometer Drivers */
#include "bma2xxdrv.h"

/* Project header files */
#include "wifi_config.h"
#include "rms_nortos.h"

#define MAIN_LOOP_DELAY     (3)

//*****************************************************************************
//                          GLOBALS
//*****************************************************************************
PowerMeasure_ControlBlock   PowerMeasure_CB;

PowerMeasure_AppData        PowerMeasure_appData = {
        PM_USECASE,         /* The exercised use case  */
        AC_USECASE,         /* The Always connected use case  */
        NUM_OF_PKT,         /* how many Intervals do on this use case */
        OPEN_SOCK_ONCE,     /* socket ID*/
        SOCKET_TYPE,        /* Socket type */
        DEST_IP_ADDR,       /* IP address */
        PORT,               /* socket port number */
};

ClockP_Handle PowerMeasure_lpdsHandle;

/* Temperature values */
volatile float g_temperatureC;
volatile float g_temperatureF;

/* Accelerometer values */
volatile int8_t g_xVal;
volatile int8_t g_yVal;
volatile int8_t g_zVal;

/* Pointer to transmission buffer used by TCP Client */
signed char *g_pClientSendBuffer = PowerMeasure_CB.frameData;

/* Flag for copying string to transmission buffer used by TCP Client */
bool g_clientSendFlag = false;

/* Stores Transmit time-stamp seconds */
extern volatile time_t g_txTm;

/* Stores offset of Transmit time-stamp seconds */
//volatile time_t g_offsetTm;

/* Stores Local Transmit time-stamp seconds */
volatile time_t g_localTm;

//****************************************************************************
//                          LOCAL FUNCTION PROTOTYPES
//****************************************************************************
int initLocalTime(SlDateTime_t *pdateTime);
int setLocalTime(SlDateTime_t *pdateTime, struct tm *pbuf);
int getLocalTime(SlDateTime_t *pdateTime, struct tm *pbuf, _u8 *pconfigOpt, _u16 *pconfigLen);
uint8_t accelerometerReading(I2C_Handle i2cHandle);

//****************************************************************************
//                          EXTERNAL FUNCTIONS
//****************************************************************************
extern time_t getNTPTime( void );

//*****************************************************************************
//
//! \brief Created by main function. This starts SimpleLink, connects to an
//!        AP. Acquires time from an NTP time server. Measures temperature
//!        sensor, measures accelerometer. Acquires time from an NTP time
//!        server. Connects to an external TCP Server, and sends measurements
//!        with time stamps.
//!
//! \param pvParameters is a general void pointer (not used here).
//!
//! \return none
//
//*****************************************************************************
void mainThread(void *pvParameters)
{
    int             state_var = 0;  // Used to control main program loop
    char            red_led_state = false; // Used to control Red LED indication
    uint8_t         txBuffer[1];    // Buffer for writing to I2C bus
    uint8_t         rxBuffer[2];    // Buffer for receiving on I2C bus
    I2C_Handle      i2c;            // Handle for I2C bus
    I2C_Params      i2cParams;      // I2C bus configuration
    I2C_Transaction i2cTransaction; // I2C driver struct
    char *timeString;               // Pointer to string converted from time value
    SlDateTime_t dateTime = {0};    // Stores time and calendar information
    struct tm timeBuf = {0};        // Stores time and calendar information too
    _u16 configLen = sizeof(SlDateTime_t); // Used for internal time on NWP
    _u8 configOpt = SL_DEVICE_GENERAL_DATE_TIME; // Used for internal time on NWP

    /* Initialize local internal time on NWP */
    initLocalTime(&dateTime);

    GPIO_init();
    SPI_init();
    I2C_init();

    /* Configure the UART */
    InitTerm();

    UART_PRINT("\r\nProject - No RTOS / Bare Metal\r\n");

    /* Turn off Red LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    UART_PRINT("Initializing I2C Bus\r\n");

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(CONFIG_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        while (1);
    }

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 2;

    UART_PRINT("Identifying temperature sensor\r\n");
    /*
     * Determine which I2C sensor is present.
     * We will prefer sensors in this order: TMP117 (on BoosterPacks),
     * TMP116 (on-board CC32XX LaunchPads), and last TMP006
     * (on older CC32XX LaunchPads).
     */
    /* Try BP TMP values */
    txBuffer[0] = TMP_BP_REG;
    i2cTransaction.slaveAddress = TMP_BP_ADDR;
    if (!I2C_transfer(i2c, &i2cTransaction)) {
        /* Not BP TMP, try LP TMP116 */
        i2cTransaction.slaveAddress = TMP116_LP_ADDR;
        if (!I2C_transfer(i2c, &i2cTransaction)) {
            /* Not a TMP116 try TMP006*/
            txBuffer[0] = TMP006_REG;
            i2cTransaction.slaveAddress = TMP006_ADDR;
            if (!I2C_transfer(i2c, &i2cTransaction)) {
                /* Could not resolve a sensor, error */
                while(1);
            }
        }
    }

    /* Clear SimpleLink status */
    PowerMeasure_CB.slStatus = 0;

    /* Connect to WIFI */
    UART_PRINT("Connecting to WIFI\r\n");

    /* Connect To AP for first time, connected use cases only */
    if (PowerMeasure_appData.useCase == UseCase_AlwaysConnected ||
        PowerMeasure_appData.useCase == UseCase_IntermittentlyConnected)
    {
        sl_Start(0,0,0);
        wlanConnect();
        /* Stay connected if the use case is always connected */
        if (PowerMeasure_appData.useCase == UseCase_IntermittentlyConnected) {
            sl_Stop(SL_STOP_TIMEOUT);
        }
    }

    /* Acquire time on start up */
    UART_PRINT("Acquiring time from NTP source\r\n");

    g_txTm = getNTPTime(); // Get time from NTP source

    timeString = ctime( ( const time_t* ) &g_txTm ); // Get string with date and time
    timeString[strlen(timeString)-1] = 0;            // Remove newline char
    //UART_PRINT("NTP Time = %s\n\r", timeString);

    /* Set local internal time on NWP */
    setLocalTime(&dateTime, &timeBuf);

    UART_PRINT("Starting main program loop\r\n");

    while (1)
    {
        state_var++; // Increment state_var
        switch (state_var) {
        case STATE_simlink:
            /* The SimpleLink host driver architecture mandate calling
               'sl_task' in a NO-RTOS application's main loop.       */
            /* The purpose of this call, is to handle asynchronous
               events and get flow control information sent from the NWP.*/
            /* Every event is classified and later handled
               by the host driver event handlers.                  */
            sl_Task(NULL);
            break;
        case STATE_delay:   // Main program loop delay
            UART_PRINT("Sleeping %d seconds...\n\r",
                       (unsigned int)MAIN_LOOP_DELAY);
            sleep((unsigned int)MAIN_LOOP_DELAY);
            break;
        case STATE_redled:  // Blink Red LED
            red_led_state = ~red_led_state;
            if (red_led_state) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            } else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            GPIO_write(CONFIG_GPIO_LED_0, red_led_state);
            break;
        case STATE_temp:    //
            if (I2C_transfer(i2c, &i2cTransaction)) {
                /*
                 *  Extract degrees C from the received data; see sensor datasheet.
                 *  Make sure we are updating the global temperature variables
                 *  in a thread-safe manner.
                 */
                g_temperatureC = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);
                g_temperatureC *= 0.03125;
                g_temperatureF = g_temperatureC * 9 / 5 + 32;

                UART_PRINT("Current temp = %3.2fC (%3.2fF)\n\r", g_temperatureC, g_temperatureC);
            }
            break;
        case STATE_accel:   //
            /* Read from 3-axis accelerometer */
            accelerometerReading(i2c);

            UART_PRINT("AccX = %d\n\r", g_xVal);
            UART_PRINT("AccY = %d\n\r", g_yVal);
            UART_PRINT("AccZ = %d\n\r", g_zVal);
            break;
        case STATE_time: // Get current time
            /* Re-acquire time */
            //g_txTm = getNTPTime(); // Update time

            /* Get local internal time on NWP */
            getLocalTime(&dateTime, &timeBuf, &configOpt, &configLen);

            timeString = ctime( ( const time_t* ) &g_localTm ); // Get string with date and time
            timeString[strlen(timeString)-1] = 0;               // Remove newline char

            //UART_PRINT("Converted Time = %s\n\r", timeString);
            break;
        case STATE_message: // Update TCP Client Message
            g_clientSendFlag = true; // Allow copying of string to send buffer
            /* TCP Client ASCII Message Format */
            UART_PRINT("%s,%3.2f,%3.2f,%d,%d,%d\n\r",
                       timeString,
                       g_temperatureC,
                       g_temperatureF,
                       g_xVal,
                       g_yVal,
                       g_zVal);
            g_clientSendFlag = false; // Deny copying of string to send buffer
            break;
        case STATE_tcpclient: // Run TCP Client
            UART_PRINT("Send Packet...\n\r");
            prepareDataFrame(PowerMeasure_appData.port,
                             PowerMeasure_appData.ipAddr);
            alwaysConnected();
            break;
        case STATE_last: // Reset state_var
            state_var = 0;
            break;
        default:
            UART_PRINT("Unknown state_var\r\n");
            state_var = 0;
            break;
        }
    }
}

/*
 * SWRU455L
 * SimpleLink™ Wi-Fi® CC3x20, CC3x3x Network Processor
 * 3.7 Time and Date
 * The SimpleLink Wi-Fi device gives users the option to set, and get time and
 * date configuration from the RTC on the device. The RTC is a continuous
 * counter which is active even during hibernation and resets only after
 * shutdown.
 */
int initLocalTime(SlDateTime_t *pdateTime) {
    _i16 Status;

    pdateTime->tm_day = (_u32)23; /* Day of month (DD format) range 1-31 */
    pdateTime->tm_mon = (_u32)6; /* Month (MM format) in the range of 1-12 */
    pdateTime->tm_year = (_u32)2014; /* Year (YYYY format) */
    pdateTime->tm_hour = (_u32)17; /* Hours in the range of 0-23 */
    pdateTime->tm_min = (_u32)55; /* Minutes in the range of 0-59 */
    pdateTime->tm_sec = (_u32)22; /* Seconds in the range of 0-59 */

    Status = sl_DeviceSet(SL_DEVICE_GENERAL,
                          SL_DEVICE_GENERAL_DATE_TIME,
                          sizeof(SlDateTime_t),
                          (_u8*)(pdateTime));
    return Status;
}

/*
 * Convert time_t from NTP source (which is a total number of seconds) to a tm
 * struct with date and time fields. Transfer the date and time fields to the
 * equivalent SimpleLink struct. Use the SimpleLink struct to set the internal
 * time of the NWP.
 */
int setLocalTime(SlDateTime_t *pdateTime, struct tm *pbuf) {
    _i16 Status;
    volatile time_t g_offsetTm;

    g_offsetTm = g_txTm - (6*60*60);
    pbuf = gmtime( ( const time_t* ) &g_offsetTm);

    pdateTime->tm_day = pbuf->tm_mday; // Day of month (DD format) range 1-31
    pdateTime->tm_mon = pbuf->tm_mon;
    pdateTime->tm_year = pbuf->tm_year;
    pdateTime->tm_hour = pbuf->tm_hour;
    pdateTime->tm_min = pbuf->tm_min;
    pdateTime->tm_sec = pbuf->tm_sec;
    /*
    UART_PRINT("Day %d,Mon %d,Year %d,Hour %d,Min %d,Sec %d\n\r",
               pdateTime->tm_day,
               pdateTime->tm_mon,
               pdateTime->tm_year,
               pdateTime->tm_hour,
               pdateTime->tm_min,
               pdateTime->tm_sec);
    */
    Status = sl_DeviceSet(SL_DEVICE_GENERAL,
                          SL_DEVICE_GENERAL_DATE_TIME,
                          sizeof(SlDateTime_t),
                          (_u8*)(pdateTime));
    return Status;
}

/*
 * Get the internal time from the NWP as a SimpleLink struct. Convert the
 * SimpleLink struct to an equivalent a tm struct. Convert the tm struct to
 * a time_t (which is a total number of seconds).
 */
int getLocalTime(SlDateTime_t *pdateTime, struct tm *pbuf, _u8 *pconfigOpt, _u16 *pconfigLen) {
    _i16 Status;

    Status = sl_DeviceGet(SL_DEVICE_GENERAL,
                          pconfigOpt,
                          pconfigLen,
                          (_u8 *)(pdateTime));
    /*
    UART_PRINT("Day %d,Mon %d,Year %d,Hour %d,Min %d,Sec %d\n\r",
               pdateTime->tm_day,
               pdateTime->tm_mon,
               pdateTime->tm_year,
               pdateTime->tm_hour,
               pdateTime->tm_min,
               pdateTime->tm_sec);
    */
    pbuf->tm_hour = pdateTime->tm_hour;
    pbuf->tm_min = pdateTime->tm_min;
    pbuf->tm_sec =  pdateTime->tm_sec;
    pbuf->tm_mday = pdateTime->tm_day;
    pbuf->tm_mon = pdateTime->tm_mon;
    pbuf->tm_year = pdateTime->tm_year;

    g_localTm = mktime(pbuf);
    return Status;
}

//*****************************************************************************
//
//! Function to read accelerometer
//!
//! \param  I2C_Handle
//!
//! \return SUCCESS or FAILURE
//!
//*****************************************************************************
uint8_t accelerometerReading(I2C_Handle i2cHandle)
{
    int8_t xValRead, yValRead, zValRead;
    int32_t status;

    /* Read accelerometer axis values */
    status = BMA2xxReadNew(i2cHandle, &xValRead, &yValRead, &zValRead);
    if(status != 0) {
        /* try to read again */
        status = BMA2xxReadNew(i2cHandle, &xValRead, &yValRead, &zValRead);
        if (status != 0) { /* leave previous values */
            UART_PRINT(
                "[Link local task] Failed to read data from accelerometer\n\r");
        }
    }

    if (status == 0) {
        g_xVal = xValRead;
        g_yVal = yValRead;
        g_zVal = zValRead;
    }

    return(status);
}

//*****************************************************************************
// Callback Functions
//*****************************************************************************

//*****************************************************************************
//
//! \brief Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1
//!
//! \param  index
//!
//! \return none
//!
//*****************************************************************************
void gpioButtonFxn1(uint32_t index)
{
}

//*****************************************************************************
// SimpleLink Callback Functions
//*****************************************************************************

void SimpleLinkNetAppRequestMemFreeEventHandler (uint8_t *buffer)
{
  // do nothing...
}

void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest,
                                         SlNetAppResponse_t *pNetAppResponse)
{
  // do nothing...
}

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    switch(pWlanEvent->Id)
    {
        case SL_WLAN_EVENT_CONNECT:
        {

            SET_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_CONNECTION);
        }
        break;

        case SL_WLAN_EVENT_DISCONNECT:
        {
            CLR_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_IP_ACQUIRED);
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Id);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    switch (slFatalErrorEvent->Id)
    {
        case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
        {
        UART_PRINT(
            "[ERROR] - FATAL ERROR: Abort NWP event "
            "detected: AbortType=%d, AbortData=0x%x\n\r",
            slFatalErrorEvent->Data.DeviceAssert.Code,
            slFatalErrorEvent->Data.DeviceAssert.Value);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
        {
        UART_PRINT(
            "[ERROR] - FATAL ERROR: "
            "No Cmd Ack detected [cmd opcode = 0x%x] \n\r",
            slFatalErrorEvent->Data.NoCmdAck.Code);
        }
        break;

        case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
        {
            UART_PRINT("[ERROR] - FATAL ERROR: Sync loss detected n\r");
        }
        break;

        case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
        {
        UART_PRINT(
            "[ERROR] - FATAL ERROR: Async event timeout "
            "detected [event opcode =0x%x]  \n\r",
            slFatalErrorEvent->Data.CmdTimeout.Code);
        }
        break;

        default:
            UART_PRINT("[ERROR] - FATAL ERROR:"
                       " Unspecified error detected \n\r");
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info 
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Id)
    {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        {
            SET_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_IP_ACQUIRED);
            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
            "Gateway=%d.%d.%d.%d\n\r", 
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,3),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,2),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,1),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,0),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Id);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerEventHandler(
    SlNetAppHttpServerEvent_t *pHttpEvent,
    SlNetAppHttpServerResponse_t *
    pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info 
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->Data.Error.Code,
               pDevEvent->Data.Error.Source);
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->SocketAsyncEvent.SockTxFailData.Status)
            {
                case SL_ERROR_BSD_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\r",
                                    pSock->SocketAsyncEvent.SockTxFailData.Sd);
                    break;
                default: 
            UART_PRINT(
                "[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                pSock->SocketAsyncEvent.SockTxFailData.Sd,
                pSock->SocketAsyncEvent.SockTxFailData.Status);
                    break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] "
                       "- Unexpected Event [%x0x]\n\n",pSock->Event);
            break;
    }

}

void SimpleLinkSocketTriggerEventHandler(SlSockTriggerEvent_t *pSlTriggerEvent)
{
    // Unused in this application
}

//*****************************************************************************
// Local Functions
//*****************************************************************************

//*****************************************************************************
//
//! \brief Display Application Banner
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
int32_t displayBanner(void)
{
    int32_t     status = -1;
    uint8_t     macAddress[SL_MAC_ADDR_LEN];
    uint16_t    macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t    configSize = 0;
    uint8_t     configOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    configSize = sizeof(SlDeviceVersion_t);
    status = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(status);

    /* Print device version info. */
    status =
        sl_DeviceGet(SL_DEVICE_GENERAL, &configOpt, &configSize,
                     (uint8_t*)(&ver));
    ASSERT_ON_ERROR(status);

    /* Print device Mac address */
    status = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                          &macAddress[0]);
    ASSERT_ON_ERROR(status);

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\t    %s Example Ver: %s\n\r",APPLICATION_NAME,
               APPLICATION_VERSION);
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t CHIP: 0x%x",ver.ChipId);
    UART_PRINT("\n\r");
    UART_PRINT("\t MAC:  %d.%d.%d.%d",ver.FwVersion[0],ver.FwVersion[1],
               ver.FwVersion[2],
               ver.FwVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t PHY:  %d.%d.%d.%d",ver.PhyVersion[0],ver.PhyVersion[1],
               ver.PhyVersion[2],
               ver.PhyVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t NWP:  %d.%d.%d.%d",ver.NwpVersion[0],ver.NwpVersion[1],
               ver.NwpVersion[2],
               ver.NwpVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t ROM:  %d",ver.RomVersion);
    UART_PRINT("\n\r");
    UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
    UART_PRINT("\n\r");
    UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x", macAddress[0],
               macAddress[1], macAddress[2], macAddress[3], macAddress[4],
               macAddress[5]);
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");
    status = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(status);
    
    return(status);
}

//*****************************************************************************
//
//! \brief    Start measure nice printing.            .
//!
//! \param  None.
//!
//! \return None.
//
//*****************************************************************************
void startMeasureBanner(void) 
{
    switch (PowerMeasure_appData.useCase) 
    {
        case UseCase_HIB :
            UART_PRINT("Entering Hibernate, Start measure current..!\n\r");
            break;
        case UseCase_LPDS :
            UART_PRINT("Entering LPDS, Start measure current..!\n\r");
            break;
        case UseCase_Transceiver :
            UART_PRINT("Starting Transceiver mode,"
                       " Start measure current..!\n\r");
            break;
        case UseCase_IntermittentlyConnected :
        UART_PRINT(
            "Starting Intermittentlly connected mode,"
            " Start measure current..!\n\r");
            break;
        case UseCase_AlwaysConnected :
            switch (PowerMeasure_appData.alwaysConnectedUseCase)
            {
                case UseCase_LSI :
            UART_PRINT(
                "Starting Always connected mode - LSI,"
                " Start measure current..!\n\r");
                    break;
                case UseCase_IotLowPower :
            UART_PRINT(
                "Starting Always connected mode - IOT low power,"
                " Start measure current..!\n\r");
                    break;
                default:
            UART_PRINT(
                "Starting Always connected mode - LPDS,"
                " Start measure current..!\n\r");
                    break;
            }
            break;
    }
    /* waiting for uart to flush out the buffer, 1ms timeout */
    usleep(1000);

}

//****************************************************************************
//
//! \brief Connecting to a WLAN Access point
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  None
//!
//! \warning    If the WLAN connection fails or we don't acquire an IP 
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
int32_t wlanConnect(void)
{
    SlWlanSecParams_t secParams = {0};
    int32_t status = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    status = sl_WlanConnect((signed char*)SSID_NAME, strlen(
                                SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(status);
    
    UART_PRINT("Trying to connect to AP : %s\n\r", SSID_NAME);
    sl_Task(NULL);
    // Wait for WLAN Event
    while((!IS_CONNECTED(PowerMeasure_CB.slStatus)) ||
          (!IS_IP_ACQUIRED(PowerMeasure_CB.slStatus)))
    { 
        /* Turn on user LED */
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        usleep(50000);
        /* Turn off user LED */
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        usleep(50000);
        sl_Task(NULL);
    }

    return(0);
   
}

//*****************************************************************************
//
//! Check the device mode and switch to STATION(STA) mode
//! restart the NWP to activate STATION mode
//!
//! \param  iMode (device mode)
//!
//! \return None
//
//*****************************************************************************
void switchToStaMode(int32_t mode)
{
    int32_t status = -1;
    
    if(mode != ROLE_STA)
    { 
        status = sl_WlanSetMode(ROLE_STA);
        if (status < 0)
        {
            ERR_PRINT(status);
            LOOP_FOREVER();
        }
        sl_Stop(SL_STOP_TIMEOUT);
        if (status < 0)
        {
            ERR_PRINT(status);
            LOOP_FOREVER();
        }
        //
        // Assumption is that the device is configured in station mode already
        // and it is in its default state
        //
        status = sl_Start(0, 0, 0);
        if (status < 0 || ROLE_STA != status)
        {
            UART_PRINT("Failed to start the device \n\r");
            LOOP_FOREVER();
        }
    }
    CLR_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_CONNECTION);
    CLR_STATUS_BIT(PowerMeasure_CB.slStatus, STATUS_BIT_IP_ACQUIRED);

}
   

//*****************************************************************************
//
//! \brief    Get from the user the selected Power Management use case.
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void setUseCase(void) 
{
    char sel[2];

    UART_PRINT("*** Power management use case Options : *** \n\r");
    UART_PRINT("    1) for Hibernate.  \n\r");
    UART_PRINT("    2) for LPDS.  \n\r");
    UART_PRINT("    3) for Transceiver Mode  \n\r");
    UART_PRINT("    4) for Intermittently Connected.  \n\r");
    UART_PRINT("    5) for Always Connected  \n\r");
    UART_PRINT("Please enter your Power management use case selection:  ");
    GetCmd(sel,sizeof(sel));
    UART_PRINT("\n\r");
    PowerMeasure_appData.useCase = (UseCases)(atoi((const char*)sel) - 1);
    return;
}

//*****************************************************************************
//
//! \brief    Get from the user the selected Always connected use case.
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void setAlwaysConnectedUseCase(void)
{
    char sel[2];

    UART_PRINT("*** Always connected use case Options : *** \n\r");
    UART_PRINT("    1) for LPDS.  \n\r");
    UART_PRINT("    2) for LSI.  \n\r");
    UART_PRINT("    3) for IoT Low Power.  \n\r");
    UART_PRINT("Please enter your Always connected use case selection:  ");
    GetCmd(sel,sizeof(sel));
    UART_PRINT("\n\r");
    PowerMeasure_appData.alwaysConnectedUseCase =
        (AlwaysConnectedUseCases)(atoi((const char*)sel) - 1);
    return;
}


//*****************************************************************************
//
//! \brief    Configure the LSI parameters according to user input.
//!
//! \param  None
//!
//! \return Success or Fail
//
//*****************************************************************************
int32_t configDuration(uint8_t *str, uint8_t min, uint16_t max,
                       SlWlanPmPolicyParams_t *PmPolicyParams)
{
    uint16_t        sel = 0;
    char            buf[6];

    UART_PRINT("*** %s *** \n\r", str);
    UART_PRINT(
        "Please enter your %s duration in milliseconds"
        "(min- %d msec, max- %d msec):  ",
        str, min, max);
    GetCmd(buf,sizeof(buf));
    sel= (uint16_t)(atoi((const char*)buf));
    UART_PRINT("\n\r");
    if ((sel >= min) && (sel <= max))
    {
        PmPolicyParams->MaxSleepTimeMs = sel;
    }
    else
    {
        ASSERT_ON_ERROR(PowerMeasurment_EINVALIDINPUT);
    }
    return(0);

}

//*****************************************************************************
//
//! \brief    Configure the device according to
//!           selected Power Management use case.
//!
//! \param  None
//!
//! \return Success or Fail
//
//*****************************************************************************
int32_t configSimplelinkToUseCase(void) 
{
    int32_t         status        = -1;
    uint32_t        ifBitmap      = 0;
    uint8_t         configOpt     = 0;
    SlWlanPmPolicyParams_t PmPolicyParams;
    SlWlanRxFilterOperationCommandBuff_t rxFilterIdMask = {
        {0}
    };

    memset(&PmPolicyParams,0,sizeof(SlWlanPmPolicyParams_t));
    PmPolicyParams.MaxSleepTimeMs = LSI_MIN_DURATION_IN_MSEC;

    status = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(status);
    
    /* Swtich to STA mode if device is not */
    switchToStaMode(status);
    
    /* disconnect from AP */
    status = sl_WlanDisconnect();
    if (status == 0)
    {
        ClockP_sleep(5);
        sl_Task(NULL);
        if((IS_CONNECTED(PowerMeasure_CB.slStatus)) ||
           (IS_IP_ACQUIRED(PowerMeasure_CB.slStatus)))
        {
            UART_PRINT("Error: disconnection\n\r");
            ASSERT_ON_ERROR(-1);
        }
    }
    /* Remove all profiles */
    status = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(status);
    
    /* Disable IPV6 */
    ifBitmap = 0;
    status = sl_NetCfgSet(SL_NETCFG_IF, SL_NETCFG_IF_STATE,\
            sizeof(ifBitmap), (uint8_t *)&ifBitmap);
    ASSERT_ON_ERROR(status);
    
    /* Disable scan */
    configOpt = SL_WLAN_SCAN_POLICY(0,0);
    status = sl_WlanPolicySet(SL_WLAN_POLICY_SCAN, configOpt, NULL, 0);
    ASSERT_ON_ERROR(status);
    
    /* Unregister mDNS services */
    status = sl_NetAppMDNSUnRegisterService(0, 0, 0);
    ASSERT_ON_ERROR(status);
    
    /* Remove  all 64 filters (8*8) */
    memset(rxFilterIdMask.FilterBitmap, 0xFF, 8);
    status = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_REMOVE,
                        sizeof(SlWlanRxFilterOperationCommandBuff_t),
                        (uint8_t *)&rxFilterIdMask);
    ASSERT_ON_ERROR(status);
    
    /* Set PM policy to normal */
    status = sl_WlanPolicySet(SL_WLAN_POLICY_PM ,
                              SL_WLAN_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(status);
    
    switch (PowerMeasure_appData.useCase)
    {
        case UseCase_Transceiver :
            /* No connection */
        status =
            sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION,
                             SL_WLAN_CONNECTION_POLICY(0, 0, 0,
                                                       0), NULL, 0);
            ASSERT_ON_ERROR(status);
            /* Low Power Policy */
        status =
            sl_WlanPolicySet(SL_WLAN_POLICY_PM, SL_WLAN_LOW_POWER_POLICY, NULL,
                             0);
            ASSERT_ON_ERROR(status);
            setHibRetentionReg(UseCase_Transceiver);
            break;
        case UseCase_IntermittentlyConnected:
            /* Connection policy Auto + fast */
        status =
            sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION,
                             SL_WLAN_CONNECTION_POLICY(1, 1, 0,
                                                       0), NULL, 0);
            ASSERT_ON_ERROR(status);
            setIpAddrAllocMode(IP_ADDR_ALLOC_MODE);
            break;
        case UseCase_AlwaysConnected :
            /* Connection policy Auto  */
        status =
            sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION,
                             SL_WLAN_CONNECTION_POLICY(1, 0, 0,
                                                       0), NULL, 0);
            ASSERT_ON_ERROR(status);
            setAlwaysConnectedUseCase();
            switch (PowerMeasure_appData.alwaysConnectedUseCase)
            {
                case UseCase_LSI :
                    /* LSI setting */
                    status = -1;
                    while (status < 0)
                    {
                status =
                    configDuration((uint8_t *)"LSI", (uint8_t)LSI_MIN_DURATION_IN_MSEC,
                                   (uint16_t)LSI_MAX_DURATION_IN_MSEC,
                                   &PmPolicyParams);
                    }
            sl_WlanPolicySet(SL_WLAN_POLICY_PM,
                             SL_WLAN_LONG_SLEEP_INTERVAL_POLICY,
                             (uint8_t *)&PmPolicyParams,
                             sizeof(PmPolicyParams));
                    break;
                case UseCase_IotLowPower :
                    /* IoT low power setting */
                    status = -1;
                    while (status < 0)
                    {
                status =
                    configDuration((uint8_t *)"IoT low power",
                                   (uint8_t)IOTLP_MIN_DURATION_IN_MSEC,
                                   (uint16_t)IOTLP_MAX_DURATION_IN_MSEC,
                                   &PmPolicyParams);
                    }
            status =
                sl_WlanPolicySet(SL_WLAN_POLICY_PM,
                                 SL_WLAN_IOT_LOW_POWER_POLICY,
                                 (uint8_t *)&PmPolicyParams,
                                 sizeof(PmPolicyParams));
                    if (status == SL_ERROR_WLAN_PM_POLICY_INVALID_OPTION)
                    {
                        UART_PRINT(" IoT low power not"
                                   " supported in CC32xx \n\r");
                        ASSERT_ON_ERROR(status);
                    }
                    break;
                default:
                    break;
            }
            setIpAddrAllocMode(IP_ADDR_ALLOC_MODE);
            break;
        default:
            /* No connection */
        status =
            sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION,
                             SL_WLAN_CONNECTION_POLICY(0, 0, 0,
                                                       0), NULL, 0);
            ASSERT_ON_ERROR(status);
            break;
    }
    status = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(status);

    return(status);
}

//*****************************************************************************
//
//! \brief    Set the IP address alloccation method for the connected use cases.
//!
//! \param  Mode (the allocation method)
//!
//! \return Success or Fail
//
//*****************************************************************************
int32_t setIpAddrAllocMode(uint8_t mode) 
{
    int32_t status = -1;
    SlNetCfgIpV4Args_t ipV4;

    switch (mode) 
    {
        case STATIC_IP :
            ipV4.Ip          = (uint32_t)SRC_IP_ADDR;
            ipV4.IpMask      = (uint32_t)SL_IPV4_VAL(255,255,255,0);
            ipV4.IpGateway   = (uint32_t)GATEWAY_IP_ADDR;
            ipV4.IpDnsServer = (uint32_t)GATEWAY_IP_ADDR;
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE,SL_NETCFG_ADDR_STATIC,
                         sizeof(SlNetCfgIpV4Args_t),
                         (uint8_t *)&ipV4);
            break;
        case DHCP_NO_FAST_RENEW :
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_DHCP, 0,
                         0);
            /* Disable the "fast renew feature" */
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE,
                         SL_NETCFG_ADDR_DISABLE_FAST_RENEW,
                         0,
                         0);
            break;
        case DHCP_FAST_RENEW_NO_ACK :
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_DHCP, 0,
                         0);
            /* Enable the "fast renew feature" + "NO wait" */
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE,
                         SL_NETCFG_ADDR_ENABLE_FAST_RENEW,
                         0,
                         0);
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE,
                         SL_NETCFG_ADDR_FAST_RENEW_MODE_NO_WAIT_ACK,
                         0,
                         0);
            break;
        case DHCP_FAST_RENEW_WAIT_ACK :
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_DHCP, 0,
                         0);
            /* Enable the "fast renew feature" + "wait ack" */
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE,
                         SL_NETCFG_ADDR_ENABLE_FAST_RENEW,
                         0,
                         0);
        status =
            sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE,
                         SL_NETCFG_ADDR_FAST_RENEW_MODE_WAIT_ACK,
                         0,
                         0);
            break;
    }
    return(status);
}

//*****************************************************************************
//
//! \brief This function practice Transceiver mode.
//!
//! \param None.
//!
//! \return 0 on success, -ve otherwise.
//
//*****************************************************************************
int32_t transceiverMode(void) 
{
    int32_t status = -1;
    int32_t socketId;
    int32_t idx;

    prepareDataFrame(PowerMeasure_appData.port,PowerMeasure_appData.ipAddr);

    status = sl_Start(0,0,0);
    ASSERT_ON_ERROR(status);
    if (CCA_BYPASS)
    {
        /* bypass the cca */
        socketId = sl_Socket(SL_AF_RF,SL_SOCK_RAW,0);
    } 
    else
    {
        /* consider cca */
        socketId = sl_Socket(SL_AF_RF,SL_SOCK_DGRAM,0);
    }
    
    for (idx = 0;idx < NUM_OF_PKT; idx++)
    {
        status =
            sl_Send(socketId,PowerMeasure_CB.frameData,FRAME_LENGTH,
                    (int16_t)(SL_WLAN_RAW_RF_TX_PARAMS(TAG_CHANNEL,
                                             TAG_FRAME_TRANSMIT_RATE,
                                             TAG_FRAME_TRANSMIT_POWER,
                                             1)));
        usleep(5000);
    }
    status = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(status);
    
    /* enter hibernate */
    powerShutdown(NOT_ACTIVE_DURATION_MSEC);
    
    /* this line is never Reached */
    return(status);
}

//*****************************************************************************
//
//! \brief This function practice Always connected mode.
//!
//! \param None.
//!
//! \return 0 on success, -ve otherwise.
//
//*****************************************************************************
int32_t alwaysConnected(void) 
{
    int32_t status = -1;

    switch (PowerMeasure_appData.socketType) 
    {
        case SocketType_UDP :
        status = bsdUdpClient(PowerMeasure_appData.port,
                              PowerMeasure_appData.sockID);
            ASSERT_ON_ERROR(status);
            break;
        case SocketType_TCP :
        status = bsdTcpClient(PowerMeasure_appData.port,
                              PowerMeasure_appData.sockID);
            ASSERT_ON_ERROR(status);
            break;
        case SocketType_SEC_TCP :
        status = bsdTcpSecClient(PowerMeasure_appData.port,
                                 PowerMeasure_appData.sockID);
            ASSERT_ON_ERROR(status);
            break;
    }
    return(status);
}

//*****************************************************************************
//
//! \brief This function practice Intermittently connected mode.
//!
//! \param None.
//!
//! \return 0 on success, -ve otherwise.
//
//*****************************************************************************
int32_t intermittentlyConnected(void) 
{
    int32_t status = -1;

    prepareDataFrame(PowerMeasure_appData.port,PowerMeasure_appData.ipAddr);

    status = sl_Start(0,0,0);
    ASSERT_ON_ERROR(status);

    /* waiting for fast connect proccess to compleate */
    sl_Task(NULL);
    while (!IS_IP_ACQUIRED(PowerMeasure_CB.slStatus))
    {
        sl_Task(NULL);
        ;
    }
    
    switch (PowerMeasure_appData.socketType) 
    {
        case SocketType_UDP :
            status = bsdUdpClient(PowerMeasure_appData.port,ALWAYS_OPEN_SOCK);
            ASSERT_ON_ERROR(status);
            break;
        case SocketType_TCP :
            status = bsdTcpClient(PowerMeasure_appData.port,ALWAYS_OPEN_SOCK);
            ASSERT_ON_ERROR(status);
            break;
        case SocketType_SEC_TCP :
            status = bsdTcpSecClient(PowerMeasure_appData.port,
                                     ALWAYS_OPEN_SOCK);
            ASSERT_ON_ERROR(status);
            break;
    }
    
    status = sl_Stop(SL_STOP_TIMEOUT);
    /* enter hibernate */
    powerShutdown(NOT_ACTIVE_DURATION_MSEC);
    /* this line is never Reached */
    /* clear SimpleLink Status  for the MSP432 Plugin,
       because we don't got to HIB */
    PowerMeasure_CB.slStatus = 0;
    return(status);
}

//*****************************************************************************
//
//! \brief This function implement UDP client .
//!
//! \param Port - socket port number; Sid - socket id,
//!               -ve if socket is alrady opened.
//!
//! \return 0 on success, -ve otherwise.
//
//*****************************************************************************
int32_t bsdUdpClient(uint16_t port, int16_t sid)
{
    int16_t         sockId;
    int16_t         idx = 0;
    int32_t         status = -1;
    
    if (sid < 0)
    {
        /* Need to open socket  */
        sockId = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
        ASSERT_ON_ERROR(sockId);
    }
    else
    {
        /* Socket is already opened */
        sockId = sid;
    }

    while (idx < NUM_OF_PKT)
    {
        status = sl_SendTo(sockId,PowerMeasure_CB.frameData ,FRAME_LENGTH , 0,
                           (SlSockAddr_t *)&PowerMeasure_CB.ipV4Addr,
                           sizeof(SlSockAddrIn_t));
        if(status <= 0 )
        {
            status = sl_Close(sockId);
            ASSERT_ON_ERROR(sockId);
        }
        idx++;
    }

    if (sid == ALWAYS_OPEN_SOCK)
    {
        /* Next time, use a new socket */
        status = sl_Close(sockId);
        ASSERT_ON_ERROR(status);
    }
    else
    {
        /* store the current open socket id*/
        PowerMeasure_appData.sockID = sockId;
    }

    return(0);
}

//*****************************************************************************
//
//! \brief This function implement TCP client .
//!
//! \param Port - socket port number; Sid - socket id,
//!               -ve if socket is alrady opened.
//!
//! \return 0 on success, -ve otherwise.
//
//*****************************************************************************
int32_t bsdTcpClient(uint16_t port, int16_t sid)
{
    int16_t         sockId;
    int16_t         idx = 0;
    int16_t         status = -1;

    if (sid < 0)
    {
        /* Need to open socket  */
        sockId = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
        ASSERT_ON_ERROR(sockId);
        /* Make connection establishment */
        status = sl_Connect(sockId, ( SlSockAddr_t *)&PowerMeasure_CB.ipV4Addr,
                            sizeof(SlSockAddrIn_t));
        if(status < 0)
        {
            sl_Close(sockId);
            ASSERT_ON_ERROR(sockId);
        }
    } 
    else
    {
        /* Socket is already opened */
        sockId = sid;
    }

    while (idx < NUM_OF_PKT)
    {
        status = sl_Send(sockId,PowerMeasure_CB.frameData ,FRAME_LENGTH, 0 );
        if(status <= 0 )
        {
            status = sl_Close(sockId);
            ASSERT_ON_ERROR(status);
        }
        idx++;
    }
    if (sid == ALWAYS_OPEN_SOCK) 
    {
        /* Next time, use a new socket */
        status = sl_Close(sockId);
        ASSERT_ON_ERROR(status);
        PowerMeasure_appData.sockID = -1;
    } 
    else 
    {
        /* store the current open socket id*/
        PowerMeasure_appData.sockID = sockId;
    }
    return(0);
}

//*****************************************************************************
//
//! \brief This function implement TLS/SLL on TCP client .
//!
//! \param Port - socket port number; Sid - socket id,
//!             -ve if socket is alrady opened.
//!
//! \return 0 on success, -ve otherwise.
//
//*****************************************************************************
int32_t bsdTcpSecClient(uint16_t port, int16_t sid) 
{
    uint32_t  cipher = SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA;
    uint8_t method = SL_SO_SEC_METHOD_SSLV3;
    int16_t sockId;
    int16_t idx = 0;
    int32_t  status = -1;

    if (sid < 0)
    { 
        /* need to open new socket */
        sockId = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
        ASSERT_ON_ERROR(sockId);
        /* Make connection establishment */
        status = sl_SetSockOpt(sockId, SL_SOL_SOCKET, SL_SO_SECMETHOD,
                                       &method, sizeof(method));
        if(status < 0 ) 
        {
            UART_PRINT(" Failed to configure the socket \n\r");
            ASSERT_ON_ERROR(status);
        }
        status = sl_SetSockOpt(sockId, SL_SOL_SOCKET, SL_SO_SECURE_MASK,
                &cipher, sizeof(cipher));
        if(status < 0 )
        {
            UART_PRINT(" Failed to configure the socket \n\r");
            ASSERT_ON_ERROR(status);
        }
        /* If the user flashed the server certificate 
		   the lines below can be uncomment  */
        /*
        status = sl_SetSockOpt(g_SockID, SL_SOL_SOCKET,
                               SL_SO_SECURE_FILES_CA_FILE_NAME,
                SL_SSL_CA_CERT, pal_Strlen(SL_SSL_CA_CERT));
        if( status < 0 ) 
        {
            UART_PRINT(" Failed to configure the socket \n\r");
            LOOP_FOREVER();
        }
        */
        /* connect to the peer server */
        status = sl_Connect(sockId, ( SlSockAddr_t *)&PowerMeasure_CB.ipV4Addr,
                            sizeof(SlSockAddrIn_t));
        if ((status < 0) && (status != SL_ERROR_BSD_ESECSNOVERIFY)) 
        { 
            /* ignore authentication error */
            UART_PRINT(" Failed to connect w/ server \n\r");
            LOOP_FOREVER();
        }
        UART_PRINT(" Connection w/ server established successfully \n\r");
    } 
    else 
    {
        sockId = sid;
    }

    while (idx < NUM_OF_PKT)
    {
        status = sl_Send(sockId, PowerMeasure_CB.frameData, FRAME_LENGTH, 0 );
        if( status <= 0 )
        {
            UART_PRINT(" [TCP Client] Data send Error \n\r");
            status = sl_Close(sockId);
            ASSERT_ON_ERROR(status);
        }
        idx++;
    }
    
    if (sid == ALWAYS_OPEN_SOCK) 
    { 
        /* Next time, use a new socket */
        status = sl_Close(sockId);
        ASSERT_ON_ERROR(status);
    } 
    else 
    {
        /* store the current open socket id */
        PowerMeasure_appData.sockID = sockId;
    }
    return(0);
}
//*****************************************************************************
//
//! \brief    This function prepare the Data Frame & initialize IP address data
//!         struct            .
//!
//! \param  Port & IP addressnn
//!
//! \return None
//
//*****************************************************************************
void prepareDataFrame(uint16_t port,uint32_t ipAddr) 
{
    /*
    uint16_t idx;
    
    for(idx = 0;idx < FRAME_LENGTH;idx++) 
    {
        PowerMeasure_CB.frameData[idx] = (signed char)(idx % 255);
    }
    */
    PowerMeasure_CB.ipV4Addr.sin_family = SL_AF_INET;
    PowerMeasure_CB.ipV4Addr.sin_port = sl_Htons(port);
    PowerMeasure_CB.ipV4Addr.sin_addr.s_addr = sl_Htonl(ipAddr);
}
//*****************************************************************************
//
//! \brief  Do nothing
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void powerMeasureDoNothing()
{
 //Do nothing ...
}

