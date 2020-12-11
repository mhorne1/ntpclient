#ifndef RMS_NORTOS_H_
#define RMS_NORTOS_H_

//*****************************************************************************
// Defines
//*****************************************************************************

#define APPLICATION_NAME            "No RTOS"
#define APPLICATION_VERSION         "1.0.0.3"

#define FRAME_LENGTH                SERVER_FRAME_LENGTH
#define OPEN_SOCK_ONCE              (-2)
#define ALWAYS_OPEN_SOCK            (-1)
#define STATIC_IP                   (0)
#define DHCP_NO_FAST_RENEW          (1)
#define DHCP_FAST_RENEW_NO_ACK      (2)
#define DHCP_FAST_RENEW_WAIT_ACK    (3)

/* USER's defines */
/* options-> UseCase_HIB, UseCase_LPDS, UseCase_Transceiver,
   UseCase_IntermittentlyConnected, UseCase_AlwaysConnected */
#define PM_USECASE                  UseCase_AlwaysConnected
/* options-> UseCase_Normal, UseCase_LSI, UseCase_IotLowPower */
#define AC_USECASE                  UseCase_Normal
/* options -> SocketType_UDP , SocketType_TCP, SocketType_SEC_TCP */
#define SOCKET_TYPE                 SocketType_TCP
#define PORT                        SERVER_PORT
#define DEST_IP_ADDR                SERVER_DEST_IP_ADDR
/* relevant for Static IP mode */
#define SRC_IP_ADDR                 SL_IPV4_VAL(192,168,39,1)
/* relevant for Static IP mode */
#define GATEWAY_IP_ADDR             SL_IPV4_VAL(192,168,39,242)
#define NUM_OF_PKT                  (1)
/* options -> STATIC_IP, DHCP_NO_FAST_RENEW,
   DHCP_FAST_RENEW_NO_ACK, DHCP_FAST_RENEW_WAIT_ACK */
#define IP_ADDR_ALLOC_MODE          DHCP_FAST_RENEW_NO_ACK
#define NOT_ACTIVE_DURATION_MSEC    (5000)  /* 5 seconds */
#define LPDS_IDLE_TIME              (5000)  /* 5 seconds */
#define LSI_MIN_DURATION_IN_MSEC    (100)
#define LSI_MAX_DURATION_IN_MSEC    (2000)
#define IOTLP_MIN_DURATION_IN_MSEC  (100)
#define IOTLP_MAX_DURATION_IN_MSEC  (255000)
/* Tag setting defines */
/* Ignore the clear-channel-assessment indication */
#define CCA_BYPASS                  (1)
#define TAG_FRAME_TRANSMIT_RATE     (6)
#define TAG_FRAME_TRANSMIT_POWER    (7)
#define TAG_CHANNEL                 (1)

#define SL_STOP_TIMEOUT             (200)

/*
 *  ======== TMP Registers ========
 */
#define TMP006_REG          0x0001  /* Die Temp Result Register for TMP006 */
#define TMP_BP_REG          0x0000  /* Die Temp Result Register for BP TMP sensor */

/*
 *  The CC32XX LaunchPads come with an on-board TMP006 or TMP116 temperature
 *  sensor depending on the revision. Newer revisions come with the TMP116.
 *  The Build Automation Sensors (BP-BASSESENSORSMKII) BoosterPack
 *  contains a TMP117.
 *
 *  We are using the DIE temperature because it's cool!
 *
 *  Additionally: no calibration is being done on the TMPxxx device to simplify
 *  the example code.
 */
#define TMP006_ADDR         0x41;
#define TMP_BP_ADDR         0x48;
#define TMP116_LP_ADDR      0x49;

//*****************************************************************************
// Typedefs
//*****************************************************************************
typedef enum
{                       // RMS states
    STATE_simlink = 1,  // Run
    STATE_delay,        //
    STATE_redled,       //
    STATE_temp,         //
    STATE_accel,        //
    STATE_time,         //
    STATE_message,      //
    STATE_tcpclient,    //
    STATE_last,      // Send TCP Client message, requires WIFI connection and time
}RMSStateCases;

typedef enum
{
    UseCase_HIB,
    UseCase_LPDS,
    UseCase_Transceiver,
    UseCase_IntermittentlyConnected,
    UseCase_AlwaysConnected
}UseCases;

typedef enum
{
    UseCase_Normal,
    UseCase_LSI,
    UseCase_IotLowPower
}AlwaysConnectedUseCases;

typedef enum
{
    SocketType_UDP,
    SocketType_TCP,
    SocketType_SEC_TCP
}SocketTypes;



typedef struct _PowerMeasure_AppData_t_
{   /* The exercised use case  */
    UseCases                       useCase;
    /* The always connected use case  */
    AlwaysConnectedUseCases        alwaysConnectedUseCase;
    /* how many packet to transmit on each interval of this use case */
    uint32_t                       pktsToDo;
    /* socket ID*/
    int32_t                        sockID;
    /* Socket type */
    SocketTypes                    socketType;
    /* IP address */
    uint32_t                       ipAddr;
    /* socket port number */
    uint32_t                       port;

}PowerMeasure_AppData;

/* Control block definition */
typedef struct _PowerMeasure_ControlBlock_t_
{
    uint32_t        slStatus;    //SimpleLink Status
    signed char     frameData[FRAME_LENGTH];
    SlSockAddrIn_t  ipV4Addr;
}PowerMeasure_ControlBlock;


//*****************************************************************************
// Function prototypes
//*****************************************************************************
void powerMeasureDoNothing();
int32_t displayBanner(void);
void startMeasureBanner(void);
int32_t wlanConnect(void);
void switchToStaMode(int32_t mode);
void setUseCase(void);
void setAlwaysConnectedUseCase(void);
int32_t configDuration(uint8_t *str,
                       uint8_t min,
                       uint16_t max,
                       SlWlanPmPolicyParams_t *PmPolicyParams);
int32_t configSimplelinkToUseCase(void);
int32_t setIpAddrAllocMode(uint8_t  mode);
int32_t transceiverMode(void);
int32_t intermittentlyConnected(void);
int32_t alwaysConnected(void);
int32_t bsdUdpClient(uint16_t Port,
                     int16_t Sid);
int32_t bsdTcpClient(uint16_t Port,
                     int16_t Sid);
int32_t bsdTcpSecClient(uint16_t Port,
                        int16_t Sid);
void prepareDataFrame(uint16_t Port,
                      uint32_t IpAddr);

#endif /* RMS_NORTOS_H_ */
