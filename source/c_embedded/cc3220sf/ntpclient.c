/*
 *
 * (C) 2014 David Lettier.
 *
 * http://www.lettier.com/
 *
 * NTP client.
 *
 * Compiled with gcc version 4.7.2 20121109 (Red Hat 4.7.2-8) (GCC).
 *
 * Tested on Linux 3.8.11-200.fc18.x86_64 #1 SMP Wed May 1 19:44:27 UTC 2013 x86_64 x86_64 x86_64 GNU/Linux.
 *
 * To compile: $ gcc main.c -o ntpClient.out
 *
 * Usage: $ ./ntpClient.out
 *
 */

/*
 *  ======== ntpclient.c ========
 *  Built with:
 *  Code Composer Studio 10.1.1.00004
 *  SimpleLink CC32xx SDK 4.30.0.06
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <netdb.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/netapp.h>

//#define NTP_TIMESTAMP_DELTA 2208988800ull
#define NTP_TIMESTAMP_DELTA     (0)

#define LI(packet)   (uint8_t) ((packet.li_vn_mode & 0xC0) >> 6) // (li   & 11 000 000) >> 6
#define VN(packet)   (uint8_t) ((packet.li_vn_mode & 0x38) >> 3) // (vn   & 00 111 000) >> 3
#define MODE(packet) (uint8_t) ((packet.li_vn_mode & 0x07) >> 0) // (mode & 00 000 111) >> 0

#define CC32XX_SIMPLELINK       (1)

    // Structure that defines the 48 byte NTP packet protocol.

typedef struct
{
    uint8_t li_vn_mode;      // Eight bits. li, vn, and mode.
                             // li.   Two bits.   Leap indicator.
                             // vn.   Three bits. Version number of the protocol.
                             // mode. Three bits. Client will pick mode 3 for client.

    uint8_t stratum;         // Eight bits. Stratum level of the local clock.
    uint8_t poll;            // Eight bits. Maximum interval between successive messages.
    uint8_t precision;       // Eight bits. Precision of the local clock.

    uint32_t rootDelay;      // 32 bits. Total round trip delay time.
    uint32_t rootDispersion; // 32 bits. Max error aloud from primary clock source.
    uint32_t refId;          // 32 bits. Reference clock identifier.

    uint32_t refTm_s;        // 32 bits. Reference time-stamp seconds.
    uint32_t refTm_f;        // 32 bits. Reference time-stamp fraction of a second.

    uint32_t origTm_s;       // 32 bits. Originate time-stamp seconds.
    uint32_t origTm_f;       // 32 bits. Originate time-stamp fraction of a second.

    uint32_t rxTm_s;         // 32 bits. Received time-stamp seconds.
    uint32_t rxTm_f;         // 32 bits. Received time-stamp fraction of a second.

    uint32_t txTm_s;         // 32 bits and the most important field the client cares about. Transmit time-stamp seconds.
    uint32_t txTm_f;         // 32 bits. Transmit time-stamp fraction of a second.

} ntp_packet;              // Total: 384 bits or 48 bytes.

#ifdef CC32XX_SIMPLELINK
    // Socket control block definition
    typedef struct _NTP_ControlBlock_t_
    {
        uint32_t        slStatus;    //SimpleLink Status
        SlSockAddrIn_t  ipV4Addr;
    }NTP_ControlBlock;
#else
    // http://web.mit.edu/macdev/Development/MITSupportLib/SocketsLib/Documentation/structures.html
    typedef uint32_t  in_addr_t;

    struct in_addr {
        in_addr_t s_addr;               the IP address in network byte order
    };

    struct sockaddr_in {
        uint16_t        sin_family;     // always AF_INET
        uint16_t        sin_port;       // the service port
        struct in_addr  sin_addr;       // the IP address
        char            sin_zero[8];    // unused (reserved for expansion
#endif

//****************************************************************************
// GLOBAL VARIABLES
//****************************************************************************
volatile time_t g_txTm;        // Stores Transmit time-stamp seconds

//****************************************************************************
// EXTERNAL FUNCTIONS
//****************************************************************************
extern int Report(const char *pcFormat, ...);
/*
void error( char* msg )
{
    perror( msg ); // Print the error message to stderr.

    exit( 0 ); // Quit the process.
}
*/

/*
 * Get current UTC time from NTP time source
 * Returns time_t
 * Total number of seconds since epoch
 * int ex_main( int argc, char* argv[ ] )
 */
time_t getNTPTime( void )
{
    //int sockfd, n;    // Socket file descriptor and the n return result from writing/reading from the socket.
    int16_t sockId;     // Socket file descriptor
    int n;              // Return result from writing/reading from the socket.
    int status;         // Stores socket connection status
    int portno = 123;   // NTP UDP port number.
    char* host_name = "us.pool.ntp.org"; // NTP server host-name.
    unsigned long DestinationIP; // host resolved from host_name
    volatile time_t g_txTm;      // Stores Transmit time-stamp seconds

    // Create and zero out the packet. All 48 bytes worth.

    ntp_packet packet = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    memset( &packet, 0, sizeof( ntp_packet ) );

    // Set the first byte's bits to 00,011,011 for li = 0, vn = 3, and mode = 3. The rest will be left set to zero.

    *( ( char * ) &packet + 0 ) = 0x1b; // Represents 27 in base 10 or 00011011 in base 2.

    // Create a UDP socket, convert the host-name to an IP address, set the port number,
    // connect to the server, send the packet, and then read in the return packet.

#ifdef CC32XX_SIMPLELINK
    NTP_ControlBlock   NTP_CB;      // Server address data structure.
#else
    struct sockaddr_in serv_addr;   // Server address data structure.
    struct hostent *server;         // Server data structure.
#endif

    //sockfd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP ); // Create a UDP socket.
    sockId = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);

    //if ( sockfd < 0 )
    if (sockId < 0) {
        //error( "ERROR opening socket" );
        Report("ERROR opening socket\n\r");
    }

    //server = gethostbyname( host_name ); // Convert URL to IP.
    status = sl_NetAppDnsGetHostByName((signed char *)host_name,
                            strlen(host_name),
                            &DestinationIP,
                            SL_AF_INET);

    //if ( server == NULL )
    if (status < 0) {
        //error( "ERROR, no such host" );
        Report("ERROR, no such host\n\r");
    }

    // Zero out the server address structure.

    //bzero( ( char* ) &serv_addr, sizeof( serv_addr ) );

    //serv_addr.sin_family = AF_INET;
    NTP_CB.ipV4Addr.sin_family = SL_AF_INET;

    // Copy the server's IP address to the server address structure.

    //bcopy( ( char* )server->h_addr, ( char* ) &serv_addr.sin_addr.s_addr, server->h_length );
    NTP_CB.ipV4Addr.sin_addr.s_addr = sl_Htonl(DestinationIP);

    // Convert the port number integer to network big-endian style and save it to the server address structure.

    //serv_addr.sin_port = htons( portno );
    NTP_CB.ipV4Addr.sin_port = sl_Htons(portno);

    // Call up the server using its IP address and port number.

    /*
    if ( connect( sockfd, ( struct sockaddr * ) &serv_addr, sizeof( serv_addr) ) < 0 )
       error( "ERROR connecting" );
    */
    status = sl_Connect(sockId,
                        ( SlSockAddr_t *)&NTP_CB.ipV4Addr,
                        sizeof(SlSockAddrIn_t));
    if ( status < 0 ) {
      Report("ERROR connecting\n\r");
    }

    // Send it the NTP packet it wants. If n == -1, it failed.

    //n = write( sockfd, ( char* ) &packet, sizeof( ntp_packet ) );
    n = sl_SendTo(sockId,
                &packet,
                sizeof( ntp_packet ),
                0,
                (SlSockAddr_t *)&NTP_CB.ipV4Addr,
                sizeof(SlSockAddrIn_t));

    if (n < 0) {
        //error( "ERROR writing to socket" );
        Report("ERROR writing to socket\n\r");
    }

    // Wait and receive the packet back from the server. If n == -1, it failed.

    //n = read( sockfd, ( char* ) &packet, sizeof( ntp_packet ) );
    n = sl_Recv(sockId,
              &packet,
              sizeof( ntp_packet ),
              0);

    if ( n < 0 ) {
      //error( "ERROR reading from socket" );
      Report("ERROR reading from socket\n\r");
    }

    // These two fields contain the time-stamp seconds as the packet left the NTP server.
    // The number of seconds correspond to the seconds passed since 1900.
    // ntohl() converts the bit/byte order from the network's to host's "endianness".

    //packet.txTm_s = ntohl( packet.txTm_s ); // Time-stamp seconds.
    packet.txTm_s = sl_Ntohl( packet.txTm_s ); // Time-stamp seconds.
    //packet.txTm_f = ntohl( packet.txTm_f ); // Time-stamp fraction of a second.
    packet.txTm_f = sl_Ntohl( packet.txTm_f ); // Time-stamp fraction of a second.

    // Extract the 32 bits that represent the time-stamp seconds (since NTP epoch) from when the packet left the server.
    // Subtract 70 years worth of seconds from the seconds since 1900.
    // This leaves the seconds since the UNIX epoch of 1970.
    // (1900)------------------(1970)**************************************(Time Packet Left the Server)

    //time_t txTm = ( time_t ) ( packet.txTm_s - NTP_TIMESTAMP_DELTA );
    g_txTm = ( time_t ) ( packet.txTm_s - NTP_TIMESTAMP_DELTA );

    // Print the time we got from the server, accounting for local timezone and conversion from UTC time.

    //printf( "Time: %s", ctime( ( const time_t* ) &txTm ) );
    //Report("%sn\r", ctime( ( const time_t* ) &g_txTm ));

    status = sl_Close(sockId);

    //return 0;
    return g_txTm;
}
