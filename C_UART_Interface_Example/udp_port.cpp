//
//  udp_port.cpp
//  C_UART_Interface_Example
//
//  Created by kangzhiyong on 2020/3/3.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#include "udp_port.hpp"

UDP_Port::UDP_Port(const char *ip , int port)
{
    initialize_defaults();
    _dest_ip = ip;
    _dest_port  = port;
}

UDP_Port::UDP_Port()
{
    initialize_defaults();
}

UDP_Port::~UDP_Port()
{
    // destroy mutex
    pthread_mutex_destroy(&lock);
}

void UDP_Port::initialize_defaults()
{
    // Initialize attributes
    debug  = false;
    fd     = -1;
    status = UDP_PORT_CLOSED;

    // Start mutex
    int result = pthread_mutex_init(&lock, NULL);
    if ( result != 0 )
    {
        printf("\n mutex init failed\n");
        throw 1;
    }
}


// ------------------------------------------------------------------------------
//   Read from UDP
// ------------------------------------------------------------------------------
int UDP_Port::read_message(mavlink_message_t &message)
{
    bool msgReceived = false;
    mavlink_status_t status;
    unsigned char buf[MAX_UDP_PACKET_SIZE];
    memset(buf, 0, MAX_UDP_PACKET_SIZE);
    int n = _read_port(buf, MAX_UDP_PACKET_SIZE);
    if (n > 0)
    {
        for (unsigned int i = 0; i < n; ++i)
        {
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1, buf[i], &message, &status);
            if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
            {
              printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
              fprintf(stderr,"%02x ", buf[i]);
            }
            lastStatus = status;
        }
    }
    

    // --------------------------------------------------------------------------
    //   DEBUGGING REPORTS
    // --------------------------------------------------------------------------
    if(msgReceived && debug)
    {
        // Report info
        printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

        fprintf(stderr,"Received serial data: ");
        unsigned int i;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        // check message is write length
        unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

        // message length error
        if (messageLength > MAVLINK_MAX_PACKET_LEN)
        {
            fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
        }

        // print out the buffer
        else
        {
            for (i=0; i<messageLength; i++)
            {
                unsigned char v=buffer[i];
                fprintf(stderr,"%02x ", v);
            }
            fprintf(stderr,"\n");
        }
    }

    // Done!
    return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to UDP
// ------------------------------------------------------------------------------
int UDP_Port::write_message(const mavlink_message_t &message)
{
    char buf[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    // Write buffer to serial port, locks port while writing
    int bytesWritten = _write_port(buf,len);

    return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Open Serial Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void UDP_Port::open_udp()
{

    // --------------------------------------------------------------------------
    //   OPEN UDP
    // --------------------------------------------------------------------------
    printf("OPEN UDP\n");

    _open_udp();

    // Check success
    if (fd == -1)
    {
        printf("failure, could not open fd.\n");
        throw EXIT_FAILURE;
    }

    if (fd <= 0)
    {
        printf("Bind attempt to port %d failed, exiting.\n", LOCAL_PORT);
        throw EXIT_FAILURE;
    }

    // --------------------------------------------------------------------------
    //   CONNECTED!
    // --------------------------------------------------------------------------
    printf("Bind to port:%d\n", LOCAL_PORT);
    lastStatus.packet_rx_drop_count = 0;

    status = true;

    printf("\n");
    
    return;

}


// ------------------------------------------------------------------------------
//   Close UDP
// ------------------------------------------------------------------------------
void UDP_Port::close_udp()
{
    printf("CLOSE UDP\n");

    int result = close(fd);

    if ( result )
    {
        fprintf(stderr,"WARNING: Error on fd close (%i)\n", result );
    }

    status = false;

    printf("\n");

}


// ------------------------------------------------------------------------------
//   Convenience Functions
// ------------------------------------------------------------------------------
void UDP_Port::start()
{
    open_udp();
}

void UDP_Port::stop()
{
    close_udp();
}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void UDP_Port::handle_quit( int sig )
{
    try {
        stop();
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop fd\n");
    }
}


// ------------------------------------------------------------------------------
//   Helper Function - Open UDP File Descriptor
// ------------------------------------------------------------------------------
int UDP_Port::_open_udp()
{
    int val = 1;
    fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val));
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(MULTIADDR);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
    {
        perror("setsockopt MEMBERSHIP");
        ::close(fd);
        return -1;
    }
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));  // Zero out address structure
    addr.sin_family = PF_INET;       // Internet address
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(LOCAL_PORT);     // Assign port in network byte order

    if (::bind(fd, (sockaddr *) &addr, sizeof(sockaddr_in)) < 0)
    {
        perror("bind:");
        ::close(fd);
        return -1;
    }
    
    return fd;
}

// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int UDP_Port::_read_port(void *buffer, int bufferLen)
{

    // Lock
    pthread_mutex_lock(&lock);
    
    int result = ::recv(fd, buffer, bufferLen, 0);
    
    // Unlock
    pthread_mutex_unlock(&lock);

    return result;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int UDP_Port::_write_port(char *buf, unsigned len)
{

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));  // Zero out address structure
    addr.sin_family = PF_INET;       // Internet address
    addr.sin_addr.s_addr = inet_addr(_dest_ip.c_str());
    addr.sin_port = htons(_dest_port);     // Assign port in network byte order
    
    // Lock
    pthread_mutex_lock(&lock);

    // Write packet via udp link
    int bytesWritten = ::sendto(fd, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    
    // Unlock
    pthread_mutex_unlock(&lock);


    return bytesWritten;
}
