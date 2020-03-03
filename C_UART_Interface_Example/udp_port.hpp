//
//  udp_port.hpp
//  C_UART_Interface_Example
//
//  Created by kangzhiyong on 2020/3/3.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#ifndef udp_port_hpp
#define udp_port_hpp

#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/fcntl.h>
#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>
#include <string>
using namespace std;
#include <common/mavlink.h>

#define MAX_UDP_PACKET_SIZE 65467 // UDP protocol max message size

#define LOCAL_PORT  14550
#define DEST_IP "192.168.4.1"
#define DEST_PORT 14555

// Status flags
#define UDP_PORT_OPEN   1;
#define UDP_PORT_CLOSED 0;
#define UDP_PORT_ERROR -1;
#define MULTIADDR  "224.0.0.1"

class UDP_Port
{

public:

    UDP_Port();
    UDP_Port(const char *dest_ip, int dest_port);
    void initialize_defaults();
    ~UDP_Port();

    bool debug;
    string _dest_ip{DEST_IP};
    int  _dest_port{DEST_PORT};
    int  status;

    int read_message(mavlink_message_t &message);
    int write_message(const mavlink_message_t &message);

    void open_udp();
    void close_udp();

    void start();
    void stop();

    void handle_quit( int sig );

private:

    int  fd;
    mavlink_status_t lastStatus;
    pthread_mutex_t  lock;

    int  _open_udp();
    int  _read_port(void *buffer, int bufferLen);
    int _write_port(char *buf, unsigned len);

};
#endif /* udp_port_hpp */
