#ifndef SERVER_UTIL_H
#define SERVER_UTIL_H

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <errno.h>

#include "rover.h"

#define MAXBUF 1024
#define DEBUG_FLAG 1
#define LISTEN_BACKLOG 10


// Comms
int sendPDU(int clientSocket, uint8_t *dataBuffer, int lengthOfData);
int recvPDU(int socketNumber, uint8_t *dataBuffer, int bufferSize);

// Network Functions
int tcpServerSetup(int serverPort);
int tcpAccept(int mainServerSocket, int debugFlag);
char *getIPAddressString6(unsigned char *ipAddress);
char *getIPAddressString46(unsigned char *ipAddress, int addressFamily);

// Rover Functions
void handleMoveRover(char *payload, int64_t speed);

#endif // SERVER_UTIL_H