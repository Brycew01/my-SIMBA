#ifndef SERVER_H
#define SERVER_H

#include "serverUtil.h"
#include "rover.h"
#include "arm.h"
#include "vision.h"


#define PORT 6500

typedef struct connection {
  uint32_t sk_num;
  struct sockaddr_in6 remote;
  uint32_t len;
} Connection;

// Functions
void addNewSocket(int socket);
void processMsgFromClient(int socket);
void serverControl(int mainSocketNumber);

#endif // SERVER_H
