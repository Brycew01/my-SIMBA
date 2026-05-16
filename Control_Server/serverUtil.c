/* SERVER SUPPORT FUNCTIONS */
#include "serverUtil.h"

//********** COMMS **********//

/*
 * This function should create the application level PDU, send() the PDU
 * in one send() call, and check for errors on the send() (<0). PDU is the
 * same as a packet.
 */
int sendPDU(int clientSocket, uint8_t *dataBuffer, int lengthOfData) {
  int res;
  // send PDU
  if ((res = send(clientSocket, dataBuffer, lengthOfData, 0)) < 0) {
    perror("sendPDU failed");
    exit(-1);
  }
  return res;
}

/*
 * This function includes checking for recv() errors (return value <0), checking
 * for closed connections (return value==0) and does the two step recv() process
 * (e.g. using MSG_WAITALL).
 */

int recvPDU(int socketNumber, uint8_t *dataBuffer, int bufferSize) {
  int recvBytes;

  if ((recvBytes = recv(socketNumber, dataBuffer, bufferSize, MSG_WAITALL)) <
      0) {
    perror("Error from receive");
    exit(-1);
  } else if (recvBytes == 0) {
    printf("Connection closed\n");
  }

  return recvBytes;
}

//********** Setup Functions **********//

/*
 * This function sets the server socket. The function returns the server
 * socket number and prints the port number to the screen.
 */

int tcpServerSetup(int serverPort) {
  // Opens a server socket, binds that socket, prints out port, call listens
  // returns the mainServerSocket

  int mainServerSocket = 0;
  struct sockaddr_in6 serverAddress;
  socklen_t serverAddressLen = sizeof(serverAddress);

  mainServerSocket = socket(AF_INET6, SOCK_STREAM, 0);
  if (mainServerSocket < 0) {
    perror("socket call");
    exit(1);
  }

  memset(&serverAddress, 0, sizeof(struct sockaddr_in6));
  serverAddress.sin6_family = AF_INET6;
  serverAddress.sin6_addr = in6addr_any;
  serverAddress.sin6_port = htons(serverPort);

  // bind the name (address) to a port
  if (bind(mainServerSocket, (struct sockaddr *)&serverAddress,
           sizeof(serverAddress)) < 0) {
    perror("bind call");
    exit(-1);
  }

  // get the port name and print it out
  if (getsockname(mainServerSocket, (struct sockaddr *)&serverAddress,
                  &serverAddressLen) < 0) {
    perror("getsockname call");
    exit(-1);
  }

  if (listen(mainServerSocket, LISTEN_BACKLOG) < 0) {
    perror("listen call");
    exit(-1);
  }

  printf("Server Port Number %d \n", ntohs(serverAddress.sin6_port));

  return mainServerSocket;
}

/*
 * This function waits for a client to ask for services.  It returns
 * the client socket number.
 */
int tcpAccept(int mainServerSocket, int debugFlag) {
  struct sockaddr_in6 clientAddress;
  int clientAddressSize = sizeof(clientAddress);
  int client_socket = 0;

  if ((client_socket =
           accept(mainServerSocket, (struct sockaddr *)&clientAddress,
                  (socklen_t *)&clientAddressSize)) < 0) {
    perror("accept call");
    exit(-1);
  }

  if (debugFlag) {
    printf("Client accepted.  Client IP: %s Client Port Number: %d\n",
           getIPAddressString6(clientAddress.sin6_addr.s6_addr),
           ntohs(clientAddress.sin6_port));
  }

  return (client_socket);
}

/*
 * Wrapper for getIPAddressString46()
 */
char *getIPAddressString6(unsigned char *ipAddress) {
  return getIPAddressString46(ipAddress, AF_INET6);
}

/*
 * This function returns a string of the IP address
 */
char *getIPAddressString46(unsigned char *ipAddress, int addressFamily) {
  // makes it easy to print the IP address (v4 or v6)
  static char ipString[INET6_ADDRSTRLEN];

  if (ipAddress != NULL) {
    inet_ntop(addressFamily, ipAddress, ipString, sizeof(ipString));
  } else {
    strcpy(ipString, "(IP not found)");
  }
  return ipString;
}

//********** Rover Functions **********//

/*
 * Handles user movement input
 */
 void handleMoveRover(char *payload, int64_t speed){
  if (strcmp(payload, "ArrowUp") == 0) {
    printf("Received up key\n");
    rover_stop();
    rover_steer_forward();
    while(check_rover_done() == 0){};
    rover_forward(64);
  } else if (strcmp(payload, "ArrowDown") == 0) {
    printf("Received down key\n");
    rover_stop();
    rover_steer_forward();
    while(check_rover_done() == 0){};
    rover_reverse(64);
  } else if (strcmp(payload, "ArrowLeft") == 0) {
    printf("Received left key\n");
    // rover_steer_left(300);
    rover_stop();
    rover_steer_point();
    rover_pointTurn_CCW(64);
  } else if (strcmp(payload, "ArrowRight") == 0) {
    printf("Received right key\n");
    // rover_steer_right(300);
    rover_stop();
    rover_steer_point();
    rover_pointTurn_CW(64);
  } else if (strcmp(payload, "Stop") == 0) {
    printf("Received stop\n");
    rover_stop();
  } 
}