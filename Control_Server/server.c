#include "server.h"
#include "pollLib.h"
#include "serverUtil.h"
// NOTE: PDU is the same as packet

// Global Variables
int workerSocket;
int64_t distance;
int64_t speed;

/*
 * Accepts new clients (support function to serverControl)
 */
void addNewSocket(int socket) {
  struct sockaddr_in6 *serverAddr;
  socklen_t addrLen = sizeof(serverAddr);
  if ((workerSocket =
           accept(socket, (struct sockaddr *)&serverAddr, &addrLen)) < 0) {
    perror("Accept failed");
    exit(-1);
  }
  addToPollSet(workerSocket);

  // Send acknowledgment back to client
  char ackMessage[] = "ACK-0";
  sendPDU(workerSocket, (uint8_t *)ackMessage, strlen(ackMessage));
  printf("New Connection...\n");
}

/*
 * Process message received from client
 */
void processMsgFromClient(int socket) {
  uint8_t dataBuffer[MAXBUF];
  int recvBytes = recvPDU(socket, dataBuffer, MAXBUF);
  char *ackMessage = NULL;

  if (recvBytes == 0) {
    // Client side closed
    close(socket);
    removeFromPollSet(socket);
  } else {
    char *synch = strtok((char *)dataBuffer, "$");

    // Check if synch is SIMBA indicating valid command
    if (strcmp(synch, "SIMBA") == 0) {
      int flg = atoi(strtok(NULL, "$"));
      char *payload = strtok(NULL, "$");
      switch (flg) {
      case 1:
        printf("PING\n");
        ackMessage = "ACK-1";
        sendPDU(workerSocket, (uint8_t *)ackMessage, strlen(ackMessage));
        break;
      case 2:
        ackMessage = "ACK-2";
        sendPDU(workerSocket, (uint8_t *)ackMessage, strlen(ackMessage));
        handleMoveRover(payload, speed);
        break;
      case 3:
        ackMessage = "ACK-3";
        sendPDU(workerSocket, (uint8_t *)ackMessage, strlen(ackMessage));
        speed = atoi(payload);
        // printf("Speed set: %lld\n", speed);
        break;
      case 4:
        ackMessage = "ACK-4";
        sendPDU(workerSocket, (uint8_t *)ackMessage, strlen(ackMessage));
        distance = atoi(payload);
        // printf("Distance set: %lld\n", distance);
        break;
      case 5:
        ackMessage = "ACK-5";
        sendPDU(workerSocket, (uint8_t *)ackMessage, strlen(ackMessage));
        printf("Received %s\n", payload);
	      arm_begin_pickup();
        break;
      case 6:
        ackMessage = arm_pickup_done() ? "True" : "False";
        sendPDU(workerSocket, (uint8_t *)ackMessage, strlen(ackMessage));
        printf("Received %s\n", payload);
        break;
      
      default:
        printf("Unknown message\n");
        break;
      }
    }
  }
}

/*
 * Accepts new clients while receiving packets from active clients
 */
void serverControl(int mainSocketNumber) {
  int socket;
  setupPollSet();
  addToPollSet(mainSocketNumber);

  while (1) {
    while ((socket = pollCall(-1)) == -1) {
      if (errno != EINTR) {
        perror("pollCall");
        return; // Exit loop on error other than interruption
      }
    }
    if (socket == mainSocketNumber) {
      addNewSocket(socket);
    } else {
      processMsgFromClient(socket);
    }
  }
}

/*
 * Initialize server
 */
void *server_init(void *arg) {
  // Initialize socket
  int mainServerSocket = 0;
  mainServerSocket = tcpServerSetup(PORT);

  // Run server control
  serverControl(mainServerSocket);

  // Clean up
  close(mainServerSocket);

  return NULL;
}


void sigint_handler() {
  printf("SIGINT caught\n");
  //vision_terminate(true);
  //arm_close();
  if (rover_close() != 0) {
    fprintf(stderr, "failed to close rover\n");
  }
  exit(0);
}

/********************************** Main **********************************/

int main(int argc, char *argv[]) {
  
    
  struct sigaction sa;
  sa.sa_handler = sigint_handler;
  sa.sa_flags = 0;
  sigemptyset(&sa.sa_mask);
  sigaddset(&sa.sa_mask, SIGALRM);
  sigaction(SIGINT, &sa, NULL);

   
  // Initialize rover
  distance = 10000;
  speed = 128;
  //vision_init();
  //arm_init(); 
  if (rover_init() != 0) {
    printf("failed to initialize rover\n");
    return -1;
  }
  else{
        printf("rover initialized successfully \n");
  }
  if (isr_init() != 0) {
    printf("failed to initialize rover\n");
    return -1;
  }
  else{
    printf("isr initialized successfully!\n");
  }
  
  //motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
  //motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
  //rover_forward(128);
  //rover_reverse(128);
  //printf("calibrating...\n");
  // wait for calibration to finish
  //while (rover_is_calibrated() == false){}
  //printf("done.\n");

  server_init(NULL);
    
  return 0;
}
