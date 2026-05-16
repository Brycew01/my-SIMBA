
#ifndef __POLLLIB_H__
#define __POLLLIB_H__

#include <stddef.h>
#include <stdint.h>

#define POLL_SET_SIZE 10
#define POLL_WAIT_FOREVER -1

void setupPollSet();
void addToPollSet(int socketNumber);
void removeFromPollSet(int socketNumber);
int pollCall(int timeInMilliSeconds);
void *srealloc(void *ptr, size_t size);
void *sCalloc(size_t nmemb, size_t size);

#endif