#include "freemodbus_tcp.h"
#include "robot.hpp"

#include <pthread.h>

// extern variables
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

/* ----------------------- global variables ---------------------------------*/
robot ccr3(usRegHoldingBuf);

// thread function declaration
static void *pvPollingThread(void *pvParameter);
static void *CanRecvThread(void *arg);

int main() {
  int res;

  /* threads */
  pthread_t modbus_thread;

  pthread_t can_recv_thread;

  /*---------------create modbus thread---------------*/
  // do not share data.
  res = pthread_create(&modbus_thread, NULL, pvPollingThread, NULL);
  if (res != 0) {
    perror("Modbus thread creation failed!\n");
    exit(EXIT_FAILURE);
  }

  /*---------------create can_recev thread---------------*/
  res = pthread_create(&can_recv_thread, NULL, CanRecvThread, NULL);
  if (res != 0) {
    perror("CAN receive thread creation failed!\n");
    exit(EXIT_FAILURE);
  }

  for (;;) {
    ccr3.system();
    // delay 1us
    usleep(1);
  }
  return 0;
}

// modbus polling thread
void *pvPollingThread(void *pvParameter) {
  // init modbus tcp
  if (eMBTCPInit(MODBUS_TCP_PORT) != MB_ENOERR) {
    fprintf(stderr, "%s: can't initialize modbus stack!\r\n", PROG);
  } else {
    // enable protocal
    if (eMBEnable() == MB_ENOERR) {
      for (;;) {
        if (eMBPoll() == MB_ENOERR) {
          // refresh rate 1ms
          usleep(1000);
        } else {
          printf("modbus poll error!");
          break;
        }
      }
    }
  }

  // disable modbus stack
  (void)eMBDisable();

  return 0;
}

// can receive thread function
void *CanRecvThread(void *arg) {
  for (;;) {
    // delay 10us
    usleep(10);
    ccr3.CanDisPatch();
  }
}
