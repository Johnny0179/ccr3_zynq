#include "robot.hpp"
#include "freemodbus_tcp.h"

#include <pthread.h>

// mutex
// pthread_mutex_t xLock = PTHREAD_MUTEX_INITIALIZER;

// extern variables
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

/* ----------------------- global variables ---------------------------------*/
robot ccr3(usRegHoldingBuf);

// thread function declaration
static void *pvPollingThread(void *pvParameter);
static void *CanRecvThread(void *arg);

int main()
{
    int res;

    /* threads */
    pthread_t modbus_thread;

    pthread_t can_recv_thread;

    /*---------------create modbus thread---------------*/
    // do not share data.
    res = pthread_create(&modbus_thread, NULL, pvPollingThread, NULL);
    if (res != 0)
    {
        perror("Modbus thread creation failed!\n");
        exit(EXIT_FAILURE);
    }

    /*---------------create can_recev thread---------------*/
    res = pthread_create(&can_recv_thread, NULL, CanRecvThread, NULL);
    if (res != 0)
    {
        perror("CAN receive thread creation failed!\n");
        exit(EXIT_FAILURE);
    }

 /*    sleep(1);
    if (ccr3.NMTPreOperation() != -1)
    {
        printf("NMT enter preopration!\n");
    } */
    /* 
    // delay 20ms
    usleep(20000);

    ccr3.SdoWrU8(1, 0x1603, 0x00, 0);
    

    // delay 20ms
    usleep(20000);
    
    ccr3.SdoWrU32(1, 0x1603, 0x01, 0);

    // delay 20ms
    usleep(20000); */

    //stop NMT
    // ccr3.NMTstop();

    // ccr3.MotorEnable(1);

    // // delay 50ms, delay enough time to wait the EPOS operate!
    // usleep(200000);
    // ccr3.SetMotorRelPos(1, -20000);

    // usleep(200000);
    // ccr3.SetCtrlWrd(1, 0x000F);

    while (1)
    {
        ccr3.system();
        // delay 100us
        usleep(100);
    }
    return 0;
}

// modbus polling thread
void *pvPollingThread(void *pvParameter)
{
    // init modbus tcp
    if (eMBTCPInit(MODBUS_TCP_PORT) != MB_ENOERR)
    {
        fprintf(stderr, "%s: can't initialize modbus stack!\r\n", PROG);
    }
    else
    {
        // enable protocal
        if (eMBEnable() == MB_ENOERR)
        {
            while (1)
            {
                if (eMBPoll() == MB_ENOERR)
                { // refresh rate 1ms
                    usleep(1000);
                }
                else
                {
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

// can send thread function
void *CanRecvThread(void *arg)
{

    while (1)
    {
        // delay 1ms
        usleep(1000);
        ccr3.CanDisPatch();
    }
}
