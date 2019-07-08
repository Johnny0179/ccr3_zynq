#include "robot.hpp"
#include "freemodbus_tcp.h"

#include <pthread.h>

// mutex
pthread_mutex_t xLock = PTHREAD_MUTEX_INITIALIZER;

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

    // sleep(5);
    if (ccr3.NMTstart() != -1)
    {
        printf("NMT started successfully!\n");
    }

    // delay 200ms
    usleep(200000);

    /* test */
    // set profile velocity mode
    // ccr3.SdoWrU32(1, 0x6060, 0, 0x03);

    ccr3.MotorEnable(1);
    ccr3.SetMotorAbsPos(1, 1000);
    
    // delay 2ms
    usleep(2000);
    ccr3.SetCtrlWrd(1, 0x000F);

    sleep(2);

    // ccr3.StopMotor(1);

    // ccr3.NMTstop();

    while (1)
    {
        sleep(1);
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
                { // refresh rate 10ms
                    usleep(10000);
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
