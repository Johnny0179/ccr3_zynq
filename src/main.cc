#include "robot.hpp"
#include "freemodbus_tcp.h"

#include <pthread.h>

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

    // mutex
    // pthread_mutex_t xLock = PTHREAD_MUTEX_INITIALIZER;

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
    // if (ccr3.NMTstart() != -1)
    // {
    //     printf("NMT started successfully!\n");
    // }

    // // delay 200ms
    // usleep(200000);

    // ccr3.MotorEnable(1);

    // sleep(2);

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
    int i;
    can_frame can_recv_frame;
    while (1)
    {
        ccr3.CanRecv(&can_recv_frame);

        // print can received frame
        printf("ID=0x%X DLC=%d \n", can_recv_frame.can_id, can_recv_frame.can_dlc);

        for (i = 0; i < can_recv_frame.can_dlc; i++)
        {

            printf("data[%d]=0x%X\n", i, can_recv_frame.data[i]);
            usRegHoldingBuf[i] = can_recv_frame.data[i];
        }
        printf("----------------------------------\n");
    }
}
