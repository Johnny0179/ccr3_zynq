#include "maxon.hpp"

#include <pthread.h>

int main()
{

    maxon ccr3;

    sleep(5);
    if (ccr3.NMTstart() != -1)
    {
        printf("NMT started successfully!\n");
    }

    // delay 200ms
    usleep(200000);

    ccr3.MotorEnable(1);

    sleep(2);

    ccr3.StopMotor(1);

    ccr3.NMTstop();
    return 0;
}
