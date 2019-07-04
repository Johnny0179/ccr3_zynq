#include "maxon.hpp"

#include "modbus-tcp.h"

#include <pthread.h>

int main()
{

    /*     maxon ccr3;

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

    ccr3.NMTstop(); */

    modbus_t *ctx;
    uint16_t tab_reg[10];
    int rc, i;

    ctx = modbus_new_tcp("10.24.4.77", 1502);
    if (modbus_connect(ctx) == -1)
    {
        printf("Modbus_tcp connection failed!\n");
        modbus_free(ctx);
        return -1;
    }
    else
    {
        printf("Modbus_tcp connection successful!\n");
    }

    // set the slave id
    modbus_set_slave(ctx, 1);

    while (1)
    {
        // refresh rate 2s
        sleep(2);
        printf("-------------------------------\n");
        
        /* Read 10 registers from the address 0 */
        rc = modbus_read_registers(ctx, 0, 10, tab_reg);
        if (rc == -1)
        {
            printf("registers read failed!\n");
            return -1;
        }
        else
        {

            for (i = 0; i < rc; i++)
            {
                printf("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
            }
        }
    }

    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
