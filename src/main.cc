#include "can.hpp"

#include <pthread.h>

int main()
{
    int i, nbytes;
    struct can_frame frame[1] = {{0}};
    struct can_frame recev_frame;

    can can0;

    // frame 1
    frame[0].can_id = 0x11;
    frame[0].can_dlc = 1;
    frame[0].data[0] = 'Y';

    struct can_frame *temp;
    temp = &frame[0];

    for (i = 0; i < 10; i++)
    {
        // send frame 1
        nbytes = can0.send(frame);
        printf("nbytes: %d\n", nbytes);
        if (nbytes != sizeof(frame[0]))
        {
            printf("Send Error frame[0]!\n");
            break;
        }
        else
        {
            printf("%c\n", *temp->data);
        }

        sleep(1);
    }

    while (1)
    {
        can0.receive(&recev_frame);
        printf("ID=0x%X DLC=%d \n",recev_frame.can_id,recev_frame.can_dlc);
    }

    can0.close_socketCAN();

    printf("can send test done!\n");
    return 0;
}
