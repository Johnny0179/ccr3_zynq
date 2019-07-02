#include "can.hpp"

can::can(/* args */)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    struct can_filter rfilter[1];

    // create socket
    s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    //
    strcpy(ifr.ifr_name, "can0");

    ioctl(s_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // bind can0
    bind(s_, (struct sockaddr *)&addr, sizeof(addr));

    // receive filter
    rfilter[0].can_id = 0x01;
    rfilter[0].can_mask = CAN_SFF_MASK;

    // set the socket options
    // setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

}

can::~can()
{
}

int can::send(const struct can_frame *send_frame)
{
    return write(s_, send_frame, sizeof(*send_frame));
}

int can::receive(can_frame *recv_frame)
{
    return read(s_, recv_frame, sizeof(*recv_frame));
}

void can::close_socketCAN(void)
{
    close(s_);
}
