#ifndef _QSERIAL_HPP_
#define _QSERIAL_HPP_
typedef unsigned char u8;
typedef unsigned short u16;

class QSerial  {
public:
    QSerial();
    virtual ~QSerial();
    int openserial(char* port);
    bool set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity);
    bool init(int fd, int speed, int flow_ctrl , int databits, int stopbits, int parity);
    int recv(int fd, u8* rcv_buf, int data_len);
    int send(int fd, u8* send_buf, int data_len);
    bool close(int fd);

    int fd;
    char* port;
    int speed, flow_ctrl, databits, stopbits, parity;
    u8* rcv_buf;
    int data_len;
    u8* send_buf;
};

#endif /* QSERIAL_HPP_ */
