#include <ros/ros.h>
#include <string>
#include <sstream>
#include <stdio.h>      
#include <stdlib.h>     
#include <unistd.h>     
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      
#include <termios.h>    
#include <errno.h>  
#include <serial_port_driver/handle.hpp>

Handle::Handle() {
    max_buffer = 4000;
    recv_cnt = 0;
    send_cnt = 0;
    recv_valid_cnt = 0;
    freq = 0;
    read_ready = 0;
    send_ready = 0;
    upLoadSum_test = 0;
    headPointer = 0;
    tailPointer = 0;
}

Handle::~Handle() {

}

int Handle::configure_serial(char* port, int speed, int flow_ctrl, int databits, int stopbits,
                             int parity) {
    qserial.port = port;
    qserial.speed = speed;
    qserial.flow_ctrl = flow_ctrl;
    qserial.databits = databits;
    qserial.stopbits = stopbits;
    qserial.parity = parity;

    qserial.fd = qserial.openserial(qserial.port);

    if (qserial.fd > 0) {
        qserial.init(qserial.fd, qserial.speed, qserial.flow_ctrl, qserial.databits, qserial.stopbits,
                     qserial.parity);
        printf("open serial success!\n");
        ROS_INFO("open serial success");
        return 1;
    } else {
        printf("open serial failed!\n");
        ROS_INFO("open serial failed!");
        return 0;
    }
}
// reading all received data
int Handle::read_handle() {
    int i;
    long timedif;
    recv_cnt = qserial.recv(qserial.fd, recv_buf_temp, max_buffer);

    //ROS_ERROR("recv_cnt=%d",recv_cnt);
    for (i = 0; i < recv_cnt; i++) {
        circle_buf[tailPointer] = recv_buf_temp[i];

        if (tailPointer < CIRCLE_BUFF_SIZE - 1) {
            tailPointer ++;
            //if((tailPointer%10)==0) ROS_ERROR("tailPoint=%d",tailPointer);
        } else {
            tailPointer = 0;
            //ROS_ERROR("tailPointer clear!");
        }
    }
}

//storing received data in circle buffer
void Handle::preparation_handle() {

    int cnt_tmp;

    if (tailPointer >= headPointer) {
        cnt_tmp = tailPointer - headPointer;
    } else {
        //already finished one circle
        cnt_tmp = tailPointer + CIRCLE_BUFF_SIZE - headPointer;
    }

    //ROS_ERROR("preparation:cnt_tmp=%d,read_ready=%d",cnt_tmp,read_ready);
    if (read_ready == 0 && cnt_tmp >= UPLOAD_DATA_LEN) {
        int head0 = headPointer;
        int head1 = headPointer + 1;

        if (head1 >= CIRCLE_BUFF_SIZE) {
            head1 -= CIRCLE_BUFF_SIZE;    //check if one circle is finished
        }


        if (circle_buf[head0] == 0x5a && circle_buf[head1] == 0xa5) { // head, end and sum check
            for (int i = 0; i < UPLOAD_DATA_LEN; i++) { //valid data and clear circular buffer
                recv_buf[i] = circle_buf[headPointer];
                circle_buf[headPointer] = 0;
                headPointer++;

                if (headPointer >= CIRCLE_BUFF_SIZE) {
                    headPointer -= CIRCLE_BUFF_SIZE;
                }

                //printf("recv is %2x\n",recv_buf[i]);
            }

            //printf("\n");
            read_ready = 1;
            //last_time_vel = ros::Time::now();

        } else {
            //ROS_ERROR("head or end error %02x",circle_buf[headPointer]);
            //printf("%02x ",circle_buf[headPointer]);
            read_ready = 0;
            circle_buf[headPointer] = 0;
            headPointer ++;

            if (headPointer >= CIRCLE_BUFF_SIZE) {
                headPointer -= CIRCLE_BUFF_SIZE;
            }
        }

    }
}


int Handle::write_handle() {
    /*clock_gettime(CLOCK_REALTIME,&ser_time1);
    if(abs(ser_time1.tv_nsec-ser_last_time1.tv_nsec)>=20000000)
    {
        send_ready=1;
        qserial.send(qserial.fd,send_buf,sizeof(struct DownLoadUnionDef));
        printf("freq is %d\n",freq++);
        ser_last_time1.tv_nsec=ser_time1.tv_nsec;
    }*/
    qserial.send(qserial.fd, send_buf, sizeof(struct DownLoadUnionDef));
}
