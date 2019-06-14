#ifndef _handle_HPP_
#define _handle_HPP_

#include <time.h>
#include <string>
#include <ros/ros.h>
#include "qserial.hpp"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned  int u32;
typedef long long u64;
typedef signed short s16;
typedef signed char s8;
typedef signed int s32;

#define UPLOAD_DATA_LEN 29 //change for every new version!!!!!!!
#define DOWNLOAD_DATA_LEN   20
#define DLEM_DATA_LEN   10
#define CIRCLE_BUFF_SIZE 18300

class Handle {
public:
    Handle();
    virtual ~Handle();
#pragma pack(1)
    ros::Time current_time_vel, last_time_vel;
    struct UpLoadUnionDef {
        struct StructDef {
            u8 Head1;
            u8 Head2;
            u16 Len;
            u16 sum;
            u8 ID;//0xaf
            u8 id;
            u8 ID1;
            s16 lax;
            s16 lay;
            s16 laz;
            u8 ID2;
            s16 aax;
            s16 aay;
            s16 aaz;
            u8 ID3;
            s16 pitch;
            s16 roll;
            s16 yaw;
        } def;
    }* updata;

    struct DownLoadUnionDef {
        struct StructDef {
            u8 Head0;//
            u8 Head1;//
            u8 Head2;//
            u8 Seq;//
            u16 Len;
            float Wheel_speed;
            float Steering_angle;
            u8 Stop;
            u8 Sensor_switchs;
            u32 Light_switchs;
            u8 Reser0;
            u8 Sum;
            u8 End0;
            u8 End1;
        } down;
    };
    //#pragma pack(pop)
#pragma pack()
    QSerial qserial;
    int configure_serial(char* port, int speed, int flow_ctrl, int databits, int stopbits, int parity);
    int read_handle();
    int write_handle();
    u8 recv_buf[3000];
    u8 recv_buf_origin[200];
    u8 recv_buf_temp[3000];
    u8 recv_valid_cnt;
    int  recv_cnt;
    u8* send_buf;
    int  send_cnt;
    int  max_buffer;
    struct timespec ser_time;
    struct timespec ser_last_time;
    struct timespec ser_time1;
    struct timespec ser_last_time1;
    int read_ready;
    int send_ready;
    int freq;
    int upLoadSum_test;
    //循环队列
    u8 circle_buf[CIRCLE_BUFF_SIZE];
    int headPointer;
    int tailPointer;
    void preparation_handle();
};
#endif /* QSERIAL_HPP_ */
