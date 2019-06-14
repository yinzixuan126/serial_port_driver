#include <stdio.h>      
#include <stdlib.h>     
#include <unistd.h>
#include <sstream>
#include <math.h>
#include <vector>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp> 

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <serial_port_driver/handle.hpp>

QSerial qserial;
Handle handle;
struct Handle::DownLoadUnionDef downloaddata;

int speed = 115200, databits = 8, stopbits = 1, parity = 'n', flow_ctrl = 0, ret = 0;

void do_pre() {
    while (1) {
        handle.preparation_handle();
        usleep(1000);
    }
}

void do_stuff() {
    while (1) {
        handle.read_handle();
        usleep(1000);
    }
}

void bitSet(unsigned int& data, int position, int flag) {
    if (flag) {
        data |= (1 << (position - 1));
    } else {
        data &= ~(1 << (position - 1));
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "serial_driver");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    ros::start();
    ros::Rate r(100.0);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("IMU", 50);
    //get parameters
    std::string serial_port1;
    pn.param<std::string>("serial_port1", serial_port1, "/dev/ttyUSB0");
    char* serial_port = ((char*)serial_port1.c_str());

    //configuring serial port parameters
    ret = handle.configure_serial(serial_port, speed, flow_ctrl, databits, stopbits, parity);

    if (!ret) {
        return 0;
    }

    // bring up another thread
    boost::thread thread_b(do_stuff);
    boost::thread thread_c(do_pre);
    // storing receving buffer
    handle.updata = (Handle::UpLoadUnionDef*)handle.recv_buf;
    //ROS_ERROR("data size is, %d", sizeof(struct Handle::UpLoadUnionDef));

    while (ros::ok()) {
        if (handle.read_ready == 1) {
            //ROS_ERROR("Read ready!!!!");
            handle.read_ready = 0;//handle.read_ready denotes if the data is prepareed well.

            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "base_link";
            imu_data.orientation.x = handle.updata->def.pitch / 100.0 * 3.14 / 180.0;
            imu_data.orientation.y = handle.updata->def.roll / 100.0 * 3.14 / 180.0;
            imu_data.orientation.z = handle.updata->def.yaw / 10.0 * 3.14 / 180.0;
            imu_data.orientation.w = 0.0;

            imu_data.linear_acceleration.x = handle.updata->def.lax / 1000.0 * 9.8;
            imu_data.linear_acceleration.y = handle.updata->def.lay / 1000.0 * 9.8;
            imu_data.linear_acceleration.z = handle.updata->def.laz / 1000.0 * 9.8;

            imu_data.angular_velocity.x = handle.updata->def.aax / 10.0;
            imu_data.angular_velocity.y = handle.updata->def.aay / 10.0;
            imu_data.angular_velocity.z = handle.updata->def.aaz / 10.0;

            imu_pub.publish(imu_data);

            // downloaddata.down.Seq = Seq;
            // //get download_data sum
            // download_data_temp = (u8*)&downloaddata;
            // download_sum_total = 0;
            // for (int i = 3; i < sizeof(struct Handle::DownLoadUnionDef) - 3; i++) {
            //     download_sum_total ^= *(download_data_temp + i);
            // }
            // download_sum_total = download_sum_total & 0xff;
            // downloaddata.down.Sum = download_sum_total;

            // **************print download data******************
            // //u8* testdata=(u8*)&downloaddata;
            // //for(int j=0;j<sizeof( struct Handle::DownLoadUnionDef);j++)
            // //{printf("%02x ",testdata[j]);}
            // //printf("\n");
            // //ROS_INFO("DOWNLAOD ZIJIESHU IS %d",sizeof( struct Handle::DownLoadUnionDef));
            // // writing all commands
            // handle.send_buf = (u8*)(&downloaddata);
            // handle.write_handle();
        }

        ros::spinOnce();
        r.sleep();
    }
}
