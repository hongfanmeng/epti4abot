//#include "ros/ros.h"
#include <chrono>
#include <math.h>
// #include <serial/serial.h> //ROS已经内置了的串口包
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
// #include "epti4abot/dfrobot.hpp"
#include "epti4abot/serial.hpp"
// #include "epti4abot/witimunode.hpp"

// #include "JY901.h"
#include "epti4abot/JY61P.h"

using namespace std::chrono_literals;

class WitImuNode : public rclcpp::Node
{

public:
    WitImuNode()
    : Node("wit_imu_node") , ser(DF::Serial("/dev/ttyUSB0", 9600))
    {
        
        int output_hz = 100;
        timer_ms = std::chrono::milliseconds(1000 / output_hz);
        imu_topic = "WitSensor";
        // port = "/dev/ttyUSB0";
        // baudrate = 115200;
        // try
        // {
            // ser.setPort(port);
            // ser.setBaudrate(baudrate);
            // ser = (DF::Serial(port, baudrate));

            // serial::Timeout to = serial::Timeout::simpleTimeout(500);
            // ser.setTimeout(to);
            // ser.open();
        // }
        // catch (serial::IOException &e)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Unable to open port ");
        //     return;
        // }

        if (ser.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Serial Port Fail");
            return;
        }

        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 20);
        pub_imu_offline = this->create_publisher<sensor_msgs::msg::Imu>("/imu_offline", 20);
        //imu = CJY901();
        imu = Wit::JY61P();
        // ser.flush();
        // int size;
        timer_ = this->create_wall_timer(
            timer_ms, std::bind(&WitImuNode::timer_callback, this));
    }

private: 
    void timer_callback()
    {
        int length = 60;
        
        std::string str = ser.recv(length);
        // if (str.size() > 0) 
        // {
        //     std::string tmp;
        //     for (int i = 0 ; i < 100 ; ++i) 
        //     {   
        //         tmp.append(to_string(int(str[i])).c_str()).append(" ");
        //     }
        //     RCLCPP_INFO(this->get_logger(), tmp.c_str());
        // }
        // else 
        // {
        //         RCLCPP_INFO(this->get_logger(), "is 0");
        // }
        // RCLCPP_INFO(this->get_logger(), to_string(str.size()).c_str());
        int count = str.size();
        //int count = ser.available();
        if (count != 0)
        {
            //ROS_INFO_ONCE("Data received from serial port.");
            rclcpp::Time now = this->get_clock()->now();
            unsigned char read_buf[count];
            if (str.size() > 0) 
            {
                std::string tmp;
                for (int i = 0 ; i < length ; ++i) 
                {   
                    read_buf[i] = str[i];
                    tmp.append(to_string(int(read_buf[i])).c_str()).append(" ");
                }
                // RCLCPP_INFO(this->get_logger(), tmp.c_str());
            }
            imu.FetchData((char *)read_buf, count);
            
            sensor_msgs::msg::Imu imu_data;
            sensor_msgs::msg::Imu imu_offline_data;
            //==============imu data===============
            imu_data.header.stamp = now;
            imu_data.header.frame_id = imu_topic;

            imu_data.linear_acceleration.x = imu.acc.x;
            imu_data.linear_acceleration.y = imu.acc.y;
            imu_data.linear_acceleration.z = imu.acc.z;
            
            imu_data.angular_velocity.x = imu.angle.r * 180.0 / M_PI;
            imu_data.angular_velocity.y = imu.angle.p * 180.0 / M_PI;
            imu_data.angular_velocity.z = imu.angle.y * 180.0 / M_PI;

            tf2::Quaternion curr_quater;
            curr_quater.setRPY(imu.angle.r, imu.angle.p, imu.angle.y); //zyx

            // RCLCPP_INFO(this->get_logger(), "angle: x=%f, y=%f, z=%f",
            //   imu.angle.r, imu.angle.p, imu.angle.y);

            imu_data.orientation.x = curr_quater.x();
            imu_data.orientation.y = curr_quater.y();
            imu_data.orientation.z = curr_quater.z();
            imu_data.orientation.w = curr_quater.w();
            // RCLCPP_INFO(this->get_logger(), "Quaternion: x=%f, y=%f, z=%f, w=%f", 
            //   imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);

            //==============imu offline data===============
            imu_offline_data.header.stamp = now;
            imu_offline_data.header.frame_id = imu_topic;

            imu_offline_data.linear_acceleration.x = imu.acc.x;
            imu_offline_data.linear_acceleration.y = imu.acc.y;
            imu_offline_data.linear_acceleration.z = imu.acc.z;
            
            imu_offline_data.angular_velocity.x = imu.gyro.x;
            imu_offline_data.angular_velocity.y = imu.gyro.y;
            imu_offline_data.angular_velocity.z = imu.gyro.z;

            imu_offline_data.orientation.x = curr_quater.x();
            imu_offline_data.orientation.y = curr_quater.y();
            imu_offline_data.orientation.z = curr_quater.z();
            imu_offline_data.orientation.w = curr_quater.w();

            pub_imu->publish(imu_data);
            pub_imu_offline->publish(imu_offline_data);
            // std::string tmp;
            RCLCPP_INFO(this->get_logger(), to_string(imu.acc.z).c_str());
            // for (int i = 0 ; i < 6 ; i++) 
            // {
            //     tmp.append(imu.dirty[i]?"true":"false").append(" ");
            // }
            // RCLCPP_INFO(this->get_logger(), tmp.c_str());
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu_offline;
    Wit::JY61P imu;
    std::string port;
    int baudrate;
    //serial::Serial ser;
    DF::Serial ser;
    std::chrono::milliseconds timer_ms;
    std::string imu_topic;
    std::string imu_offline_topic;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WitImuNode>());
    rclcpp::shutdown();
    return 0;
}