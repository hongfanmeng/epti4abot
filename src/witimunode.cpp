// #include "ros/ros.h"
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
  WitImuNode() : Node("wit_imu_node")
  {
    int output_hz = 100;
    timer_ms = std::chrono::milliseconds(1000 / output_hz);
    frame_id = "wit_sensor";

    port = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate = this->declare_parameter<int>("baudrate", 9600);
    ser = new DF::Serial(port, baudrate);

    RCLCPP_INFO(this->get_logger(), "Port: %s", port.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate);

    if (ser->isOpened())
    {
      RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Serial Port Fail");
      return;
    }

    pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 20);
    imu = Wit::JY61P();
    timer_ = this->create_wall_timer(timer_ms, std::bind(&WitImuNode::timer_callback, this));
  }

  ~WitImuNode()
  {
    delete ser;
  }

private:
  void timer_callback()
  {
    int length = 60;

    std::string str = ser->recv(length);
    int count = str.size();
    if (count != 0)
    {
      rclcpp::Time now = this->get_clock()->now();
      unsigned char* read_buf = new unsigned char[count];
      if (str.size() > 0)
      {
        std::string tmp;
        for (int i = 0; i < length; ++i)
        {
          read_buf[i] = str[i];
          tmp.append(to_string(int(read_buf[i])).c_str()).append(" ");
        }
      }
      imu.FetchData((char*)read_buf, count);
      delete read_buf;

      sensor_msgs::msg::Imu imu_data;

      imu_data.header.stamp = now;
      imu_data.header.frame_id = frame_id;

      imu_data.linear_acceleration.x = imu.acc.x;
      imu_data.linear_acceleration.y = imu.acc.y;
      imu_data.linear_acceleration.z = imu.acc.z;

      RCLCPP_INFO(this->get_logger(), "linear_acc, x=%f, y=%f, z=%f", imu.acc.x, imu.acc.y, imu.acc.z);

      imu_data.angular_velocity.x = imu.gyro.x;
      imu_data.angular_velocity.y = imu.gyro.y;
      imu_data.angular_velocity.z = imu.gyro.z;

      RCLCPP_INFO(this->get_logger(), "angular_vel: x=%f, y=%f, z=%f", imu.gyro.x, imu.gyro.y, imu.gyro.z);

      tf2::Quaternion curr_quater;
      curr_quater.setRPY(imu.angle.r, imu.angle.p, imu.angle.y);  // zyx

      imu_data.orientation.x = curr_quater.x();
      imu_data.orientation.y = curr_quater.y();
      imu_data.orientation.z = curr_quater.z();
      imu_data.orientation.w = curr_quater.w();

      RCLCPP_INFO(this->get_logger(), "orientation: x=%f, y=%f, z=%f, w=%f", imu_data.orientation.x,
                  imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);

      pub_imu->publish(imu_data);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  Wit::JY61P imu;
  std::string port;
  int baudrate;
  DF::Serial* ser;
  std::chrono::milliseconds timer_ms;
  std::string frame_id;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WitImuNode>());
  rclcpp::shutdown();
  return 0;
}