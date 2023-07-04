#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <thread>
#include "imu_model.hpp"

struct imu_data_raw
{
  uint64_t accel_stamp;
  uint64_t gyro_stamp;
  int16_t accel[3];
  int16_t gyro[3];
  float temperature;
};

typedef struct imu_data_raw imu_data_raw_t;

union
{
  struct
  {
    uint8_t header[4];
    imu_data_raw_t data;
    uint64_t cam_sync_60hz_stamp;
    uint64_t cam_sync_30hz_stamp;
  } packet;
  char buffer[sizeof(imu_data_raw_t) + 4 + 8 + 8];
} udp_packet;

using boost::asio::ip::udp;
int udp_port = 5555;
std::string imu_name;

ros::NodeHandle* nh;
ros::Publisher imu_pub;
void start_server()
{
  try
  {
    boost::asio::io_service io_service;
    udp::socket socket(io_service, udp::endpoint(udp::v4(), udp_port));
    sensor_msgs::Imu imu_msg;
    imuModel imu;
    Vector3d accel(0, 0, 0);
    Vector3d gyro(0, 0, 0);
    int counter = 0;
    while (ros::ok())
    {
      boost::array<char, 128> recv_buf;
      udp::endpoint remote_endpoint;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint);
      memcpy(udp_packet.buffer, recv_buf.data(), sizeof(udp_packet.buffer));
      // Feed the raw IMU sample through the calibration map
      Vector3d accel_raw(udp_packet.packet.data.accel[0], udp_packet.packet.data.accel[1],
                         udp_packet.packet.data.accel[2]);
      Vector3d gyro_raw(udp_packet.packet.data.gyro[0], udp_packet.packet.data.gyro[1], udp_packet.packet.data.gyro[2]);
      std::tie(accel, gyro) = imu.compute(accel_raw, gyro_raw);

      imu_msg.header.stamp = ros::Time::now();
      // imu_msg.linear_acceleration.x = udp_packet.packet.data.accel[0];
      // imu_msg.linear_acceleration.y = udp_packet.packet.data.accel[1];
      // imu_msg.linear_acceleration.z = udp_packet.packet.data.accel[2];
      // imu_msg.angular_velocity.x = udp_packet.packet.data.gyro[0];
      // imu_msg.angular_velocity.y = udp_packet.packet.data.gyro[1];
      // imu_msg.angular_velocity.z = udp_packet.packet.data.gyro[2];
      imu_msg.linear_acceleration.x = accel(0);
      imu_msg.linear_acceleration.y = accel(1);
      imu_msg.linear_acceleration.z = accel(2);
      imu_msg.angular_velocity.x = gyro(0);
      imu_msg.angular_velocity.y = gyro(1);
      imu_msg.angular_velocity.z = gyro(2);
      imu_msg.orientation.x = 0;
      imu_msg.orientation.y = 0;
      imu_msg.orientation.z = 0;
      imu_msg.orientation.w = 1;
      counter++;
      if (counter > 0)
      {
        counter = 0;
        imu_pub.publish(imu_msg);
      }
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_publisher");
  nh = new ros::NodeHandle;
  imu_pub = nh->advertise<sensor_msgs::Imu>("imu_topic", 1);
  ros::Rate rate(10);  // Publish at a rate of 10 Hz
  std::thread server_thread(start_server);
  server_thread.join();  // Wait for the server thread to finish
  while (ros::ok())
  {
    rate.sleep();
  }

  return 0;
}
