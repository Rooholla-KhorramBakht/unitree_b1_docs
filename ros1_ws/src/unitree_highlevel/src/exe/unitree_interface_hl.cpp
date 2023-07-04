#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

using namespace UNITREE_LEGGED_SDK;
ros::Subscriber sub_high;

ros::Publisher pub_high;
ros::Publisher imu_publisher;

class CommunicationManager
{
public:
  UDP high_udp;

  HighCmd high_cmd = { 0 };
  HighState high_state = { 0 };

public:
  CommunicationManager() : high_udp(8090, "192.168.123.220", 8082, sizeof(HighCmd), sizeof(HighState))
  {
    high_udp.InitCmdData(high_cmd);
  }

  void highUdpSend()
  {
    high_udp.SetSend(high_cmd);
    high_udp.Send();
  }

  void highUdpRecv()
  {
    high_udp.Recv();
    high_udp.GetRecv(high_state);
    unitree_legged_msgs::HighState high_state_ros;
    high_state_ros = state2rosMsg(high_state);
    pub_high.publish(high_state_ros);
    // Generate and publish ROS IMU messages
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu0";
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.linear_acceleration.x = high_state_ros.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = high_state_ros.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = high_state_ros.imu.accelerometer[2];

    imu_msg.angular_velocity.x = high_state_ros.imu.gyroscope[0];
    imu_msg.angular_velocity.y = high_state_ros.imu.gyroscope[1];
    imu_msg.angular_velocity.z = high_state_ros.imu.gyroscope[2];

    imu_msg.orientation.w = high_state_ros.imu.quaternion[0];
    imu_msg.orientation.x = high_state_ros.imu.quaternion[1];
    imu_msg.orientation.y = high_state_ros.imu.quaternion[2];
    imu_msg.orientation.z = high_state_ros.imu.quaternion[3];
    imu_publisher.publish(imu_msg);
  }
};

CommunicationManager dataManager;

long high_count = 0;

void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr& msg)
{
  dataManager.high_cmd = rosMsg2Cmd(msg);
  unitree_legged_msgs::HighState high_state_ros;
  high_state_ros = state2rosMsg(dataManager.high_state);
  // pub_high.publish(high_state_ros);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_udp");

  ros::NodeHandle nh;
  sub_high = nh.subscribe("unitree_high_cmd", 1, highCmdCallback);
  pub_high = nh.advertise<unitree_legged_msgs::HighState>("unitree_high_state", 1);
  imu_publisher = nh.advertise<sensor_msgs::Imu>("unitree_imu", 1);
  LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&CommunicationManager::highUdpSend, &dataManager));
  LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&CommunicationManager::highUdpRecv, &dataManager));
  loop_udpSend.start();
  loop_udpRecv.start();
  ros::spin();
  return 0;
}
