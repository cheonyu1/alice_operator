#include <iostream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/socket.h>

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <alice_msgs/MoveCommand.h>

#include <RoboCupGameControlData.h>

using namespace std;

mutex mtx;

void UDP_Thread();
void ROS_Thread();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "alice_operator_node");
  ros::NodeHandle nh;

  thread udp_recv(UDP_Thread);
  thread ros_main(ROS_Thread);

  if(!ros::ok())
  {
    cout << "ROS is not ok. " << endl;
    exit(0);
  }

  udp_recv.join();
  ros_main.join();

  return 0;
}


void UDP_Thread()
{
  int i=0;
  while(ros::ok())
  {
    mtx.lock();
    cout << "thread loop : " << i << endl;
    mtx.unlock();
    i++;
    usleep(1000000);
  }
}

void ROS_Thread()
{
  while(ros::ok())
  {
    mtx.lock();
    cout << "ros main loop. " << endl;
    mtx.unlock();
    ros::spinOnce();
    usleep(10);
  }
}









