#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <stdio.h>
#include <robot_inverse_ver1.h>
#include <robot_msgs/omega.h>
#include <robot_msgs/ik.h>

#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class teleoperation
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  HomoKinematics kinematics_;
  Eigen::Vector3d master_pos_zero;
  Eigen::Vector3d master_rpy_zero;
  Eigen::Vector3d master_pos;
  Eigen::Vector3d master_rpy;
  Eigen::Affine3d frame_end_zero_position;
  Eigen::Affine3d frame_end_zero_rotation;
  Eigen::Affine3d frame_end;

  Eigen::Vector3d slave_pos_zero;
  Eigen::Vector3d slave_desire_pos;
  Eigen::Vector3d slave_desire_rpy_increase;
  Eigen::Matrix<double,3,3> slave_rotation_zero;
  Eigen::Matrix<double,3,3> slave_desire_rotation;


  Eigen::VectorXd joint_values;

  double direction_pos_x;
  double direction_pos_y;
  double direction_pos_z;
  double scale_p_x;
  double scale_p_y;
  double scale_p_z;

  double omega_button_zero;
  double omega_button;
  double omega_button_desire;
  bool omega_button_is_open;
  bool is_first_;
  bool key1, key2;
  std::thread* keyboard_thread_;

public:
  teleoperation():
    is_first_(true),
    key1(false),
    key2(false)
  {
    direction_pos_x = -1;
    direction_pos_y = 1;
    direction_pos_z = 1;
    scale_p_x = 0.2;
    scale_p_y = 0.2;
    scale_p_z = 0.2;
    std::cout<<"teleoperation start ..."<<std::endl;
    pub = nh.advertise<robot_msgs::ik>("/ik", 100, true);
    sub = nh.subscribe("/omega_pose", 100, &teleoperation::operationCallback, this);
    keyboard_thread_ = new std::thread(boost::bind(&teleoperation::keyboard_func,this));// read keyboard input thread

  }

  ~teleoperation(){}

  void keyboard_func(void)
  {
    int keys_fd;
    struct input_event t;
    keys_fd = open ("/dev/input/event4", O_RDONLY);
    if (keys_fd <= 0){
      ROS_ERROR("can't open keyboard device!");
      exit(-1);
      }
    else std::cout <<"open keyboard device success"<<std::endl;
    while (ros::ok())
    {
      if (read (keys_fd, &t, sizeof (t)) == sizeof (t))
        {
          if(t.value ==1 && t.code == KEY_1)
          {
            key1 = true;
            key2 = false;
            std::cout << "KEY_1" << std::endl;
          }
          if(t.value ==1 && t.code == KEY_2)
          {
            key1 = false;
            key2 = true;
            std::cout << "KEY_2" << std::endl;
          }
        }
    }
  }

  void operationCallback(const robot_msgs::omega::ConstPtr& omega7_msg)
  {
    if(is_first_)
    {
      for(unsigned int i=0;i<3;i++)
          master_pos_zero[i] = omega7_msg->data[i];
      for(unsigned int i=0;i<3;i++)
          master_rpy_zero[i] = omega7_msg->data[i+3];

      omega_button_zero = omega7_msg->button[0];

      kinematics_.getTransformAtIndex(8, frame_end_zero_position);
      slave_pos_zero = frame_end_zero_position.translation();
//      std::cout << slave_pos_zero << std::endl;


      is_first_=false;
      return;
    }

    for(unsigned int i=0;i<3;i++)
        master_pos[i] = omega7_msg->data[i];
    for(unsigned int i=0;i<3;i++)
        master_rpy[i] = omega7_msg->data[i+3];
    omega_button = omega7_msg->button[0];

    omega_button_desire = omega_button - omega_button_zero;

    if(key1 == true)
    {
      master_pos_zero = master_pos;
      slave_pos_zero = slave_desire_pos;
      return;
    }

    slave_desire_pos[0] = direction_pos_x * (master_pos[0]-master_pos_zero[0]) * scale_p_x + slave_pos_zero[0];
    slave_desire_pos[1] = direction_pos_y * (master_pos[1]-master_pos_zero[1]) * scale_p_y + slave_pos_zero[1];
    slave_desire_pos[2] = direction_pos_z * (master_pos[2]-master_pos_zero[2]) * scale_p_y + slave_pos_zero[2];

    frame_end = Eigen::Translation3d(slave_desire_pos);

    joint_values.resize(6);
    kinematics_.getIk(frame_end, joint_values);

    robot_msgs::ik ik_msg;
    ik_msg.data.resize(3);
    ik_msg.data[0] = joint_values[0];
    ik_msg.data[1] = joint_values[1];
    ik_msg.data[2] = joint_values[2];

    pub.publish(ik_msg);

    std::cout << "ik_msg: " << std::endl<<ik_msg <<std::endl;

  }
  void start(void)
  {
      std::cout<<"remote operation start ..."<<std::endl;
  }
};

int main(int argc, char **argv)
{
    ros::init (argc, argv, "robot_inverse");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    teleoperation operation;
    operation.start();
    std::cout << "ok" << std::endl;
    ros::waitForShutdown();
    return 0;
}































