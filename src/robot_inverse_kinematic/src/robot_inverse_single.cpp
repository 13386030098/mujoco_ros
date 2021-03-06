#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <stdio.h>
#include <robot_inverse.h>
#include <robot_msgs/omega.h>
#include <robot_msgs/ik.h>

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
  Eigen::Matrix<double,3,3> rotation_0_3;
  Eigen::Matrix<double,3,3> rotation_3_6;
  Eigen::Matrix<double,3,3> rotation_0_6;

  Eigen::Matrix<double,3,3> frame_end_constant;


  Eigen::VectorXd joint_values;

  double direction_pos_x;
  double direction_pos_y;
  double direction_pos_z;
  double direction_rpy_r;
  double direction_rpy_p;
  double direction_rpy_y;
  double scale_p_x;
  double scale_p_y;
  double scale_p_z;
  double scale_r_x;
  double scale_r_y;
  double scale_r_z;
  double rotate_angle;
  double roll_angle;
  double clip_angle;

  double omega_button_zero;
  double omega_button;
  double omega_button_desire;
  bool omega_button_is_open;
  bool is_first_;
public:
  teleoperation():
    is_first_(true)
  {
    direction_pos_x = -1;
    direction_pos_y = 1;
    direction_pos_z = 1;
    direction_rpy_r = 1;
    direction_rpy_p = 1;
    direction_rpy_y = 1;
    scale_p_x = 0.2;
    scale_p_y = 0.2;
    scale_p_z = 0.2;
    scale_r_x = 0.1;
    scale_r_y = 0.1;
    scale_r_z = 0.1;

    std::cout<<"teleoperation start ..."<<std::endl;
    pub = nh.advertise<robot_msgs::ik>("/ik", 100, true);
    sub = nh.subscribe("/omega_pose", 100, &teleoperation::operationCallback, this);
  }

  ~teleoperation(){}

  void operationCallback(const robot_msgs::omega::ConstPtr& omega7_msg)
  {
    if(is_first_)
    {
      for(unsigned int i=0;i<3;i++)
          master_pos_zero[i] = omega7_msg->data[i];
      for(unsigned int i=0;i<3;i++)
          master_rpy_zero[i] = omega7_msg->data[i+3];

      omega_button_zero = omega7_msg->button[0];

      kinematics_.getTransformAtIndex(10, frame_end_zero_position);
      kinematics_.getTransformAtIndex_rotation(11, frame_end_zero_rotation);
      slave_pos_zero = frame_end_zero_position.translation();
      slave_rotation_zero = frame_end_zero_rotation.rotation();

      is_first_=false;
      return;
    }
    for(unsigned int i=0;i<3;i++)
        master_pos[i] = omega7_msg->data[i];
    for(unsigned int i=0;i<3;i++)
        master_rpy[i] = omega7_msg->data[i+3];
    omega_button = omega7_msg->button[0];

    omega_button_desire = omega_button - omega_button_zero;

    slave_desire_pos[0] = direction_pos_x * (master_pos[0]-master_pos_zero[0]) * scale_p_x + slave_pos_zero[0];
    slave_desire_pos[1] = direction_pos_y * (master_pos[1]-master_pos_zero[1]) * scale_p_y + slave_pos_zero[1];
    slave_desire_pos[2] = direction_pos_z * (master_pos[2]-master_pos_zero[2]) * scale_p_y + slave_pos_zero[2];

    slave_desire_rpy_increase[0] = direction_rpy_r * scale_r_x*(master_rpy[0]-master_rpy_zero[0]);
    slave_desire_rpy_increase[1] = direction_rpy_p * scale_r_y*(master_rpy[1]-master_rpy_zero[1]);
    slave_desire_rpy_increase[2] = direction_rpy_y * scale_r_z*(master_rpy[2]-master_rpy_zero[2]);

    slave_desire_rotation = (Eigen::AngleAxisd(slave_desire_rpy_increase[2], Eigen::Vector3d::UnitZ()))*
                            (Eigen::AngleAxisd(slave_desire_rpy_increase[1], Eigen::Vector3d::UnitY()))*
                            (Eigen::AngleAxisd(slave_desire_rpy_increase[0], Eigen::Vector3d::UnitX()))
                             *slave_rotation_zero;

    frame_end = Eigen::Translation3d(slave_desire_pos);

    joint_values.resize(6);
    kinematics_.getIk(frame_end, joint_values);


    rotation_0_6 = slave_desire_rotation;

    rotation_0_3 = (Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitZ()))*//child_20
                   (Eigen::AngleAxisd(0 , Eigen::Vector3d::UnitY()))*
                   (Eigen::AngleAxisd(0 , Eigen::Vector3d::UnitX()))*
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()))*//child_21
                   (Eigen::AngleAxisd(1.0471975512, Eigen::Vector3d::UnitY()))*
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))*
                   (Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()))*//roll_21
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()))*
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))*
                   (Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()))*//roll_22
                   (Eigen::AngleAxisd(0.261799, Eigen::Vector3d::UnitY()))*
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))* (Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ()))*
                   (Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()))*//para22
                   (Eigen::AngleAxisd(0.69813170079, Eigen::Vector3d::UnitY()))*
                   (Eigen::AngleAxisd(1.57079632679, Eigen::Vector3d::UnitX()))* (Eigen::AngleAxisd(joint_values[1],Eigen::Vector3d::UnitZ()))*
                   (Eigen::AngleAxisd(2.09439510239,Eigen::Vector3d::UnitZ()))*//para23
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()))*
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))* (Eigen::AngleAxisd(-joint_values[1],Eigen::Vector3d::UnitZ()))*
                   (Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()))*//slide21
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()))*
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))* (Eigen::AngleAxisd(joint_values[1],Eigen::Vector3d::UnitZ()))*
                   (Eigen::AngleAxisd(1.57079632679,Eigen::Vector3d::UnitZ()))*//instrument2
                   (Eigen::AngleAxisd(-1.57079632679, Eigen::Vector3d::UnitY()))*
                   (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))* (Eigen::AngleAxisd(joint_values[2],Eigen::Vector3d::UnitZ()));

    rotation_3_6 = rotation_0_6.inverse() * rotation_0_3;
    double rotate_angle = std::atan2(rotation_3_6(2,1), rotation_3_6(2,0));
    rotate_angle = rotate_angle - 1.5708;
//    double roll_angle = std::atan2(rotation_3_6(1,0)*std::cos(rotate_angle) + rotation_3_6(1,1)*std::sin(rotate_angle), rotation_3_6(1,2));
    double roll_angle = std::atan2(-rotation_3_6(0,2), rotation_3_6(0,0)*std::cos(rotate_angle) + rotation_3_6(0,1)*std::sin(rotate_angle));
//    double roll_angle = std::atan2(-rotation_3_6(2,2), rotation_3_6(2,0)*std::cos(rotate_angle) + rotation_3_6(2,1)*std::sin(rotate_angle));

//    double clip_angle = std::acos(rotation_3_6(0,1)*cos(rotate_angle) - rotation_3_6(0,0)*sin(rotate_angle));
    double clip_angle = std::asin(std::cos(roll_angle)*(rotation_3_6(0,0)*std::cos(rotate_angle) + rotation_3_6(0,1)*std::sin(rotate_angle)) -
                                   rotation_3_6(0,2)*std::sin(roll_angle));

//    std::cout << clip_angle - 1.57079<<std::endl;

    joint_values[3] = rotate_angle;
    joint_values[4] = roll_angle;
//    joint_values[5] = clip_angle;

//    double rotate_angle = std::atan2(frame_end_constant(2,0), frame_end_constant(2,1));
//    double roll_angle = std::atan2((frame_end_constant(2,1) * std::cos(rotate_angle) - frame_end_constant(2,0)*std::sin(rotate_angle)), frame_end_constant(2,2));
//    double clip_angle = std::acos(frame_end_constant(0,0) * std::cos(rotate_angle) + frame_end_constant(0,1)*std::sin(rotate_angle));

    robot_msgs::ik ik_msg;
    ik_msg.data.resize(6);
    ik_msg.data[0] = joint_values[0];
    ik_msg.data[1] = joint_values[1];
    ik_msg.data[2] = joint_values[2];
//    ik_msg.data[3] = joint_values[3];
//    ik_msg.data[4] = joint_values[4];
//    ik_msg.data[5] = joint_values[5];

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































