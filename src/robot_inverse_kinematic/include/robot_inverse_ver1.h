#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <stdio.h>

using namespace std;

class HomoKinematics
{
private:

  Eigen::Matrix<double,8,6> xyz_rpy_graber_end_;
  Eigen::Affine3d frame_remote_center_;
  Eigen::Affine3d frame_graber_end_to_remote_center;
  Eigen::Vector3d frame_graber_end_to_remote_center_translation;

  unsigned int link_num_;
  double roll2_angle;
  double link1_angle;
  double slide_length;


public:
  HomoKinematics(void)
  {
    //order: x,y,z,r,p,y
    xyz_rpy_graber_end_ <<
       0,0,1,0,0,0,//base
       0.1862,0,-0.1475,0,2.0943951,0,//roll1
       0.15587,0,0.155,0,-0.261799,0,//roll2
       0.06078,-0.029,0.25714,-1.570796,-1.745329,0,//link1
       -0.2,0,-0.071,0,0,0,//link2
       0.35,0,0.077,0,0,0.61115,//link3
       0.14795,-0.13458,0.023,0,1.570796,-0.5635668,//remote_center
       0,0.05342,0.0845,0,0,0;//tool_end
  }

  ~HomoKinematics(){}

  Eigen::Affine3d transXYZRPY(double x,double y,double z,double roll,double pitch,double yaw)
  {
    Eigen::Matrix<double,3,3> rotation;
    Eigen::Vector3d posotion;
    posotion << x, y, z;
    rotation = (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()))*(Eigen::AngleAxisd(pitch,
                Eigen::Vector3d::UnitY()))*(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    Eigen::Affine3d frame(Eigen::Translation3d(posotion) * rotation);
    return frame;
  }

  void getTransformAtIndex(int index, Eigen::Affine3d &frame)
  {
    if(index> 10 || index<1)
    {
      printf("invalid index.[1-11]");
    }
    frame=Eigen::Affine3d::Identity();
    for(int i=0;i<index;i++)
    {
      frame = frame*
          transXYZRPY(xyz_rpy_graber_end_(i,0),xyz_rpy_graber_end_(i,1),xyz_rpy_graber_end_(i,2),
                      xyz_rpy_graber_end_(i,3),xyz_rpy_graber_end_(i,4),xyz_rpy_graber_end_(i,5));
    }
  }

  void getIk(Eigen::Affine3d frame_end, Eigen::VectorXd& joint_values)
  {
    joint_values.resize(6);
    getTransformAtIndex(7, frame_remote_center_);
    frame_graber_end_to_remote_center = frame_remote_center_.inverse()*frame_end;
    frame_graber_end_to_remote_center_translation = frame_graber_end_to_remote_center.translation();
    slide_length = frame_graber_end_to_remote_center_translation.norm();
    roll2_angle = std::atan2(frame_graber_end_to_remote_center_translation(0), frame_graber_end_to_remote_center_translation(1));
    link1_angle = std::asin(frame_graber_end_to_remote_center_translation(2)/slide_length);
    joint_values[0] = roll2_angle;
    joint_values[1] = link1_angle - 1.00704;
    joint_values[2] = slide_length;
  }
};





















