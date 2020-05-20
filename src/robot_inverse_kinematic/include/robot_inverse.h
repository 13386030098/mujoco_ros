#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <stdio.h>

using namespace std;

class HomoKinematics
{
private:
  Eigen::Matrix<double,11,6> xyz_rpy_;
  Eigen::Matrix<double,10,6> xyz_rpy_graber_end_;
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
    xyz_rpy_ <<
        0,0,2,0,0,3.141592653589793,//child_20
        -0.055,0,-0.36518,0,1.0471975512,0,//child_21
        0,0,-0.15236273603,0,0,0,//roll_21
        0.1619,0,-0.11296,0,0.261799,0,//roll_22
        0.06078,-0.0025,-0.27804,1.57079632679,0.69813170079,0,//para22
        -0.2,0,-0.0415,0,0,2.09439510239,//para23
        -0.350,0,0.0305,0,0,0,//slide21
        -0.1,0.08,0.0085,0,-1.57079632679,1.57079632679,//instrument2
        0,0,0.3759,0,0,0,//rotate2
        0,0,0,3.1415,0,1.570796,//roll2
        0,0.0003,-0.0089,-1.570796,0,0;//clip21

    xyz_rpy_graber_end_ <<
        0,0,2,0,0,3.141592653589793,//child_20
        -0.055,0,-0.36518,0,1.0471975512,0,//child_21
        0,0,-0.15236273603,0,0,0,//roll_21
        0.1619,0,-0.11296,0,0.261799,0,//roll_22
        0.06078,-0.0025,-0.27804,1.57079632679,0.69813170079,0,//para22
        -0.2,0,-0.0415,0,0,2.09439510239,//para23
        -0.350,0,0.0305,0,0,0,//slide21
        -0.1,0.08,0.0085,0,-1.57079632679,1.57079632679,//instrument2
        0,0,0.273205080757,1.3962634016,0,-3.1415,//remote_center
        0,0.0921730028586,0.016252587969,0,0,0;//graber_end

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

  void getTransformAtIndex_rotation(int index, Eigen::Affine3d &frame)
  {
    if(index> 11 || index<1)
    {
      printf("invalid index.[1-11]");
    }
    frame=Eigen::Affine3d::Identity();
    for(int i=8;i<index;i++)
    {
      frame = frame*
          transXYZRPY(xyz_rpy_(i,0),xyz_rpy_(i,1),xyz_rpy_(i,2),
                      xyz_rpy_(i,3),xyz_rpy_(i,4),xyz_rpy_(i,5));
    }
  }

  void getIk(Eigen::Affine3d frame_end, Eigen::VectorXd& joint_values)
  {
    joint_values.resize(6);
    getTransformAtIndex(9, frame_remote_center_);
    frame_graber_end_to_remote_center = frame_remote_center_.inverse()*frame_end;
    frame_graber_end_to_remote_center_translation = frame_graber_end_to_remote_center.translation();
    slide_length = frame_graber_end_to_remote_center_translation.norm();
    roll2_angle = std::atan2(frame_graber_end_to_remote_center_translation(0), frame_graber_end_to_remote_center_translation(1));
    link1_angle = std::asin(frame_graber_end_to_remote_center_translation(2)/slide_length);
    joint_values[0] = roll2_angle;
    joint_values[1] = link1_angle - 0.174533;
    joint_values[2] = slide_length - 0.0935949;
  }
};





















