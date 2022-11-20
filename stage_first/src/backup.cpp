#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/Float64.h"

#include "main.hpp"


double joint_positions[6] = {0};
bool get_states = false;
ros::Time current_time;
ros::Time initial_time;
bool get_initial_time = false;
void ChatterCallBack_JointStates(const sensor_msgs::JointState::ConstPtr& msg)
{
//  std::cout << "arm callback" << std::endl;
  for (int i = 0; i < 6;i++) {
    joint_positions[i] = msg->position[i];
  }

  std::cout << "current joint positions: " << std::endl;
  for (int i = 0; i < 6;i++) {
    std::cout << joint_positions[i]*180/3.1415 << std::endl;
  }
  get_states = true;
}


int main(int argc, char** argv)
{
  remove(InterPolationData);
  std::cout << "enter main function" << std::endl;
  ros::init(argc, argv, "main");
  ros::NodeHandle masterNode;

  ros::Publisher J1_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j1_position_controller/command", 1);
  ros::Publisher J2_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j2_position_controller/command", 1);
  ros::Publisher J3_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j3_position_controller/command", 1);
  ros::Publisher J4_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j4_position_controller/command", 1);
  ros::Publisher J5_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j5_position_controller/command", 1);
  ros::Publisher J6_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j6_position_controller/command", 1);

  std::cout << "publisher defined" << std::endl;
  ros::Subscriber joint_states_sub = masterNode.subscribe("/rrbot/joint_states",1,ChatterCallBack_JointStates);
  ros::Rate loop_rate(25);
  std::cout << "subscriber defined" << std::endl;

  std_msgs::Float64 joint_cmd1;
  std_msgs::Float64 joint_cmd2;
  std_msgs::Float64 joint_cmd3;
  std_msgs::Float64 joint_cmd4;
  std_msgs::Float64 joint_cmd5;
  std_msgs::Float64 joint_cmd6;
  std::vector<double> joint_cmd(6, 0.0);

  KDL::Tree my_tree;
//  KDL::TreeFkSolverPosFull_recursive fk_solver_();
  if (!kdl_parser::treeFromFile("/home/u16/roscode/manipulator20/src/ros_robotics/urdf/arm_er20.urdf", my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
     }
  int tree_nj = my_tree.getNrOfJoints();
  int tree_ns = my_tree.getNrOfSegments();
  std::cout << "number of tree joints: " << tree_nj << std::endl;
  std::cout << "number of tree segments: " << tree_ns << std::endl;

//kdl tree to kdl chain
  KDL::Chain my_chain;
  bool exit_value;
  exit_value = my_tree.getChain("base_link", "link6", my_chain);
  int chain_nj = my_chain.getNrOfJoints();
  int chain_ns = my_chain.getNrOfSegments();
  std::cout << "exit_value: " << exit_value << std::endl;
  std::cout << "number of chain joints: " << chain_nj << std::endl;
  std::cout << "number of chain segments: " << chain_ns << std::endl;

  // define kinematic solvers
  using namespace KDL;
  int n = my_chain.getNrOfJoints();
  ChainFkSolverPos_recursive fwdkin(my_chain);
  KDL::ChainJntToJacSolver JcbSolver(my_chain);
  ChainIkSolverPos_LMA iksolver(my_chain);
  JntArray q_init(n);
  for (int i = 0; i < chain_nj; i++)
  {
      q_init.data[i] = 0.0;
      std::cout<< "init joint: " << q_init.data[i] << std::endl;
  }

  KDL::Frame init_pos;
  fwdkin.JntToCart(q_init, init_pos);
//  std::cout << "current frame: " << init_pos << std::endl;
  KDL::Frame F2(init_pos);


/**
 * read the raw angle from teaching
 * Interpolate the raw angle
 */
  ReadRawAng();
  Interpolation(angles);

/// remove the weird angles (last 10)
//  cout<<"size_Int_o="<<Inter_angles[0].size()<<endl;
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j<10;j++) {
      Inter_angles[i].pop_back();
    }
  }

//  cout<<"size_Int_m="<<Inter_angles[0].size()<<endl;


/**
 * output the readed joint angles
 */
//  std::vector<double>::iterator Iter;
//  for (int i = 0; i < 7 ; i++) {
//    angles[i].pop_back();
//    for (Iter = angles[i].begin(); Iter!=angles[i].end(); Iter++) {
//      std::cout << *Iter << "  " ;
//    }
//    std::cout<<std::endl;
//  }

  /**
   * Create trajectory
   * Ref: https://www.guyuehome.com/26274
   */
  KDL::Path_RoundedComposite* path1 = new KDL::Path_RoundedComposite(0.1, 0.01, new KDL::RotationalInterpolation_SingleAxis());
  path1->Add(init_pos);
  path1->Add(F2);
  path1->Finish();

  KDL::VelocityProfile* velprof1 = new KDL::VelocityProfile_Trap(0.5, 0.1);
//  velprof1->SetProfile(0, path1->PathLength());
  velprof1->SetProfileDuration(0,path1->PathLength(), 5);
  KDL::Trajectory* traject1 = new KDL::Trajectory_Segment(path1, velprof1);

  KDL::Trajectory_Composite* ctraject = new KDL::Trajectory_Composite();
  ctraject->Add(traject1);

  std::cout << "path without velocity generated" << std::endl;


  // path planning
  /*
   * viapoint1, ..., n
    */


//  // double t = 0, dt = 0.008;
//  int n = 100;
  double delta = 0.05;
//  ros::Duration forward_time;
//  forward_time.sec = 0;
//  forward_time.nsec = 30000000;
//  // for (t = 0;t < path.Duration();t+=dt)

  for (int i = 0; i < Inter_angles[0].size(); i++) {

//    joint_cmd1.data = angles[0][0];
//    joint_cmd2.data = angles[1][0];
//    joint_cmd3.data = angles[2][0];
//    joint_cmd4.data = angles[3][0];
//    joint_cmd5.data = angles[4][0];
//    joint_cmd6.data = angles[5][0];

    joint_cmd1.data = Inter_angles[0][i];
    joint_cmd2.data = Inter_angles[1][i];
    joint_cmd3.data = Inter_angles[2][i];
    joint_cmd4.data = Inter_angles[3][i];
    joint_cmd5.data = Inter_angles[4][i];
    joint_cmd6.data = Inter_angles[5][i];

    J1_cmd_pub.publish(joint_cmd1);
    J2_cmd_pub.publish(joint_cmd2);
    J3_cmd_pub.publish(joint_cmd3);
    J4_cmd_pub.publish(joint_cmd4);
    J5_cmd_pub.publish(joint_cmd5);
    J6_cmd_pub.publish(joint_cmd6);

    ros::spinOnce();
    loop_rate.sleep();
  }

////  for (int i = 0; i < angles[0].size(); i++) {
////    // target pose of current loop
////    /* path-> get_pose(t)*/
////    // kdl inverse
////    /*get joint positions */
////    std::cout << "i: " << i << std::endl;
////    joint_group.data.clear();
////    joint_group.data.resize(6);
////    for (int j = 0;j < 6;j++)
////    {
////      joint_group.data[j] = delta * i;
////    }
//////    joint_group_publisher.publish(joint_group);


////    target_states.points.clear();
////    trajectory_msgs::JointTrajectoryPoint target_position;
//////    std::cout << "time from start: " << target_position.time_from_start << std::endl;
////    target_position.positions.clear();
////    target_position.positions.resize(6);
////    target_position.velocities.clear();
////    target_position.velocities.resize(6);
////    target_position.accelerations.clear();
////    target_position.accelerations.resize(6);

////    for (int j = 0; j < 6;j++)
////    {
////      joint_cmd[j] = i * delta;
////    }

////    joint_cmd1.data = joint_cmd[0];
////    joint_cmd2.data = joint_cmd[1];
////    joint_cmd3.data = joint_cmd[2];
////    joint_cmd4.data = joint_cmd[3];
////    joint_cmd5.data = joint_cmd[4];
////    joint_cmd6.data = joint_cmd[5];
//      joint_cmd1.data = angles[0][i];
//      joint_cmd2.data = angles[1][i];
//      joint_cmd3.data = angles[2][i];
//      joint_cmd4.data = angles[3][i];
//      joint_cmd5.data = angles[4][i];
//      joint_cmd6.data = angles[5][i];


//    J1_cmd_pub.publish(joint_cmd1);
//    J2_cmd_pub.publish(joint_cmd2);
//    J3_cmd_pub.publish(joint_cmd3);
//    J4_cmd_pub.publish(joint_cmd4);
//    J5_cmd_pub.publish(joint_cmd5);
//    J6_cmd_pub.publish(joint_cmd6);
////    position_publisher.publish(target_states);

//    ros::spinOnce();
//    loop_rate.sleep();
//  }
  std::cout << "finished" << std::endl;
  return 0;
}

void ReadRawAng()
{
  //  read joint angles();
    std::ifstream file( "/home/u16/roscode/manipulator20/src/stage_first/src/jointangles.txt", std::ios::in );

    if( !file )
        std::cerr << "Cannot open " << std::endl;

  std::string jname1, jname2, jname3, jname4, jname5, jname6, jname_aux;
  double jang1, jang2, jang3, jang4, jang5, jang6, jang_aux;

  while( !file.eof() )
  {
      file >> jname1 >> jang1 >> jname2 >> jang2 >> jname3 >> jang3 >> jname4 >> jang4 >> jname5 >> jang5 >> jname6 >> jang6 >> jname_aux >> jang_aux;
      angles[0].push_back(jang1/180*PI);      angles[1].push_back(jang2/180*PI);      angles[2].push_back(jang3/180*PI);      angles[3].push_back(jang4/180*PI);
      angles[4].push_back(jang5/180*PI);      angles[5].push_back(jang6/180*PI);      angles[6].push_back(jang_aux/180*PI);
//      std::cout << jname_aux << ": " << jang_aux << std::endl;
  }
  file.close();
//  cout<<"read finished!"<<endl;

}

void Interpolation(vector<vector<double> > angles)
{

  /**
   * Interpolation
   */

  //  std::vector<std::vector<double> > Inter_angles(7);
    std::vector<double> xx,x;
    int size_0 = angles[0].size();
    cout<<"size_0 defined!"<<endl;
    cout<<"angles end:"<<angles[0][size_0-1]*180/PI<<std::endl;
    cout<<"angles end:"<<angles[0][size_0-2]*180/PI<<std::endl;

    double step = 0.1;
    for (int i = 0; i < size_0; i++) {
      x.push_back(i);
    }
    double value = x[0];
    int num = size_0 / step;
    for (double i = 0; i <= num; i++) {
      xx.push_back(value);
      value = value + step;
    }
    cout<<"xx defined!"<<endl;
    int size_xx = xx.size();

    vector<double> dx,dy;
    for (int i = 0; i < size_0 - 1; i++) {
      double temp_dx = x[i + 1] - x[i];
      dx.push_back(temp_dx);
    }
    MatrixXd H = MatrixXd::Random(size_0, size_0);
    for (int i = 0; i < size_0; i++) {
      for (int j = 0; j < size_0; j++) {
        H(i, j) = 0;
      }
    }
    VectorXd Y(size_0);
    for (int i = 0; i < size_0; i++) {
      Y(i) = 0;
    }
    VectorXd M(size_0);
    for (int i = 0; i < size_0; i++) {
      M(i) = 0;
    }

    H(0, 0) = 1;
    H(size_0 - 1, size_0 - 1) = 1;
    for (int i = 1; i < size_0 - 1; i++) {
        H(i, i - 1) = dx[i - 1];
        H(i, i) = 2 * (dx[i - 1] + dx[i]);
        H(i, i + 1) = dx[i];
      }
    /// ************** ///
    for (int k_ = 0; k_ < 7; k_++) {
      for (int i = 0; i < size_0 - 1; i++) {
        double temp_dy = angles[k_][i+1] - angles[k_][i];
        dy.push_back(temp_dy);
      }
      for (int i = 0; i < size_0 - 1; i++) {
        Y(i) = 3 * (dy[i] / dx[i] - dy[i - 1] / dx[i - 1]);
      }
      M = H.colPivHouseholderQr().solve(Y);
      vector<double> ai, bi, ci, di;
      for (int i = 0; i < size_0 - 1; i++) {
        ai.push_back(angles[k_][i]);
        di.push_back((M(i + 1) - M(i)) / (3 * dx[i]));
        bi.push_back(dy[i] / dx[i] - dx[i] * (2 * M(i) + M(i + 1)) / 3);
        ci.push_back(M(i));
      }
      vector<int> x_, xx_;
      for (int i = 0; i < size_0; i++) {
        int temp = x[i] / 0.1;
        x_.push_back(temp);
      }
      for (int i = 0; i < size_xx; i++) {
        int temp = xx[i] / 0.1;
        xx_.push_back(temp);
      }
  //    cout<<"M defined!"<<endl;


      for (int i = 0; i < size_xx; i++) {
        int k = 0;
        for (int j = 0; j < size_0 - 1; j++) {
          if (xx_[i] >= x_[j] && xx_[i] < x_[j + 1]) {
            k = j;
            break;
          }
          else if (xx[i] == x[size_0 - 1]) {
            k = size_0 - 1;
          }
        }
        //yy(i) = y[i] + bi(k) * (xx[i] - x[k]) + 1 / 2.0 * M(i) * pow((xx[i] - x[k]) , 2) + di(k) * pow((xx[i] - x[k]),3);
        double temp = ai[k] + bi[k] * (xx[i] - x[k]) + M(k) * pow((xx[i] - x[k]), 2) + di[k] * pow((xx[i] - x[k]), 3);
        Inter_angles[k_].push_back(temp);
      }

      cout<<"start write!"<<endl;

      std::ofstream output;
        output.open(InterPolationData, ios::app);
        for (unsigned i = 0; i < size_xx; i++) {
          output << xx[i] << '\t' << Inter_angles[k_][i]*180/PI << std::endl;
        }
      output.close();
      ai.clear(); bi.clear(); ci.clear(); di.clear(); x_.clear(); xx_.clear(); dy.clear();
    }
}
