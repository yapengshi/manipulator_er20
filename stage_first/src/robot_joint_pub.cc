#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
// #include "std_msgs/Header.h"
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"state_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states",1000);
    sensor_msgs::JointState joint_state_pub;
    ros::Rate rate(20);
    while (ros::ok())
    {
        
        for (double i = 0; i <= 628 ; i++)
        {   
            joint_state_pub.header.stamp = ros::Time::now();
            joint_state_pub.header.frame_id = "robot01";

            joint_state_pub.name.resize(6);
            joint_state_pub.position.resize(6);
            //不知道为什么这种方式不可以
		    // jonit_state_pub.name = {    "left_trunk12left_joint1", 
		    //                             "left_trunk22left_joint2", 
		    //                             "left_joint32left_foot1", 
		    //                             "right_trunk12right_joint1",
		    //                             "right_trunk22right_joint2",
		    //                             "right_joint32right_foot1"      };
		 
            joint_state_pub.name[0] = "joint1";// name是xacro文件中定义的关节(joint)
            joint_state_pub.position[0] = ( i / 100 );	//控制运动
            joint_state_pub.name[1] = "joint2";
            joint_state_pub.position[1] = ( i /100 );
            joint_state_pub.name[2] = "joint3";
            joint_state_pub.position[2] = 0;
            joint_state_pub.name[3] = "joint4";
            joint_state_pub.position[3] = 0;
            joint_state_pub.name[4] = "joint5";
            joint_state_pub.position[4] = 0;
            joint_state_pub.name[5] = "joint6";
            joint_state_pub.position[5] = 0;

            pub.publish(joint_state_pub);
            ROS_INFO(" i = %.2f",i/100);
            rate.sleep(); 
        }
        ros::spinOnce();   
    }
    return 0;
}
