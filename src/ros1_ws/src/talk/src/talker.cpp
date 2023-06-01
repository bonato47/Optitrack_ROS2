#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <fstream>
#include <cmath>
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <filesystem>
#include <unistd.h>


using namespace Eigen;
using namespace std;
   

void CC_vrpn1(const geometry_msgs::PoseStamped::ConstPtr msg);
void CC_vrpn2(const geometry_msgs::PoseStamped::ConstPtr msg);
void CC_vrpn3(const geometry_msgs::PoseStamped::ConstPtr msg);
void CC_vrpn4(const geometry_msgs::PoseStamped::ConstPtr msg);

void CC_vrpn1a(const geometry_msgs::PoseStamped::ConstPtr msg);
void CC_vrpn2a(const geometry_msgs::PoseStamped::ConstPtr msg);
void CC_vrpn3a(const geometry_msgs::PoseStamped::ConstPtr msg);
void CC_vrpn4a(const geometry_msgs::PoseStamped::ConstPtr msg);



class vrpn {       // The class
  public:             // Access specifier
    Vector3d pos;
    Vector4d quat;

/*     void vrpn() {  // Method/function defined inside the class
        quat   = {0,0,0,0};     
        pos= {0,0,0};
    }  */
};

vrpn base1;
vrpn base2;
vrpn base3;
vrpn base4;
vrpn obj1;
vrpn obj2;
vrpn obj3;
vrpn obj4;

Vector4d outobj1;
Vector4d outobj2;
Vector4d outobj3;
Vector4d outobj4;

vector<double> row;
int main(int argc, char **argv)
{
    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "objectbase");
    ros::NodeHandle Nh_;
    ros::Subscriber sub_BF1 = Nh_.subscribe("/vrpn_client_node/franka_base16/pose", 1000, CC_vrpn1);
    ros::Subscriber sub_BF2 = Nh_.subscribe("/vrpn_client_node/franka_base17/pose", 1000, CC_vrpn2);
    ros::Subscriber sub_BF3 = Nh_.subscribe("/vrpn_client_node/franka_base18/pose", 1000, CC_vrpn3);
    ros::Subscriber sub_BF4 = Nh_.subscribe("/vrpn_client_node/franka_base19/pose", 1000, CC_vrpn4);
   
    ros::Subscriber sub_obj1 = Nh_.subscribe("/vrpn_client_node/ball_16/pose", 1000, CC_vrpn1a);
    ros::Subscriber sub_obj2 = Nh_.subscribe("/vrpn_client_node/ball_17/pose", 1000, CC_vrpn2a);
    ros::Subscriber sub_obj3 = Nh_.subscribe("/vrpn_client_node/ball_18/pose", 1000, CC_vrpn3a);
    ros::Subscriber sub_obj4 = Nh_.subscribe("/vrpn_client_node/ball_19/pose", 1000, CC_vrpn4a);

    ros::Publisher pub1 = Nh_.advertise<geometry_msgs::PoseStamped>("/vrpn/Object16_base16", 1000);
    ros::Publisher pub2 = Nh_.advertise<geometry_msgs::PoseStamped>("/vrpn/Object17_base17", 1000);
    ros::Publisher pub3 = Nh_.advertise<geometry_msgs::PoseStamped>("/vrpn/Object18_base18", 1000);
    ros::Publisher pub4 = Nh_.advertise<geometry_msgs::PoseStamped>("/vrpn/Object19_base19", 1000);

    ros::Rate loop_rate(400);

    geometry_msgs::PoseStamped msgP1;
    geometry_msgs::PoseStamped msgP2;
   
    geometry_msgs::PoseStamped msgP3;
    geometry_msgs::PoseStamped msgP4;

   ROS_INFO("Talker running");
    //begin the ros loop
    double count = 0;
    while (ros::ok())
    {
        Quaterniond q1,q2,q3,q4;
        q1.x() = base1.quat[0];
        q1.y() = base1.quat[1];
        q1.z() = base1.quat[2];
        q1.w() = base1.quat[3];    
        Matrix3d R1 = q1.toRotationMatrix();
        
        q2.x() = base2.quat[0];
        q2.y() = base2.quat[1];
        q2.z() = base2.quat[2];
        q2.w() = base2.quat[3];    
        Matrix3d R2 = q2.toRotationMatrix();
       
        q3.x() = base3.quat[0];
        q3.y() = base3.quat[1];
        q3.z() = base3.quat[2];
        q3.w() = base3.quat[3];    
        Matrix3d R3 = q3.toRotationMatrix();
         
        q4.x() = base4.quat[0];
        q4.y() = base4.quat[1];
        q4.z() = base4.quat[2];
        q4.w() = base4.quat[3];    
        Matrix3d R4 = q4.toRotationMatrix();
       
       
        MatrixXd M1(4,4) ,M2(4,4), M3(4,4), M4(4,4);
 
        M1 << R1(0,0),R1(0,1),R1(0,2),base1.pos[0],
              R1(1,0),R1(1,1),R1(1,2),base1.pos[1],
              R1(2,0),R1(2,1),R1(2,2),base1.pos[2],
              0,0,0,1;
        M2 << R2(0,0),R2(0,1),R2(0,2),base2.pos[0],
              R2(1,0),R2(1,1),R2(1,2),base2.pos[1],
              R2(2,0),R2(2,1),R2(2,2),base2.pos[2],
              0,0,0,1;
         M3 << R3(0,0),R3(0,1),R3(0,2),base3.pos[0],
              R3(1,0),R3(1,1),R3(1,2),base3.pos[1],
              R3(2,0),R3(2,1),R3(2,2),base3.pos[2],
              0,0,0,1;
         M4 << R4(0,0),R4(0,1),R4(0,2),base4.pos[0],
              R4(1,0),R4(1,1),R4(1,2),base4.pos[1],
              R4(2,0),R4(2,1),R4(2,2),base4.pos[2],
              0,0,0,1;
       
       
       
        Vector4d obj1temp ={obj1.pos[0],obj1.pos[1],obj1.pos[2],1};
        Vector4d obj2temp ={obj2.pos[0],obj2.pos[1],obj2.pos[2],1};
        Vector4d obj3temp ={obj3.pos[0],obj3.pos[1],obj3.pos[2],1};
        Vector4d obj4temp ={obj4.pos[0],obj4.pos[1],obj4.pos[2],1};

        outobj1=M1.inverse()*obj1temp;
        outobj2=M2.inverse()*obj2temp; 
        outobj3=M3.inverse()*obj3temp;
        outobj4=M4.inverse()*obj4temp; 


        msgP1.pose.position.x = outobj1[0];
        msgP1.pose.position.y = outobj1[1];
        msgP1.pose.position.z = outobj1[2];

        msgP2.pose.position.x = outobj2[0];
        msgP2.pose.position.y = outobj2[1];
        msgP2.pose.position.z = outobj2[2];
       
       
        msgP3.pose.position.x = outobj3[0];
        msgP3.pose.position.y = outobj3[1];
        msgP3.pose.position.z = outobj3[2];
       
       
        msgP4.pose.position.x = outobj4[0];
        msgP4.pose.position.y = outobj4[1];
        msgP4.pose.position.z = outobj4[2];
    
        pub1.publish(msgP1);
        pub2.publish(msgP2);
        pub3.publish(msgP3);
        pub4.publish(msgP4);
        ++count;
        //--------------------------------------------------------------------
        ros::spinOnce();        
        loop_rate.sleep();  
    }
}

void CC_vrpn1(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    base1.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    base1.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}

void CC_vrpn2(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    base2.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    base2.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}

void CC_vrpn3(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    base3.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    base3.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}

void CC_vrpn4(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    base4.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    base4.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}

void CC_vrpn1a(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    obj1.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    obj1.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}

void CC_vrpn2a(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    obj2.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    obj2.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}


void CC_vrpn3a(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    obj3.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    obj3.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}

void CC_vrpn4a(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    obj4.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    obj4.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}
