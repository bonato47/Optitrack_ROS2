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
vrpn obj1;
vrpn obj2;

Vector4d outobj1;
Vector4d outobj2;

vector<double> row;
int main(int argc, char **argv)
{
    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "objectbase");
    ros::NodeHandle Nh_;
    ros::Subscriber sub_BF1 = Nh_.subscribe("/vrpn_client_node/franka/pose", 1000, CC_vrpn1);
    ros::Subscriber sub_BF2 = Nh_.subscribe("/vrpn_client_node/BaseFranka2/pose", 1000, CC_vrpn2);
    ros::Subscriber sub_obj1 = Nh_.subscribe("/vrpn_client_node/ball/pose", 1000, CC_vrpn3);
    ros::Subscriber sub_obj2 = Nh_.subscribe("/vrpn_client_node/ObjectPetit2/pose", 1000, CC_vrpn4);

    ros::Publisher pub1 = Nh_.advertise<geometry_msgs::PoseStamped>("/vrpn/Object1_base1", 1000);
    ros::Publisher pub2 = Nh_.advertise<geometry_msgs::PoseStamped>("/vrpn/Object1_base2", 1000);

    ros::Rate loop_rate(400);

    geometry_msgs::PoseStamped msgP1;
    geometry_msgs::PoseStamped msgP2;


    //begin the ros loop
    double count = 0;
    while (ros::ok())
    {
        Quaterniond q1,q2;
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
        MatrixXd M1(4,4) ,M2(4,4);
 
        M1 << R1(0,0),R1(0,1),R1(0,2),base1.pos[0],
              R1(1,0),R1(1,1),R1(1,2),base1.pos[1],
              R1(2,0),R1(2,1),R1(2,2),base1.pos[2],
              0,0,0,1;
        M2 << R2(0,0),R2(0,1),R2(0,2),base2.pos[0],
              R2(1,0),R2(1,1),R2(1,2),base2.pos[1],
              R2(2,0),R2(2,1),R2(2,2),base2.pos[2],
              0,0,0,1;
        Vector4d obj1temp ={obj1.pos[0],obj1.pos[1],obj1.pos[2],1};
        Vector4d obj2temp ={obj2.pos[0],obj2.pos[1],obj2.pos[2],1};

        outobj1=M1.inverse()*obj1temp;
        outobj2=M2*obj2temp; 


        msgP1.pose.position.x = outobj1[0];
        msgP1.pose.position.y = outobj1[1];
        msgP1.pose.position.z = outobj1[2];

        msgP2.pose.position.x = outobj2[0];
        msgP2.pose.position.y = outobj2[1];
        msgP2.pose.position.z = outobj2[2];
    
        pub1.publish(msgP1);
        pub2.publish(msgP2);
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
    obj1.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    obj1.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}

void CC_vrpn4(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    obj2.pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    obj2.quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
}



