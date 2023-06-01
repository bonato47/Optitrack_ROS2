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
   

class vrpn {       // The class
    public:             // Access specifier
    Vector3d pos, obj;
    Vector4d quat;
    geometry_msgs::PoseStamped msgP;

    void CC_vrpn_base(const geometry_msgs::PoseStamped::ConstPtr msg) {  // Method/function defined inside the class
        pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
        quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
    } 
    void CC_vrpn_obj(const geometry_msgs::PoseStamped::ConstPtr msg) {  // Method/function defined inside the class
        obj    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    } 
    void transform_new_base() {  // Method/function defined inside the class
        // trasform the quaternion to rotation matrix
        Quaterniond q;
        q.x() = quat[0];
        q.y() = quat[1];
        q.z() = quat[2];
        q.w() = quat[3];    
        Matrix3d R = q.toRotationMatrix();

        // add one line of 0,0,0,1 at the matrix for the transform
        MatrixXd M(4,4);
        M << R(0,0),R(0,1),R(0,2),pos[0],
              R(1,0),R(1,1),R(1,2),pos[1],
              R(2,0),R(2,1),R(2,2),pos[2],
              0,0,0,1;

        // Inverse * object => to have the position relatively of the base oof the robot
        Vector4d objtemp ={obj[0],obj[1],obj[2],1};
        Vector4d outobj;
        outobj=M.inverse()*objtemp;

        msgP.pose.position.x = outobj[0];
        msgP.pose.position.y = outobj[1];
        msgP.pose.position.z = outobj[2];
    } 

};

vrpn object1, object2, object3, object4;

int main(int argc, char **argv)
{
    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "objectbase");
    ros::NodeHandle Nh_;
    ros::Subscriber sub_BF1 = Nh_.subscribe("/vrpn_client_node/franka_base16/pose", 1000, &vrpn::CC_vrpn_base, &object1);
    ros::Subscriber sub_BF2 = Nh_.subscribe("/vrpn_client_node/franka_base17/pose", 1000, &vrpn::CC_vrpn_base, &object2);
    ros::Subscriber sub_BF3 = Nh_.subscribe("/vrpn_client_node/franka_base18/pose", 1000, &vrpn::CC_vrpn_base, &object3);
    ros::Subscriber sub_BF4 = Nh_.subscribe("/vrpn_client_node/franka_base19/pose", 1000, &vrpn::CC_vrpn_base, &object4);
   
    ros::Subscriber sub_obj1 = Nh_.subscribe("/vrpn_client_node/ball_16/pose", 1000, &vrpn::CC_vrpn_obj, &object1);
    ros::Subscriber sub_obj2 = Nh_.subscribe("/vrpn_client_node/ball_17/pose", 1000, &vrpn::CC_vrpn_obj, &object2);
    ros::Subscriber sub_obj3 = Nh_.subscribe("/vrpn_client_node/ball_18/pose", 1000, &vrpn::CC_vrpn_obj, &object3);
    ros::Subscriber sub_obj4 = Nh_.subscribe("/vrpn_client_node/ball_19/pose", 1000, &vrpn::CC_vrpn_obj, &object4);

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
        object1.transform_new_base();
        object2.transform_new_base();
        object3.transform_new_base();
        object4.transform_new_base();

        pub1.publish(object1.msgP);
        pub2.publish(object2.msgP);
        pub3.publish(object3.msgP);
        pub4.publish(object4.msgP);
        ++count;
        //--------------------------------------------------------------------
        ros::spinOnce();        
        loop_rate.sleep();  
    }
}
