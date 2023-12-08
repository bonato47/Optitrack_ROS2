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
    Vector3d pos, pos_obj;
    Vector4d quat, quat_obj;
    geometry_msgs::PoseStamped msgP;

    void CC_vrpn_base(const geometry_msgs::PoseStamped::ConstPtr msg) {  // Method/function defined inside the class
        pos    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
        quat   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
    } 
    void CC_vrpn_obj(const geometry_msgs::PoseStamped::ConstPtr msg) {  // Method/function defined inside the class
        pos_obj    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
        quat_obj   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
    } 
    void transform_new_base() {  // Method/function defined inside the class
        // transform the quaternion to rotation matrix
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

        // 90 rot in x for the base of Optitrack
        // NOTE: This matrix was previously used for tracking position in Z-up
        // currently pose tracking works in Y-up without it 
        // MatrixXd M_rot(4,4);
        // M_rot << 1, 0, 0, 0,
        //          0, 0, -1, 0,
        //          0, 1, 0, 0,
        //          0, 0, 0, 1;

        // Inverse * object => to have the position relatively of the base of the robot
        // Vector4d objtemp ={obj[0],obj[1],obj[2],1};
        // Vector4d outobj;
        
        // outobj = M_rot.inverse()*M.inverse()*objtemp;

        // msgP.pose.position.x = outobj[0];
        // msgP.pose.position.y = outobj[1];
        // msgP.pose.position.z = outobj[2];

        Quaterniond q_obj;
        q_obj.x() = quat_obj[0];
        q_obj.y() = quat_obj[1];
        q_obj.z() = quat_obj[2];
        q_obj.w() = quat_obj[3];    
        Matrix3d R_obj = q_obj.toRotationMatrix();
        MatrixXd M_obj(4,4), M_out(4,4);
        M_obj << R_obj(0,0),R_obj(0,1),R_obj(0,2),pos_obj[0],
              R_obj(1,0),R_obj(1,1),R_obj(1,2),pos_obj[1],
              R_obj(2,0),R_obj(2,1),R_obj(2,2),pos_obj[2],
              0,0,0,1;


        //M_out = M_rot.inverse()*M.inverse()*Mo;
        M_out = M.inverse()*M_obj;

        Matrix3d R_out;
        R_out << M_out(0,0),M_out(0,1),M_out(0,2),
              M_out(1,0),M_out(1,1),M_out(1,2),
              M_out(2,0),M_out(2,1),M_out(2,2);

        Quaterniond q_out(R_out);
        q_out.normalize();

        msgP.pose.position.x = M_out(0,3);
        msgP.pose.position.y = M_out(1,3);
        msgP.pose.position.z = M_out(2,3);
        msgP.pose.orientation.x = q_out.x();
        msgP.pose.orientation.y = q_out.y();
        msgP.pose.orientation.z = q_out.z();
        msgP.pose.orientation.w = q_out.w();

    } 

};


std::vector<vrpn> list_objects;
std::vector<ros::Publisher> list_pub;
std::vector<ros::Subscriber> list_sub_base;
std::vector<ros::Subscriber> list_sub;

int main(int argc, char **argv)
{
    string name_base;
    std::vector<std::string> subscribedTopics;
    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "objectbase");
    ros::NodeHandle Nh;

    Nh.getParam("/optitrack_publisher/name_base", name_base);
    Nh.getParam("/optitrack_publisher/list_object", subscribedTopics);


    ROS_INFO("Base topic is:");
    ROS_INFO("%s", name_base.c_str());

    // Subscribe to each topic in the list
    ROS_INFO("List of topics contains:");

    for (const auto& topic : subscribedTopics) {
        ROS_INFO("%s", topic.c_str());

        vrpn object;
        list_objects.emplace_back(object);
    }
    
    // Get the length of the list
    size_t list_n = list_objects.size();
    for (size_t i = 0; i < list_n; ++i) {
        list_sub_base.emplace_back(Nh.subscribe(name_base, 1000, &vrpn::CC_vrpn_base, &list_objects[i]));
        list_sub.emplace_back(Nh.subscribe(subscribedTopics[i], 1000, &vrpn::CC_vrpn_obj, &list_objects[i]));
        list_pub.emplace_back(Nh.advertise<geometry_msgs::PoseStamped>(subscribedTopics[i] + "_transform", 1000));
    }
   
    ros::Rate loop_rate(400);

    geometry_msgs::PoseStamped msgP1;


    //begin the ros loop
    ROS_WARN("WARNING! Verify that Y-up frame is applied on motive");
    double count = 0;
    while (ros::ok())
    {

        for (size_t i = 0; i < list_n; ++i) {
            vrpn object = list_objects[i];
            object.transform_new_base();

            ros::Publisher pub = list_pub[i];
            pub.publish(object.msgP);
        }

        ++count;
        //--------------------------------------------------------------------
        ros::spinOnce();        
        loop_rate.sleep();  
    }
}
