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
        MatrixXd M(4,4),M_rot(4,4);
        M << R(0,0),R(0,1),R(0,2),pos[0],
              R(1,0),R(1,1),R(1,2),pos[1],
              R(2,0),R(2,1),R(2,2),pos[2],
              0,0,0,1;
        // 90 rot in x for the base of Optitrack
        M_rot << 1, 0, 0, 0,
                 0, 0, -1, 0,
                 0, 1, 0, 0,
                 0, 0, 0, 1;

        // Inverse * object => to have the position relatively of the base oof the robot
        Vector4d objtemp ={obj[0],obj[1],obj[2],1};
        Vector4d outobj;
        
        outobj = M_rot.inverse()*M.inverse()*objtemp;

        msgP.pose.position.x = outobj[0];
        msgP.pose.position.y = outobj[1];
        msgP.pose.position.z = outobj[2];
    } 

};

vrpn object1, object2, object3, object4;

int main(int argc, char **argv)
{
    string name_object;
    string name_base;
    //Initialisation of the Ros Node (Service, Subscrber and Publisher)
    ros::init(argc, argv, "objectbase");
    ros::NodeHandle Nh;

    Nh.getParam("/optitrack_publisher/name_object", name_object);
    Nh.getParam("/optitrack_publisher/name_base", name_base);


    string name_object_transform = name_object+ "_transform";
    printf("\n%s\n",name_base.c_str());

    ros::Subscriber sub_BF1 = Nh.subscribe(name_base, 1000, &vrpn::CC_vrpn_base, &object1);
    
    ros::Subscriber sub_obj1 = Nh.subscribe(name_object, 1000, &vrpn::CC_vrpn_obj, &object1);
    
    ros::Publisher pub1 = Nh.advertise<geometry_msgs::PoseStamped>(name_object_transform, 1000);
   
    ros::Rate loop_rate(400);

    geometry_msgs::PoseStamped msgP1;

   ROS_INFO("Talker running");

    //begin the ros loop
    double count = 0;
    while (ros::ok())
    {
        object1.transform_new_base();
        
        pub1.publish(object1.msgP);

        ++count;
        //--------------------------------------------------------------------
        ros::spinOnce();        
        loop_rate.sleep();  
    }
}
