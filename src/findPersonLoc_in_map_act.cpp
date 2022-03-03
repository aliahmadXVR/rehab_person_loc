/**
 * @file findPersonLoc_in_map.cpp
 * @author Ali Ahmad (ali.ahmad@xavor.com)
 * @date 2022-02-24
 * 
 * DESCRIPTION:
 * This node subscribes to the /person_loc topic which is publishing 
 * the person location in camera frame a.k.a  "/azure_link". The node transforms
 * the point in the map frame and displays the location of person in map. 
 * In later stages, this node will subscribe to the person location topic 
 * provided by AI team and the same process will be done on that point. 
 * This specific node is being used for Testing on actual robot.
 */

#include "iostream"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include <cmath>
using namespace std;


class MAP_TAG
{
    private:
    
    struct COORDINATE
    {
        double x1;
        double y1;
        double x2;
        double y2;
    }kitchen,lounge,entrance,tvRoom,bedRoom,lobby;
 
    public:

    MAP_TAG()
    {
        // Actual Living Lab Coordinates (MAP-1) //
        /*kitchen.x1  = -0.2;  kitchen.y1  = -43.5;  kitchen.x2  = -3.5;    kitchen.y2  = -40.5;
        lounge.x1   = -0.3;  lounge.y1   = -46;    lounge.x2   = -6.1;    lounge.y2   = -43;
        entrance.x1 = -2.6;  entrance.y1 = -47.3;  entrance.x2 = -5.6;    entrance.y2 = -45.9; //to be re entered//
        lobby.x1    = -3.6;  lobby.y1    = -43.1;  lobby.x2    = -6.2;    lobby.y2    = -41.1;
        tvRoom.x1   = -6.2;  tvRoom.y1   = -47.2;  tvRoom.x2   = -10.9;   tvRoom.y2   = -43.4; 
        bedRoom.x1  = -6.1;  bedRoom.y1  = -43.0;  bedRoom.x2  = -11.1;    bedRoom.y2 = -39.4;*/
        
        // Actual Living Lab Coordinates (MAP-2) //
        kitchen.x1  =  1.97;  kitchen.y1  = 1.50;  kitchen.x2  = -2.4;    kitchen.y2  =4.3;
        lounge.x1   =  0.67;  lounge.y1   = -1.24;    lounge.x2   = -5.1;    lounge.y2   = 1.4;
        entrance.x1 = -1.63;  entrance.y1 = -2.7;  entrance.x2 = -4.6;    entrance.y2 = -1.3; //to be re entered//
        lobby.x1    = -2.7;  lobby.y1    = 2.7;  lobby.x2    = -5.1;    lobby.y2    = 4.2;
        tvRoom.x1   = -5.2;  tvRoom.y1   = -2.5;  tvRoom.x2   = -10;   tvRoom.y2   = 1.4; 
        bedRoom.x1  = -5.1;  bedRoom.y1  = 2.27;  bedRoom.x2  = -9.9;    bedRoom.y2 = 6.2; 
    }

    geometry_msgs::PointStamped robot_loc_map;
    geometry_msgs::PointStamped person_loc_cam;
    geometry_msgs::PointStamped person_loc_base;
    geometry_msgs::PointStamped person_loc_map;
    
    bool FindPoint(double x, double y)
    {
        if (x < kitchen.x1 and x > kitchen.x2 and y > kitchen.y1 and y < kitchen.y2)
        cout<<"Person is *Inside Kitchen"<<endl;

        else if (x < lounge.x1 and x > lounge.x2 and y > lounge.y1 and y < lounge.y2)
        cout<<"Person is *Inside Lounge"<<endl;        
        
        else if (x < entrance.x1 and x > entrance.x2 and y > entrance.y1 and y < entrance.y2)
        cout<<"Person is *At Entrance"<<endl;  

        else if (x < lobby.x1 and x > lobby.x2 and y > lobby.y1 and y < lobby.y2)
        cout<<"Person is *Inside Lobby"<<endl;  

        else if (x < tvRoom.x1 and x > tvRoom.x2 and y > tvRoom.y1 and y < tvRoom.y2)
        cout<<"Person is *Inside TV Room"<<endl; 

        else if (x < bedRoom.x1 and x > bedRoom.x2 and y > bedRoom.y1 and y < bedRoom.y2)
        cout<<"Person is *Inside Bedroom"<<endl; 

        else 
        {
            cout<<"--Away--"<<endl;
        }
    }
   
} robot; 

//CallBack Function for Subscriber to /rtabmap/localization_pose)  //Not being used anymore//
/*void localization_poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{  
}*/

//Callback function for person location in camera_frame  a.k.a "/azure_link"
void person_loc_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    //Stores Person Location in Camera Frame//
    robot.person_loc_cam.header.stamp    =   msg->header.stamp;
    robot.person_loc_cam.header.frame_id = msg->header.frame_id;
    robot.person_loc_cam.point.x = msg->point.x;
    robot.person_loc_cam.point.y = msg->point.y;
    robot.person_loc_cam.point.z = msg->point.z;
    
    //Debug Purpose Only
    /*cout<<"Person X: "<<robot.person_loc_map.point.x<<endl;
    cout<<"Person Y: "<<robot.person_loc_map.point.y<<endl;
    cout<<"-------"<<endl;*/

  
    robot.FindPoint(robot.person_loc_map.point.x,robot.person_loc_map.point.y);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "findPointLoc_in_map");
    ros::NodeHandle n;
    //ros::Subscriber robot_sub = n.subscribe("/rtabmap/localization_pose", 1000,localization_poseCallback);
    ros::Subscriber person_sub = n.subscribe("/person_loc", 1000,person_loc_callback);
    ros::Publisher person_Loc_pub = n.advertise<geometry_msgs::PointStamped>("/person_loc_estimated", 1000);
    ros::Rate loop_rate(10);
    unsigned int seq = 0;
    
    tf::TransformListener listener;

    while (ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            ros::Time now = ros::Time::now();

            listener.waitForTransform("/map", "/azure_link",
                                    now, ros::Duration(3.0));
            listener.lookupTransform("/map", "/azure_link",  
                                    now, transform);
        
            listener.transformPoint("map",robot.person_loc_cam,robot.person_loc_map);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::PointStamped person_location_est;
        person_location_est.header.seq = seq;
        ++seq;
        person_location_est.header.frame_id = "map"; 
        person_location_est.header.stamp = ros::Time::now();
        person_location_est.point.x = robot.person_loc_map.point.x;
        person_location_est.point.y = robot.person_loc_map.point.y;
        person_location_est.point.z = robot.person_loc_map.point.z;
        person_Loc_pub.publish(person_location_est);  //publish person location

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
