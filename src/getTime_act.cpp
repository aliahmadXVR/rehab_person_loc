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
 */

#include "iostream"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include <cmath>
#include "rehab_person_loc/location_info.h"
#include "rehab_person_loc/time_info.h"
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
 
    enum LOC_TAG{kitchen_loc=1, lounge_loc=2, entrance_loc=3, lobby_loc=4, tvRoom_loc=5, bedRoom_loc=6, away_loc=7};

    public:

    MAP_TAG()
    {
        //Add the coordinates of the locations in map here//
        /*kitchen.x1  = 4.23; kitchen.y1  = 1.46;  kitchen.x2  = 7.46;   kitchen.y2  = 4.94;
        lounge.x1   = 1.66; lounge.y1   = 1.52;  lounge.x2   = 4.22;   lounge.y2   = 7.49;
        entrance.x1 = 0;    entrance.y1 = 0;     entrance.x2 = 1.58;   entrance.y2 = 6.85;
        lobby.x1    = 5.22; lobby.y1    = 4.81;  lobby.x2    = 7.1;    lobby.y2    = 7.3;
        tvRoom.x1   = 0.52; tvRoom.y1   = 7.57;  tvRoom.x2   = 4.63;   tvRoom.y2   = 12.12; 
        bedRoom.x1  = 5.2;  bedRoom.y1  = 7.4;   bedRoom.x2  = 8.9;    bedRoom.y2  = 12.1;*/

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
    LOC_TAG CURRENT_R, CURRENT_P;
    rehab_person_loc::location_info loc_info;
    rehab_person_loc::time_info time_info;

    struct TIME_TRACK
    {
        double kitchen;
        double lounge;
        double entrance;
        double lobby;
        double tvRoom;
        double bedRoom;
        double away;
    }person_time;
    
    bool FindPerson(double x, double y)
    {
        if (x < kitchen.x1 and x > kitchen.x2 and y > kitchen.y1 and y < kitchen.y2)
        {
          //cout<<"Person is *Inside Kitchen"<<endl;
          loc_info.person_location = "Inside Kitchen";
          CURRENT_P = kitchen_loc;
        }
        

        else if (x < lounge.x1 and x > lounge.x2 and y > lounge.y1 and y < lounge.y2)
        { 
          //cout<<"Person is *Inside Lounge"<<endl;
          loc_info.person_location = "Inside Lounge";        
          CURRENT_P = lounge_loc;
        }
        
        else if (x < entrance.x1 and x > entrance.x2 and y > entrance.y1 and y < entrance.y2)
        {
          //cout<<"Person is *At Entrance"<<endl;
          loc_info.person_location = "At Entrance";
          CURRENT_P = entrance_loc;
        }

        else if (x < lobby.x1 and x > lobby.x2 and y > lobby.y1 and y < lobby.y2)
        {
          //cout<<"Person is *Inside Lobby"<<endl;
          loc_info.person_location = "Inside Lobby";          
          CURRENT_P = lobby_loc;
        }

        else if (x < tvRoom.x1 and x > tvRoom.x2 and y > tvRoom.y1 and y < tvRoom.y2)
        {
          //cout<<"Person is *Inside TV Room"<<endl;
          loc_info.person_location = "Inside TvRoom";
          CURRENT_P = tvRoom_loc;
        }

        else if (x < bedRoom.x1 and x > bedRoom.x2 and y > bedRoom.y1 and y < bedRoom.y2)
        {
          //cout<<"Person is *Inside Bedroom"<<endl;
          loc_info.person_location = "Inside BedRoom"; 
          CURRENT_P = bedRoom_loc;
        }

        else 
        {
            //cout<<"--Away--"<<endl;
            loc_info.person_location = "Away";
            CURRENT_P = away_loc;
        }
    }

    bool FindRobot(double x, double y)
    {
        if (x < kitchen.x1 and x > kitchen.x2 and y > kitchen.y1 and y < kitchen.y2)
        {
          //cout<<"Robot is *Inside Kitchen"<<endl;
          loc_info.robot_location = "Inside Kitchen";
          CURRENT_R = kitchen_loc;
        }
        

        else if (x < lounge.x1 and x > lounge.x2 and y > lounge.y1 and y < lounge.y2)
        { 
          //cout<<"Robot is *Inside Lounge"<<endl;
          loc_info.robot_location = "Inside Lounge";        
          CURRENT_R = lounge_loc;
        }
        
        else if (x < entrance.x1 and x > entrance.x2 and y > entrance.y1 and y < entrance.y2)
        {
          //cout<<"Robot is *At Entrance"<<endl;
          loc_info.robot_location = "At Entrance";
          CURRENT_R = entrance_loc;
        }

        else if (x < lobby.x1 and x > lobby.x2 and y > lobby.y1 and y < lobby.y2)
        {
          //cout<<"Robot is *Inside Lobby"<<endl;
          loc_info.robot_location = "Inside Lobby";          
          CURRENT_R = lobby_loc;
        }

        else if (x < tvRoom.x1 and x > tvRoom.x2 and y > tvRoom.y1 and y < tvRoom.y2)
        {
          //cout<<"Robot is *Inside TV Room"<<endl;
          loc_info.robot_location = "Inside TvRoom";
          CURRENT_R = tvRoom_loc;
        }

        else if (x < bedRoom.x1 and x > bedRoom.x2 and y > bedRoom.y1 and y < bedRoom.y2)
        {
          //cout<<"Robot is *Inside Bedroom"<<endl;
          loc_info.robot_location = "Inside BedRoom"; 
          CURRENT_R = bedRoom_loc;
        }

        else 
        {
            //cout<<"--Away--"<<endl;
            loc_info.robot_location = "Away";
            CURRENT_R = away_loc;
        }
    }
} robot;

//Callback function for person location in camera_frame  a.k.a "/azure_link"
void person_loc_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    //Stores Person Location in Camera Frame//
    robot.person_loc_cam.header.stamp = msg->header.stamp;
    robot.person_loc_cam.header.frame_id = msg->header.frame_id;
    
    /* robot.person_loc_cam.point.x = msg->point.x;
    robot.person_loc_cam.point.y = msg->point.y;
    robot.person_loc_cam.point.z = msg->point.z; */

    //Assigning the points accroding to Azure Kinect Axis   
    robot.person_loc_cam.point.x = msg->point.z;
    robot.person_loc_cam.point.y = msg->point.x;
    robot.person_loc_cam.point.z = msg->point.y;
 
}

//CallBack Function for Subscriber to /amcl_pose)  
void localization_poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{  
    robot.robot_loc_map.header.stamp = msg->header.stamp;
    robot.robot_loc_map.header.frame_id = msg->header.frame_id;
    robot.robot_loc_map.point.x = msg->pose.pose.position.x;
    robot.robot_loc_map.point.y = msg->pose.pose.position.y;
    robot.robot_loc_map.point.z = msg->pose.pose.position.z;
}


//Timer Callback
void timerCallback(const ros::TimerEvent&)
{
  //ROS_INFO("Timer Callback");
  
  switch(robot.CURRENT_P)
  {
    case 1:
    {
      robot.person_time.kitchen ++;
      break;
    }

    case 2:
    {
      robot.person_time.lounge ++;
      break;
    }

    case 3:
    {
      robot.person_time.entrance ++;
      break;
    }

    case 4:
    {
      robot.person_time.lobby ++;
      break;
    }

    case 5:
    {
      robot.person_time.tvRoom ++;
      break;
    }

    case 6:
    {
      robot.person_time.bedRoom ++;
      break;
    }

    case 7:
    {
      robot.person_time.away ++;
      break;
    }
    default:
    break;
  }

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "track_person_time");
    ros::NodeHandle n;
    ros::Subscriber robot_sub = n.subscribe("/amcl_pose", 1000,localization_poseCallback);
    ros::Subscriber person_sub = n.subscribe("/person_loc", 1000,person_loc_callback);
    ros::Publisher person_Loc_pub = n.advertise<geometry_msgs::PointStamped>("/person_loc_estimated", 1000);
    ros::Publisher location_pub = n.advertise<rehab_person_loc::location_info>("/location_tag", 1000);
    ros::Publisher time_pub = n.advertise<rehab_person_loc::time_info>("/time_info", 1000);
    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);
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

            cout<<"Got the Transform"<<endl;                                    

            listener.lookupTransform("/map", "/azure_link",  
                                    now, transform);
        
            listener.transformPoint("map",robot.person_loc_cam,robot.person_loc_map);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        //Location Update over ROSTopic//
        robot.loc_info.stamp = ros::Time::now();
        robot.loc_info.frame_id = "map";
        location_pub.publish(robot.loc_info);

        //Time Update over ROSTopic//
        robot.time_info.kitchen_time = robot.person_time.kitchen;
        robot.time_info.lounge_time = robot.person_time.lounge;
        robot.time_info.entrance_time = robot.person_time.entrance;
        robot.time_info.lobby_time = robot.person_time.lobby;
        robot.time_info.tvRoom_time = robot.person_time.tvRoom;
        robot.time_info.bedRoom_time = robot.person_time.bedRoom;

        time_pub.publish(robot.time_info);

        
        //Timing Part
        robot.FindPerson(robot.person_loc_map.point.x,robot.person_loc_map.point.y);
        robot.FindRobot(robot.robot_loc_map.point.x,robot.robot_loc_map.point.y);
        
        /*cout<<"---"<<endl;
        cout<<"Current Loc Tag: "<<robot.CURRENT<<endl;

        cout<<"Kitchen Time: "<<robot.person_time.kitchen<<" sec"<<endl;
        cout<<"Lounge Time: "<<robot.person_time.lounge<<" sec"<<endl;
        cout<<"Entrance Time: "<<robot.person_time.entrance<<" sec"<<endl;
        cout<<"Lobby Time: "<<robot.person_time.lobby<<" sec"<<endl;
        cout<<"TvRoom Time: "<<robot.person_time.tvRoom<<" sec"<<endl;
        cout<<"BedRoom Time: "<<robot.person_time.bedRoom<<" sec"<<endl;
        cout<<"Away Time: "<<robot.person_time.away<<" sec"<<endl;
        cout<<"---"<<endl;*/

        // Publishing the Person location after transforming from
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
