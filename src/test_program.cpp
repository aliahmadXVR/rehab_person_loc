/**
 * @file test_program.cpp
 * @author Ali Ahmad (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "iostream"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include <cmath>
#include "rehab_person_loc/location_info.h"
#include "rehab_person_loc/time_info.h"
#include<ctime> // used to work with  system date and time
#include <fstream>

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
 
    float input_coord[24];
    time_t t; // t passed as argument in function time()
    struct tm * tt; // decalring variable for localtime()
    string log_file_loc = "rehab_robot_log.txt";


    void init_log_file()
    {
      //myfile.open ("example.bin", ios::out | ios::app)
      ofstream myfile;
      myfile.open (log_file_loc,ios::out | ios::app );
      myfile << "\n \n";
      myfile << "Date & Time \t \t \t \t Robot Location \t Person Location \n";
      myfile.close();
    }

    
    void init_coordinates()
    {
        ifstream file("/home/ali-ahmed/catkin_ws/src/rehab_person_loc/logs/data.txt");
        // ifstream file("$(find rehab_person_loc)/logs/data.txt");
        string data = "";
        
        int i = 0;

        if(file.is_open())
        {
            for(i = 0; (i<sizeof(input_coord)/sizeof(input_coord[0])); i++)
            {
                getline(file, data,',');
                    
                input_coord[i] = stof(data);
                cout << input_coord[i] << endl;
            }
            file.close();
        }
        else
        cout<<"Unable to Open Text File!!"<<endl;
        
    }

    public:

    MAP_TAG()
    {
        
        init_coordinates();
        // Actual Living Lab Coordinates (MAP-2) //
        kitchen.x1  =  input_coord[0];   kitchen.y1  = input_coord[1];   kitchen.x2  = input_coord[2];    kitchen.y2  = input_coord[3];
        lounge.x1   =  input_coord[4];   lounge.y1   = input_coord[5];   lounge.x2   = input_coord[6];    lounge.y2   = input_coord[7];
        entrance.x1 =  input_coord[8];   entrance.y1 = input_coord[9];   entrance.x2 = input_coord[10];   entrance.y2 = input_coord[11]; //to be re entered//
        lobby.x1    =  input_coord[12];  lobby.y1    = input_coord[13];  lobby.x2    = input_coord[14];    lobby.y2    = input_coord[15];
        tvRoom.x1   =  input_coord[16];  tvRoom.y1   = input_coord[17];  tvRoom.x2   = input_coord[18];    tvRoom.y2   = input_coord[19]; 
        bedRoom.x1  =  input_coord[20];  bedRoom.y1  = input_coord[21];  bedRoom.x2  = input_coord[22];    bedRoom.y2  = input_coord[23]; 
    }


    string get_time_local()
    {
        time (&t); //passing argument to time()
        tt = localtime(&t);
        // cout << "Current Day, Date and Time is = "<< asctime(tt)<<endl;
        string time_string;
        time_string = asctime(tt);
        time_string.erase(std::remove(time_string.begin(), time_string.end(), '\n'), time_string.end());
        return time_string;
    }

    void write_log()
    {
      ofstream myfile;
      myfile.open (log_file_loc,ios::out | ios::app );
      // myfile << "Date & Time \t \t Robot Location \t Person Location \n";
      myfile<<get_time_local();
     
      myfile.close();
    }

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_program");
    ros::NodeHandle n;
    
    MAP_TAG robot;

    
        
    return 0;
}





// ros::Rate loop_rate(10);

// while (ros::ok())
//     {
        



//         ros::spinOnce();
//         loop_rate.sleep();
//     }