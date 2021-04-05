/***************************************************************************************************************************
 * TEST.cpp
 *
 * Author: LYZ
 *
 * Update Time: 2018.8.17
 *
 * 说明: 货物投放
 *      1.起飞
 *      2.飞到0 0 1
 *      3.飞到2 0 1
 *      4.飞到2 0 0.2
 *      5.货物投放
 *      6.飞到0 2 1
 *      7.飞到0 2 0.2
 *      8.货物投放
 *      9.飞到2 2 1
 *      10.飞到2 2 0.2
 *      11.货物投放
 *      12。飞到0 0 1
 *      13.降落
***************************************************************************************************************************/

#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <px4_command/ControlCommand.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <command_to_mavros.h>
#include <Eigen/Eigen>

using namespace std;
 
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
px4_command::ControlCommand Command_Now;
//---------------------------------------正方形参数---------------------------------------------
float height_square;                //飞行高度
float pay_height;                //投放高度
float sleep_time;
int switch_flag = 0;
int time_sec = 0;  //记时函数
float limit_time;
float min_distance;
float distance_to_target;
Eigen::Vector3f drone_pos;
Eigen::Vector3f point0;
Eigen::Vector3f point1;
Eigen::Vector3f point2;
Eigen::Vector3f point3;
Eigen::Vector3f point4;
Eigen::Vector3f point5;
Eigen::Vector3f point6;

float cal_distance(Eigen::Vector3f a,Eigen::Vector3f b);
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    drone_pos  = Eigen::Vector3f(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "square");
    ros::NodeHandle nh("~");

    // 频率 [1hz]
    ros::Rate rate(1.0);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher move_pub = nh.advertise<px4_command::ControlCommand>("/px4_command/control_command", 10);
        //Subscribe the drone position
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    // Drop cmd send to mavros
    ros::Publisher drop_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("height_square", height_square, 1.0);
    nh.param<float>("sleep_time", sleep_time, 10.0);
    nh.param<float>("pay_height", sleep_time, 0.2);
    nh.param<float>("limit_time", limit_time, 10.0);
    nh.param<float>("min_distance", min_distance, 0.2);


    mavros_msgs::OverrideRCIn drop_cmd;

    drop_cmd.channels[0] = 0;
    drop_cmd.channels[1] = 0;
    drop_cmd.channels[2] = 0;
    drop_cmd.channels[3] = 0;
    drop_cmd.channels[4] = 0;
    drop_cmd.channels[5] = 0;
    drop_cmd.channels[6] = 0;
    drop_cmd.channels[7] = 1600;


    point0[0] = 0;
    point0[1] = 0;
    point0[2] = height_square;

    point1[0] = 2;
    point1[1] = 0;
    point1[2] = height_square;

    point2[0] = 2;
    point2[1] = 0;
    point2[2] = pay_height;

    point3[0] = 0;
    point3[1] = 2;
    point3[2] = height_square;

    point4[0] = 0;
    point4[1] = 2;
    point4[2] = pay_height;

    point5[0] = 2;
    point5[1] = 2;
    point5[2] = height_square;

    point6[0] = 2;
    point6[1] = 2;
    point6[2] = pay_height;


    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "height_square: "<<height_square<<"[m]"<<endl;
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }


    int comid = 0;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //takeoff
    //起飞
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point0[0];
        Command_Now.Reference_State.position_ref[1] = point0[1];
        Command_Now.Reference_State.position_ref[2] = point0[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point0);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        cout << "Takeoff: flying to Point0"<<endl;

        rate.sleep();
        ros::spinOnce();

    }

    //wait 1s more
    sleep(1.0);
    //reset the flag
    // 第一个点
    switch_flag = 0;
    time_sec = 0;
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point1[0];
        Command_Now.Reference_State.position_ref[1] = point1[1];
        Command_Now.Reference_State.position_ref[2] = point1[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point1);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        cout << "flying to Point1"<<endl;

        rate.sleep();
        ros::spinOnce();

    }



    //wait 1s more
    sleep(1.0);
    //reset the flag
    // 第2个点
    switch_flag = 0;
    time_sec = 0;
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point2[0];
        Command_Now.Reference_State.position_ref[1] = point2[1];
        Command_Now.Reference_State.position_ref[2] = point2[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point2);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        cout << "flying to Point2"<<endl;

        rate.sleep();
        ros::spinOnce();
    }


    sleep(1.0);
    //reset the flag
    switch_flag = 0;
    time_sec = 0;

    //Drop
    //投放货物
    while (time_sec < 3)
    {
        drop_pub.publish(drop_cmd);

        cout << "Droping......"<<endl;

        time_sec++;
        rate.sleep();
        ros::spinOnce();
    }

    //wait 1s more
    sleep(1.0);
    //reset the flag
    // 第3个点
    switch_flag = 0;
    time_sec = 0;
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point3[0];
        Command_Now.Reference_State.position_ref[1] = point3[1];
        Command_Now.Reference_State.position_ref[2] = point3[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point3);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        cout << "flying to Point3"<<endl;

        rate.sleep();
        ros::spinOnce();
    }


      //wait 1s more
    sleep(1.0);
    //reset the flag
    // 第4个点
    switch_flag = 0;
    time_sec = 0;
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point4[0];
        Command_Now.Reference_State.position_ref[1] = point4[1];
        Command_Now.Reference_State.position_ref[2] = point4[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point4);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        cout << "flying to Point4"<<endl;

        rate.sleep();
        ros::spinOnce();
    }

      sleep(1.0);
    //reset the flag
    switch_flag = 0;
    time_sec = 0;

    //Drop
    //投放货物
    while (time_sec < 3)
    {
        drop_pub.publish(drop_cmd);

        cout << "Droping......"<<endl;

        time_sec++;
        rate.sleep();
        ros::spinOnce();
    }

        //wait 1s more
    sleep(1.0);
    //reset the flag
    // 第5个点
    switch_flag = 0;
    time_sec = 0;
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point5[0];
        Command_Now.Reference_State.position_ref[1] = point5[1];
        Command_Now.Reference_State.position_ref[2] = point5[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point5);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        cout << "flying to Point5"<<endl;

        rate.sleep();
        ros::spinOnce();
    }


        //wait 1s more
    sleep(1.0);
    //reset the flag
    // 第6个点
    switch_flag = 0;
    time_sec = 0;
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point6[0];
        Command_Now.Reference_State.position_ref[1] = point6[1];
        Command_Now.Reference_State.position_ref[2] = point6[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point6);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        cout << "flying to Point6"<<endl;

        rate.sleep();
        ros::spinOnce();
    }

      sleep(1.0);
    //reset the flag
    switch_flag = 0;
    time_sec = 0;

    //Drop
    //投放货物
    while (time_sec < 3)
    {
        drop_pub.publish(drop_cmd);

        cout << "Droping......"<<endl;

        time_sec++;
        rate.sleep();
        ros::spinOnce();
    }


        //wait 1s more
    sleep(1.0);
    //reset the flag
    // 回到起点
    switch_flag = 0;
    time_sec = 0;
    while (switch_flag == 0)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = command_to_mavros::Move_ENU;  //Move模式
        Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;             //子模式：位置控制模式
        Command_Now.Reference_State.position_ref[0] = point0[0];
        Command_Now.Reference_State.position_ref[1] = point0[1];
        Command_Now.Reference_State.position_ref[2] = point0[2];
        Command_Now.Reference_State.yaw_ref = 0;
        Command_Now.Command_ID = comid;
        comid++;

        move_pub.publish(Command_Now);

        distance_to_target = cal_distance(drone_pos,point0);

        if(time_sec > limit_time ||  distance_to_target< min_distance )
        {
            switch_flag =1;
        }

        time_sec++;

        cout << "flying to Point0"<<endl;

        rate.sleep();
        ros::spinOnce();
    }


    //降落


    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode = command_to_mavros::Land;
    move_pub.publish(Command_Now);

    rate.sleep();

    cout << "Land"<<endl;





    return 0;
}
float cal_distance(Eigen::Vector3f a,Eigen::Vector3f b)
{
    float distance;
    distance = sqrt(  (a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]) + (a[2] - b[2])*(a[2] - b[2]) );
    return distance;
}