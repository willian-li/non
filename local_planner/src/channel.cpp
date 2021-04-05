#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
 
using namespace std;
 


int main(int argc, char **argv)
{
    ros::init(argc, argv, "payload_drop");
    ros::NodeHandle nh("~");

    // 频率 [1hz]
    ros::Rate rate(1.0);
    // Drop cmd send to mavros
    ros::Publisher drop_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    
    mavros_msgs::OverrideRCIn drop_cmd;

 
    drop_cmd.channels[0] = 0;
    drop_cmd.channels[1] = 0;
    drop_cmd.channels[2] = 0;
    drop_cmd.channels[3] = 0;
    drop_cmd.channels[4] = 0;
    drop_cmd.channels[5] = 0;
    drop_cmd.channels[6] = 0;
    drop_cmd.channels[7] = 0;
    
    
    cout << "start"<<endl;
  
   
    while (1)
    {
	cout << "put channels 7"<<endl;
        cin >> drop_cmd.channels[6];
	cout << "put channels 8"<<endl;
        cin >> drop_cmd.channels[8];

        drop_pub.publish(drop_cmd);
 
        cout << "Droping......"<<endl;
 
        
        rate.sleep();
        ros::spinOnce();
    }
    cout << "finish"<<endl;
    sleep(1.0);
    return 0;
}
 
