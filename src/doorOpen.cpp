#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

float range_ahead = 0.0;

void callback(const sensor_msgs::LaserScan& msg) {
    range_ahead = msg.ranges[int(msg.ranges.size() / 2)];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "door_open_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_laser = nh.subscribe("/hsrb/base_scan", 10, callback);
    ros::Publisher pub_velocity = nh.advertise<geometry_msgs::Twist>("/hsrb/command_velocity", 10);
    ros::Rate loop_rate(10);
    // while (ros::ok()) {
    //     ROS_INFO("Range ahead: %.1f", range_ahead);
    //     if(range_ahead > 1.0){
    //         geometry_msgs::Twist twi;
    //         twi.linear.x = 0.2;
    //         pub_velocity.publish(twi);
    //         ros::spinOnce();
    //         loop_rate.sleep();
    //         break;
    //     }
    //     ros::spinOnce();
    // }
    while (ros::ok()) {
        ROS_INFO("Range ahead: %.1f", range_ahead);
        if(range_ahead > 1.0){
            geometry_msgs::Twist twi;
            twi.linear.x = 0.2;
            pub_velocity.publish(twi);
            ros::spinOnce();
            loop_rate.sleep();
            break;
        }
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
