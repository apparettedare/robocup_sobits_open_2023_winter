#include <ros/ros.h>
#include <hsr_navigation_library/hsr_navigation_library.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_include_library");
    ros::NodeHandle nh;
    printf("attyon");
    HSRNavigationStack::HSRNavigationLibrary nav_lib;
    printf("purike");
    nav_lib.move2Location( "table", false );
    double start_time = ros::Time::now().toSec();
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if( !nav_lib.exist_goal_ ) {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    // nav_lib.move2Location( "kitchen", false );

    // while (ros::ok()) {
    //     if( !nav_lib.exist_goal_ ) {
    //         break;
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // nav_lib.move2Location( "table_left", false );

    // while (ros::ok()) {
    //     if( !nav_lib.exist_goal_ ) {
    //         break;
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // nav_lib.move2Location( "shelf", false );

    // while (ros::ok()) {
    //     if( !nav_lib.exist_goal_ ) {
    //         break;
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // nav_lib.move2Location( "table_right", false );

    // while (ros::ok()) {
    //     if( !nav_lib.exist_goal_ ) {
    //         break;
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // nav_lib.move2Location( "exit", false );

    // while (ros::ok()) {
    //     if( !nav_lib.exist_goal_ ) {
    //         break;
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    ros::spin();
}