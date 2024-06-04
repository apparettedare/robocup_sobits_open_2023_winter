#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    tf::TransformListener listener;
    while (ros::ok()) {
        std::string target_frame = "green_tea";
        std::string source_frame = "base_footprint";

        tf::StampedTransform transform;
        try
        {
            // ターゲットフレームからソースフレームへの変換を取得
            listener.lookupTransform(source_frame, target_frame, ros::Time(0), transform);

            // 取得した変換行列の情報を表示
            ROS_INFO("Translation: (%f, %f, %f)", -transform.getOrigin().x(), -transform.getOrigin().y(), -transform.getOrigin().z());
            // ROS_INFO("Rotation: (%f, %f, %f, %f)", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
