#include <ros/ros.h>
#include "web_speech_recognition/SpeechRecognition.h"
#include <string>
#include <vector>
#include <cstring>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "web_speech_recognition_node");
    ros::ServiceClient voice_client;
    ros::NodeHandle nh;

    // FILE *stream = popen("roslaunch web_speech_recognition web_speech_recognition_jp.launch", "r");

    voice_client = nh.serviceClient<web_speech_recognition::SpeechRecognition>("/speech_recognition");
    
    web_speech_recognition::SpeechRecognition speech_rec;
    std::vector<std::string> word_list;
    std::string word;
    speech_rec.request.timeout_sec = 5;
    
    while (ros::ok())
    {
        ros::spinOnce();
        if (voice_client.call(speech_rec))
        {
            ros::spinOnce();
            word_list = speech_rec.response.transcript;
            word = "";
            for (int i=0; i<word_list.size(); i++)
            {
                printf("%s:", word_list[i].c_str());
                // ROS_INFO("%s:", word_list[i].c_str());
                if (word_list[i] == "テスト")
                {
                    word = word_list[i];
                    break;
                }
            }
            if (word == "テスト")
            {
                ROS_INFO("attonourike");
                break;
            }
        }
        ros::spinOnce();
    }
    // pclose(stream);
    ros::spin();
    return 0;
}