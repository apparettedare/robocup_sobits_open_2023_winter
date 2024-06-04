#include <cstdlib>
#include <math.h>
#include <iostream>
#include <string>

#include <boost/format.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <tmc_msgs/TalkRequestAction.h>
#include <tmc_msgs/TalkRequestGoal.h>
#include <tmc_msgs/Voice.h>

using namespace std;

namespace {
const double kConnectionTimeout = 10.0;

// Create speak sentences
// Index 0: in Japanese, 1: in English
const char* ask_order = {"ご注文はなんですか"};
}  // unnamed namespace

namespace hsrb_mounted_devices_samples {

/// @class Speaker
/// @brief Speak sentence in robot's language
class Speaker {
 public:
  Speaker() : talk_request_client_("/talk_request_action") {
    std::string talk_action = "/talk_request_action";

    // Wait for connection
    if (!talk_request_client_.waitForServer(
        ros::Duration(kConnectionTimeout))) {
      ROS_ERROR("%s does not exist", talk_action.c_str());
      exit(EXIT_FAILURE);
    }

    // Detect robot's language
    if (!strcmp(std::getenv("LANG"), "ja_JP.UTF-8")) {
      lang_ = tmc_msgs::Voice::kJapanese;
    } else {
      lang_ = tmc_msgs::Voice::kEnglish;
    }
  }

  int32_t GetLanguage() {
    return lang_;
  }

  bool SpeakSentence(const std::string& sentence) {
    tmc_msgs::TalkRequestGoal goal;
    goal.data.language = lang_;
    goal.data.sentence = sentence;

    if (talk_request_client_.sendGoalAndWait(goal) ==
        actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
    } else {
      return false;
    }
  }

 private:
  typedef actionlib::SimpleActionClient<
      tmc_msgs::TalkRequestAction> TalkRequestActionClient;

  TalkRequestActionClient talk_request_client_;
  int32_t lang_;
};
}  // namespace hsrb_mounted_devices_samples

int main(int argc, char** argv) {
  ros::init(argc, argv, "speak_node");

  hsrb_mounted_devices_samples::Speaker speaker;

  while (ros::ok()) {
    ROS_INFO("ご注文はなんですか");
    speaker.SpeakSentence(ask_order);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }
  ros::spin();

  return 0;
}
