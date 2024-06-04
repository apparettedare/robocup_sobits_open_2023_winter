#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <tmc_msgs/TalkRequestAction.h>
#include <tmc_msgs/TalkRequestGoal.h>
#include <tmc_msgs/Voice.h>
#include <hsr_navigation_library/hsr_navigation_library.hpp>
#include <web_speech_recognition/SpeechRecognition.h>
#include <tf/transform_listener.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>
#include <math.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <boost/format.hpp>
#include <vector>
#include <regex>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;
using namespace Eigen;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointClient;
typedef actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction> ApplyForceClient;

// joint
float arm_flex_joint = 0.0;
float arm_lift_joint = 0.0;
float arm_roll_joint = 0.0;
float wrist_flex_joint = 0.0;
float wrist_roll_joint = 0.0;
float head_pan_joint = 0.0;
float head_tilt_joint = 0.0;

//　逆運動学で使う変数
MatrixXd W_E = MatrixXd::Identity(3, 3) * 0.01;
MatrixXd W_N_bar_p = MatrixXd::Identity(3, 3) * 0.01;
MatrixXd W_N_bar_r = MatrixXd::Identity(2, 2); 
float arm_flex_to_arm_roll = 0.345;
float wrist_to_hand = 0.192;
float x_delta = -0.1;
float y_delta = -0.1;
float z_delta = 0.06;
float potato_z_delta = 0.09;
float arm_delta = -0.01;
float threshold = 0.001; 
int maxIterations = 1000;
int iteration = 0;

bool flag1 = false;
bool flag2 = false;
bool find_object = false;
bool is_right = false;
float range_ahead = 0.0;
float object_x = 0.0;
float object_y = 0.0;
float object_z = 0.0;
float hand_to_base = 0.078000;

string speak_word = "";
string target_name = "ポテトチップス";
string target_frame = "/green_tea";
string source_frame = "/base_footprint";
VectorXd position(3);
VectorXd target_position(3);

nav_msgs::Odometry odom;


// 残差を計算する関数
void cal_e_pos(VectorXd& e, VectorXd& q, Vector2d& target_pos) {
    // 目標位置と現在の位置の差分を計算
    e(0) = target_pos(0) - (arm_flex_to_arm_roll * sin(q(1)) + wrist_to_hand * sin(q(1) + q(2)));
    e(1) = 0;
    e(2) = target_pos(1) - (0.339 + q(0) + arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2)));
    return;
}

void cal_e_rot(VectorXd& e, VectorXd& q, float target_rot) {
    // 目標位置と現在の姿勢の差分を計算
    MatrixXd dR(3, 3), R(3, 3), rrq(3, 3);
    VectorXd l(3), m(3), a(3);
    float currentY = q(1)+ q(2);
    float theta = 0.0;
    dR << cos(theta), 0, -sin(theta),
                        0, 1, 0,
            sin(theta), 0, cos(theta);

    R << cos(currentY), 0, -sin(currentY),
                        0, 1, 0,
            sin(currentY), 0, cos(currentY);
    rrq = dR * R.transpose();

    l(0) = rrq(1,2) - rrq(2,1);
    l(1) = rrq(2,0) - rrq(0,2);
    l(2) = rrq(0,1) - rrq(1,0);

    m(0) = rrq(0,0) + 1;
    m(1) = rrq(1,1) + 1;
    m(2) = rrq(2,2) + 1;

    // Rが対角行列
    if(theta == 0) a.setZero();
    else if(theta == M_PI) a = M_PI / 2 * m;

    // Rが対角行列でない
    else a = atan2(l.squaredNorm(), rrq(0,0) + rrq(1,1) + rrq(2,2) - 1) / l.squaredNorm() * l;
    e = a;
    return;
}

// ヤコビ行列を計算する関数
void cal_J_pos(MatrixXd& J, VectorXd& q) {
    J << 0, arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2)), wrist_to_hand * cos(q(1) + q(2)),
            0,                                                                   0,                                 0,
            1, -arm_flex_to_arm_roll * sin(q(1)) - wrist_to_hand * sin(q(1) + q(2)), -wrist_to_hand * sin(q(1) + q(2));
    return;
}

void cal_J_rot(MatrixXd& J) {
    J << 0, 0,
        1, 1,
        0, 0;
    return;
}

// 勾配を計算する関数
void cal_g(VectorXd& g, const MatrixXd& J, const VectorXd& e) {
    g = J.transpose() * W_E * e;
    return;
}   

// 減衰因子行列を計算する関数
void cal_W_N_pos(MatrixXd &W_N_p, const VectorXd &e_p) {
    W_N_p = MatrixXd::Identity(3, 3) * e_p.squaredNorm() + W_N_bar_p;
    return;
}

void cal_W_N_rot(MatrixXd &W_N_r, const VectorXd &e_r) {
    W_N_r = MatrixXd::Identity(2, 2) * e_r.squaredNorm() + W_N_bar_r;
    return;
}

void cal_H(MatrixXd &H, const MatrixXd &J, const MatrixXd &W_N) {
    H = J.transpose() * W_E * J + W_N;
    return;
}

// 逆運動学を行う関数
VectorXd solve(VectorXd& q_ref, Vector2d& target_pos, float target_rot) {
    ROS_INFO("Solve Inverse Kinematics");
    MatrixXd J_p(3, 3), J_r(3, 2), W_N_p(3, 3), W_N_r(2, 2), H_p(3, 3), H_r(2, 2), R(3, 3), dR(3, 3);
    VectorXd e_p(3), e_r(3), g_p(3), g_r(2), q(3), q_new(3), delta_q_p(3), delta_q_r(2);
    q = q_ref;
    // 逆運動学の反復計算
    do {
        // 角度
        cal_e_rot(e_r, q, target_rot);
        cal_J_rot(J_r);
        cal_g(g_r, J_r, e_r);
        cal_W_N_rot(W_N_r, e_r);
        cal_H(H_r, J_r, W_N_r);
        // 角度の変化量を計算
        delta_q_r = H_r.inverse() * g_r;
        // 角度を更新
        q(1) += delta_q_r(0);
        q(2) += delta_q_r(1);
        // 位置
        cal_e_pos(e_p, q, target_pos);
        cal_J_pos(J_p, q);
        cal_g(g_p, J_p, e_p);
        cal_W_N_pos(W_N_p, e_p);
        cal_H(H_p, J_p, W_N_p);
        // 位置の変化量を計算
        delta_q_p = H_p.inverse() * g_p;
        // 位置を更新
        q += delta_q_p;
        if (q(0) > 0.69) q(0) = 0.69;
        else if (q(0) < 0) q(0)  = 0; 
        if (q(1) < 0) q(1)  = 0;
        else if (q(1) > 2.617) q(1) = 2.617;
        if (q(2) < -1.221) q(2)  = -1.221;
        else if (q(2) > 1.919) q(2) = 1.919;
        // 反復回数をインクリメント
        iteration++;
    } while (abs(delta_q_p(0)) > threshold && abs(delta_q_p(1)) > threshold && abs(delta_q_p(2)) > threshold  && iteration < maxIterations);
    q_ref = q;
    iteration = 0;
    VectorXd answer_pos(3);
    answer_pos = q;
    return answer_pos;
}

namespace text2speech {

/// @class Speaker
/// @brief Speak sentence in robot's language
class Speaker {
 public:
  Speaker() : talk_request_client_("/talk_request_action") {
    std::string talk_action = "/talk_request_action";
    // Wait for connection
    if (!talk_request_client_.waitForServer(
        ros::Duration(10.0))) {
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
}

void callback_laser(const sensor_msgs::LaserScan& msg) {
    flag1 = true;
    range_ahead = msg.ranges[int(msg.ranges.size() / 2)];
}

void callback_odom(const nav_msgs::Odometry& msg) {
    flag2 = true;
    odom = msg;
}

void odom_control(float position_x, float position_y, float degree) {
    ROS_INFO("Odom control");
    JointClient cli("/hsrb/omni_base_controller/follow_joint_trajectory", true);
    cli.waitForServer();
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(
    "/hsrb/controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers;
    bool running = false;
    while(running == false) {
        ros::Duration(0.1).sleep();
        if(client.call(list_controllers)) {
        for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
            controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
            if(c.name == "omni_base_controller" && c.state == "running") running = true;
        }
        }
    }
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("odom_x");
    goal.trajectory.joint_names.push_back("odom_y");
    goal.trajectory.joint_names.push_back("odom_t");
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(3);
    goal.trajectory.points[0].positions[0] = odom.pose.pose.position.x + position_x;
    goal.trajectory.points[0].positions[1] = odom.pose.pose.position.y + position_y;
    goal.trajectory.points[0].positions[2] = degree;
    goal.trajectory.points[0].velocities.resize(3);
    ROS_INFO("move_x: %.1f move_y: %.1f move_angle: %.1f", goal.trajectory.points[0].positions[0], goal.trajectory.points[0].positions[1], goal.trajectory.points[0].positions[2]);
    for(size_t i = 0; i < 3; ++i) {
        goal.trajectory.points[0].velocities[i] = 0.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(15.0);
    // send message to the action server
    cli.sendGoal(goal);
    // wait for the action server to complete the order
    cli.waitForResult(ros::Duration(10.0));
    ros::spinOnce();
}

void head_control(float head_pan_joint, float head_tilt_joint) {
    ROS_INFO("Head control");
    // initialize action client
    JointClient cli("/hsrb/head_trajectory_controller/follow_joint_trajectory", true);
    // wait for the action server to establish connection
    cli.waitForServer();
    // make sure the controller is running
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(
        "/hsrb/controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers;
    bool running = false;
    while(running == false) {
        ros::Duration(0.1).sleep();
        if(client.call(list_controllers)) {
        for(unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
            controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
            if(c.name == "head_trajectory_controller" && c.state == "running") running = true;
        }
        }
    }
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("head_pan_joint");
    goal.trajectory.joint_names.push_back("head_tilt_joint");
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = head_pan_joint;
    goal.trajectory.points[0].positions[1] = head_tilt_joint;
    goal.trajectory.points[0].velocities.resize(2);
    for(size_t i = 0; i < 2; ++i) {
        goal.trajectory.points[0].velocities[i] = 0.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(3.0);
    // send message to the action server
    cli.sendGoal(goal);
    // wait for the action server to complete the order
    cli.waitForResult(ros::Duration(10.0));
}

void arm_control(float arm_lift_joint, float arm_flex_joint, float arm_roll_joint, float wrist_flex_joint,float wrist_roll_joint) {
    ROS_INFO("Arm control");
    // initialize action client
    JointClient cli("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true);
    // wait for the action server to establish connection
    cli.waitForServer();
    // make sure the controller is running
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(
        "/hsrb/controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers;
    bool running = false;
    while(running == false) {
        ros::Duration(0.1).sleep();
        if(client.call(list_controllers)) {
        for(unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
            controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
            if(c.name == "arm_trajectory_controller" && c.state == "running") running = true;
        }
        }
    }
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("arm_lift_joint");
    goal.trajectory.joint_names.push_back("arm_flex_joint");
    goal.trajectory.joint_names.push_back("arm_roll_joint");
    goal.trajectory.joint_names.push_back("wrist_flex_joint");
    goal.trajectory.joint_names.push_back("wrist_roll_joint");
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(5);
    goal.trajectory.points[0].positions[0] = arm_lift_joint;
    goal.trajectory.points[0].positions[1] = arm_flex_joint;
    goal.trajectory.points[0].positions[2] = arm_roll_joint;
    goal.trajectory.points[0].positions[3] = wrist_flex_joint;
    goal.trajectory.points[0].positions[4] = wrist_roll_joint;
    goal.trajectory.points[0].velocities.resize(5);
    ROS_INFO("arm_lift_joint: %.1f", arm_lift_joint);
    ROS_INFO("arm_flex_joint: %.1f", arm_flex_joint);
    ROS_INFO("wrist_flex_joint: %.1f", wrist_flex_joint);
    for(size_t i = 0; i < 5; ++i) {
        goal.trajectory.points[0].velocities[i] = 0.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(3.0);
    // send message to the action server
    cli.sendGoal(goal);
    // wait for the action server to complete the order
    cli.waitForResult(ros::Duration(10.0));
}

void nav(string location) {
    ROS_INFO("navigation");
    ros::Rate loop_rate(10);
    HSRNavigationStack::HSRNavigationLibrary nav_lib;
    ROS_INFO("Move to %s", location.c_str());
    nav_lib.move2Location( location, false );
    ros::spinOnce();
    while(ros::ok()) {
        ros::spinOnce();
        if(!nav_lib.exist_goal_ ) {
            ROS_INFO("Arrived at %s", location.c_str());
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spinOnce();
}

void door_open(float move) {
    ROS_INFO("Door open");
    ros::Rate loop_rate(10);
    ros::spinOnce();
    while(ros::ok()) {
        ros::spinOnce();
        ROS_INFO("range: %.1f", range_ahead);
        if(range_ahead > 1.0) {
            odom_control(move, 0.0, 0.0);
            ros::Duration(5).sleep();
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("door open succeeded");
    ros::spinOnce();
}

void speak(string word) {
    cout << word << endl;
    text2speech::Speaker speaker;
    ros::spinOnce();
    speaker.SpeakSentence(word);
    ros::spinOnce();
}

string ask_order() {
    ROS_INFO("Ask order");
    ros::NodeHandle nh;
    string ask_order = "ご注文はなんですか";
    vector<string> word_list;
    string target_name = "お茶";
    regex word_regex("(ポテトチップス|お茶|カップヌードル)");
    smatch match;
    web_speech_recognition::SpeechRecognition speech_rec;
    ros::ServiceClient VoiceClient;
    VoiceClient = nh.serviceClient<web_speech_recognition::SpeechRecognition>("/speech_recognition");
    speech_rec.request.timeout_sec = 5;
    bool asked_order = false;
    ros::spinOnce();
    while(ros::ok()) {
        ros::spinOnce();
        speak(ask_order);
        ros::Duration(1.0).sleep();
        if(VoiceClient.call(speech_rec)) {
            ros::spinOnce();
            word_list = speech_rec.response.transcript;
            // 注文の中に特定の単語が含まれているかをチェック
            for(int i = 0; i < word_list.size(); i++) {
                cout << word_list[i] << endl;
                if(std::regex_search(word_list[i], match, word_regex)) {
                    target_name = match.str();
                    break;
                }
            }
            if(!target_name.empty()) {
                speak_word = "ご注文は" + target_name + "ですか はい,か,いいえで答えてください";
                speak(speak_word);
                ros::Duration(1.0).sleep();
                if(VoiceClient.call(speech_rec)) {
                    ros::spinOnce();
                    word_list = speech_rec.response.transcript;
                    for(int i = 0; i < word_list.size(); i++) {
                        cout << word_list[i] << endl;
                        if(word_list[i] == "はい") {
                            speak_word = "かしこまりました";
                            speak(speak_word);
                            ros::Duration(1.0).sleep();
                            // 注文を確定し、ループを抜ける
                            asked_order = true;
                            break;
                        }else {
                            // 「いいえ」が答えられた場合、もう一度注文を尋ねる
                            speak_word = "えー";
                            speak(speak_word);
                            ros::Duration(1.0).sleep();
                            asked_order = false;
                            break;
                        }
                    }
                }
            }else {
                // もし他の注文が来たら、その処理をここに追加する
                speak_word = "申し訳ありません、その注文には対応していません";
                speak(speak_word);
                ros::Duration(1.0).sleep();
            }
        }
        if(asked_order) {
            ROS_INFO("Ask order succeeded");
            break;
        }
        ros::spinOnce();
    }
    FILE *fpp = popen("rosnode kill /web_video_server /web_video_server2", "r");
    ros::Duration(2.0).sleep();
    ros::spinOnce();
    return target_name;
}

//  TFを取得する関数
VectorXd get_position() {
    ROS_INFO("Get position");
    ros::Rate loop_rate(10);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::spinOnce();
    ros::Time start_time = ros::Time::now();
    while(ros::ok()) {
        ros::spinOnce();
        if(listener.canTransform(source_frame, target_frame, ros::Time(0))) {
            listener.lookupTransform(source_frame, target_frame, ros::Time(0), transform);
            ROS_INFO("Translation: (%f, %f, %f)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            object_x = transform.getOrigin().x();
            object_y = transform.getOrigin().y();
            object_z = transform.getOrigin().z();
            find_object = true;
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
        // 10秒経過したかどうかをチェック
        if((ros::Time::now() - start_time).toSec() > 10.0) {
            ROS_INFO("Timeout: Failed to get transform within 10 seconds");
            find_object = false;
            break;
        }
    }
    ros::spinOnce();
    position << object_x, object_y, object_z;
    return position;
}

// 物体認識をする関数
VectorXd object_recognition(string target_name) {
    ROS_INFO("Object recognition");
    FILE *fp2 = popen("roslaunch yolov8_ros yolov8_with_tf.launch weight_name:=object.pt", "r");
    ros::Duration(5.0).sleep();
    speak_word = target_name + "を探します";
    speak(speak_word);
    ros::Duration(5.0).sleep();
    position = get_position();
    FILE *kill_fp = popen("rosnode kill /yolov8_ros/bbox_to_tf /yolov8_ros/rviz /yolov8_ros/yolov8", "r");
    ros::Duration(0.5).sleep();
    if(find_object) speak_word = "見つかりました";
    else speak_word = "見つかりませんでした";
    speak(speak_word);
    ros::Duration(1.0).sleep();
    return position;
}

// 物体の近くまで移動する関数
void move_to_target_object(VectorXd target_position, float move_delta) {
    ROS_INFO("Move to target object");
    VectorXd q(3);
    Vector2d target_pos(2);
    VectorXd ans_pos(3);
    float target_rot = 0.0;
    // arm_lift_joint = 0.6836822057617669;
    // arm_flex_joint = -2.138891547845276;
    // wrist_flex_joint = 0.5600194233461813;
    float hand_to_target = target_position(2) - 0.87;
    arm_lift_joint = 0.5636992756806017 + hand_to_target;
    arm_flex_joint = -1.598930547845276;
    wrist_flex_joint = -0.009968576653818673;
    q << arm_lift_joint, arm_flex_joint, wrist_flex_joint;
    target_pos << target_position(1), target_position(2);
    float move_x = 0.0;
    if(target_pos(0) > (arm_flex_to_arm_roll + wrist_to_hand)) {
        move_x = target_pos(0) - arm_flex_to_arm_roll - wrist_to_hand;
        target_pos(0) = arm_flex_to_arm_roll + wrist_to_hand - move_delta;
    }
    if(target_pos(1) > (0.339 + 0.69 + arm_flex_to_arm_roll + wrist_to_hand)) target_pos(1) = 0.339 + 0.69 + arm_flex_to_arm_roll + wrist_to_hand;
    ans_pos = solve(q, target_pos, target_rot);
    ROS_INFO("target_pos_x, target_pos_z: (%f, %f)", -target_pos(0), target_pos(1));
    odom_control(-target_position(0) - hand_to_base, -move_x + move_delta, -M_PI/2);
    arm_lift_joint = ans_pos(0) + arm_delta;
    arm_flex_joint = -ans_pos(1);
    arm_roll_joint = 2.5574241302450673e-07;
    wrist_flex_joint = -ans_pos(2);
    wrist_roll_joint = -0.062042862168314894;
    head_pan_joint = 0.017863324470846376;
    head_tilt_joint = -0.6664196769555847;
    arm_control(arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint);
    ros::Duration(0.5).sleep();
    head_control(head_pan_joint, head_tilt_joint);
    ros::Duration(0.5).sleep();
    odom_control(0.0, 0.02 - move_delta, -M_PI/2);
    ros::Duration(10.0).sleep();
}

// 腕を閉じる関数
void grasp(float force) {
    ROS_INFO("Grasp");
    // initialize action client
    ApplyForceClient cli("/hsrb/gripper_controller/apply_force", true);
    // wait for the action server to establish connection
    cli.waitForServer();
    // make sure the controller is running
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(
        "/hsrb/controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers;
    bool running = false;
    while(running == false) {
        ros::Duration(0.1).sleep();
        if(client.call(list_controllers)) {
        for(unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
            controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
            if(c.name == "gripper_controller" && c.state == "running") {
            running = true;
            }
        }
        }
    }
    // fill ROS message
    tmc_control_msgs::GripperApplyEffortGoal goal;
    goal.effort = 1.0;
    // send message to the action server
    cli.sendGoal(goal);
    // wait for the action server to complete the order
    cli.waitForResult(ros::Duration(5.0));
}

// 認識する姿勢にする関数
void recognition_pose() {
    ROS_INFO("Recognition pose");
    arm_lift_joint = 0.48394059797982736;
    arm_flex_joint = -0.22179854784527597;
    arm_roll_joint = -1.600002744257587;
    wrist_flex_joint = -1.7919765766538185;
    wrist_roll_joint = -0.062042862168314894;
    head_pan_joint = 1.5498503244708468;
    head_tilt_joint = -0.5464426769555847;
    arm_control(arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint);
    ros::Duration(0.5).sleep();
    head_control(head_pan_joint, head_tilt_joint);
    ros::Duration(0.5).sleep();
    odom_control(0.0, 0.03, M_PI);
    ros::spinOnce();
}

// 平面検出する関数
VectorXd placement_recognition() {
    ROS_INFO("Placement position");
    FILE *fp5 = popen("roslaunch placeable_position_estimator placeable_position_estimator.launch", "r");
    ros::Duration(10.0).sleep();
    target_frame = "placeable_point";
    speak_word = "置く場所を探します";
    speak(speak_word);
    position = get_position();
    if(target_name == "ポテトチップス") position(2) += potato_z_delta;
    else position(2) += z_delta;
    FILE *fp6 = popen("rosnode kill /placeable_position_estimator/placeable_position_estimater_node", "r");
    ros::Duration(1.0).sleep();
    ROS_INFO("position (x, y, z): (%f, %f, %f)", position(0), position(1), position(2));
    if(find_object) speak_word = "見つかりました";
    else speak_word = "見つかりませんでした";    speak(speak_word);
    return position;
}

// 腕を開く関数
void open_hand() { 
    ROS_INFO("Open hand");
    // initialize action client
    ApplyForceClient cli("/hsrb/gripper_controller/grasp", true);
    // wait for the action server to establish connection
    cli.waitForServer();
    // make sure the controller is running
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(
        "/hsrb/controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers;
    bool running = false;
    while(running == false) {
        ros::Duration(0.1).sleep();
        if(client.call(list_controllers)) {
        for(unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
            controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
            if(c.name == "gripper_controller" && c.state == "running") running = true;
        }
        }
    }
    // fill ROS message
    tmc_control_msgs::GripperApplyEffortGoal goal;
    goal.effort = 1.0;
    // send message to the action server
    cli.sendGoal(goal);
    // wait for the action server to complete the order
    cli.waitForResult(ros::Duration(5.0));
}

// 初期状態の姿勢にする関数
void set_initial_pose() {
    ROS_INFO("Set initial pose");
    arm_lift_joint = 0.0;
    arm_flex_joint = 0.0;
    arm_roll_joint = -1.57;
    wrist_flex_joint = -1.57;
    wrist_roll_joint = 0.0;
    head_pan_joint = 0.0;
    head_tilt_joint = 0.0;  
    head_control(head_pan_joint, head_tilt_joint);
    ros::Duration(0.5).sleep();
    arm_control(arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint);
    ros::Duration(0.5).sleep();
}

void gripper_control(float pos) {
  // initalize ROS publisher
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/gripper_controller/command", 10);
  // wait to establish connection between the controller
  while (pub.getNumSubscribers() == 0) {
    ros::Duration(0.1).sleep();
  }
  // make sure the controller is running
  ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(
      "/hsrb/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  bool running = false;
  while (running == false) {
    ros::Duration(0.1).sleep();
    if (client.call(list_controllers)) {
      for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
        controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
        if (c.name == "gripper_controller" && c.state == "running") running = true;
      }
    }
  }
  // fill ROS message
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.push_back("hand_motor_joint");
  traj.points.resize(1);
  traj.points[0].positions.resize(1);
  traj.points[0].positions[0] = 0.5;
  traj.points[0].velocities.resize(1);
  traj.points[0].velocities[0] = 0.0;
  traj.points[0].effort.resize(1);
  traj.points[0].effort[0] = 0.1;
  traj.points[0].time_from_start = ros::Duration(3.0);
  // publish ROS message
  pub.publish(traj);
  ros::spinOnce();
}

// ドアをロボットが開ける関数
void open_door() {
    ROS_INFO("Open door");
    ros::Rate loop_rate(10);
    ROS_INFO("range: %.1f", range_ahead);
    float base_to_laser = 0.16;
    float hand_to_base = 0.59;
    float arm_to_door = base_to_laser+range_ahead-hand_to_base + 0.05;
    ROS_INFO("arm_to_door: %.1f", arm_to_door);
    odom_control(arm_to_door, 0.0, 0.0);
    arm_lift_joint = 0.0;
    arm_flex_joint = 0.0;
    arm_roll_joint = -1.57;
    wrist_flex_joint = 0.0;
    wrist_roll_joint = 0.0;
    head_pan_joint = 0.0;
    head_tilt_joint = 0.0;
    arm_control(arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint);
    ros::Duration(0.5).sleep();
    head_control(head_pan_joint, head_tilt_joint);
    ros::Duration(0.5).sleep();
    grasp(2.0);
    ros::Duration(2.0).sleep();
    arm_lift_joint = 0.67;
    arm_control(arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint);
    ros::Duration(0.5).sleep();
    odom_control(2.0-arm_to_door, 0.0, M_PI);
    ros::spinOnce();
    arm_lift_joint = 0.36154560720863893;
    arm_control(arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint);
    ros::Duration(0.5).sleep();
    open_hand();
    ros::Duration(0.5).sleep();
    loop_rate.sleep( );
}

// 手を挙げている人を検出する関数
void find_person() {
    ROS_INFO("Find person");
    FILE *fp2 = popen("roslaunch yolov8_ros yolov8_with_tf.launch weight_name:=person.pt", "r");
    ros::Duration(5.0).sleep();
    target_name = "hand";
    position = get_position();
    FILE *kill_fp = popen("rosnode kill /yolov8_ros/bbox_to_tf /yolov8_ros/rviz /yolov8_ros/yolov8", "r");
    ros::Duration(0.5).sleep();
    if(position(1) > 0) is_right = false;
    else if(position(1) < 0) is_right = true;
    if(find_object) speak_word = "見つかりました";
    else speak_word = "見つかりませんでした";
    speak(speak_word);
    ros::Duration(1.0).sleep();
    
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_laser = nh.subscribe("/hsrb/base_scan", 10, callback_laser);
    ros::Subscriber sub_odom = nh.subscribe("/hsrb/odom", 10, callback_odom);

    //start
    while(ros::ok) {
        ros::spinOnce();
        if(flag1 and flag2) break;
        ros::spinOnce();
    }

    set_initial_pose();
    
    ros::spinOnce();
    speak("ドアを開けてください");
    ros::Duration(1.0).sleep();

    // door open
    ros::spinOnce();
    door_open(1.0);

    // arm_lift_joint = 0.67;
    // arm_flex_joint = -1.6395805478452758;
    // arm_roll_joint = 0.0;
    // wrist_flex_joint = -1.4240975766538186;
    // wrist_roll_joint = 1.5421191378316852;
    // head_pan_joint = -0.00035967552915350254;
    // head_tilt_joint = -0.6763496769555847;
    // arm_control(arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint);
    // ros::Duration(0.5).sleep();
    // head_control(head_pan_joint, head_tilt_joint);
    // ros::Duration(0.5).sleep();
    // open_door();

    // navigation
    ros::spinOnce();
    nav("table");

    // // find person
    // speak_word = "注文する人は手を広げて挙げてください";
    // speak(speak_word);
    // ros::Duration(1.0).sleep();
    // find_person();

    // if (is_right) nav("table_right");
    // else nav("table_left");

    speak_word = "僕が訪ね終わったら１秒後に発言してください";
    speak(speak_word);
    ros::Duration(1.0).sleep();

    // ask order
    ros::spinOnce();
    target_name = ask_order();

    if(target_name == "カップヌードル") target_frame = "ramen";
    else if(target_name == "お茶") target_frame = "green_tea";
    else if(target_name == "ポテトチップス") target_frame = "potato";

    // navigation
    ros::spinOnce();
    nav("shelf");

    ros::spinOnce();
    speak_word = "  キッチンに到着しました";
    speak(speak_word);
    ros::Duration(1.0).sleep();

    // object recognition
    recognition_pose();
    ros::spinOnce();
    target_position = object_recognition(target_name);
    ros::spinOnce();

    if(find_object) {
        speak_word = target_name + "を掴みます";
        speak(speak_word);
        ros::Duration(1.0).sleep();

        // grasp object
        ros::spinOnce();
        open_hand();
        ros::spinOnce();
        move_to_target_object(target_position, 0.33);
        ros::spinOnce();
        grasp(0.8);

        odom_control(0.0, 0.2, -M_PI/2);
        set_initial_pose();
        odom_control(0.0, 0.09, M_PI/2);
    }

    // navigation
    ros::spinOnce();
    nav("table");
    // if(is_right) nav("table_right");
    // else nav("table_left");

    ros::spinOnce();
    speak_word = "お待たせしました";
    speak(speak_word);
    ros::Duration(1.0).sleep();

    // placement recognition
    ros::spinOnce();
    recognition_pose();
    ros::spinOnce();
    target_position = placement_recognition();


    // put object
    if(find_object) {
        ros::spinOnce();
        speak_word = target_name + "を置きます";
        speak(speak_word);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        move_to_target_object(target_position, 0.0);
        ros::spinOnce();
        open_hand();
        odom_control(0.0, 0.23, -M_PI/2);
        set_initial_pose();
        odom_control(0.0, 0.09, M_PI/2);
    }

    // navigation
    ros::spinOnce();
    nav("exit");

    ros::spinOnce();
    speak_word = "出口に到着しました。ドアを開けてください";
    speak(speak_word);

    // door open
    ros::spinOnce();
    door_open(-1.2);

    // end
    ros::spin();
    return 0;
}
