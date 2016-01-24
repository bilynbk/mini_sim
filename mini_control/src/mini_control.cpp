#include "ros/ros.h" 
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

using namespace std;

#define MINI_1_5

#ifdef MINI_1_5
// mini1.5
const int JOINT_NUM = 8;
#else
// mini1.0
const int JOINT_NUM = 6;  
#endif

std_msgs::Float64 tmp_joint[JOINT_NUM];
std_msgs::Float64 target_joint[JOINT_NUM];

double pos_x, pos_y, pos_z;

void monitorJointState(const sensor_msgs::JointState::ConstPtr& jointstate)
{
  for (int i=0; i < JOINT_NUM; i++) {
    tmp_joint[i].data = jointstate->position[i];
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mini_control"); 
  // initでROSを初期化して、my_teleopという名前をノードにつける                        
  // 同じ名前のノードが複数あってはいけないので、ユニークな名前をつける

  ros::NodeHandle nh;
  // ノードハンドラの作成。ハンドラは必要になったら起動される。

  ros::Publisher  pub_joint[JOINT_NUM];
  // パブリッシャの作成。トピックに対してデータを送信。

  ros::Subscriber sub_joints, sub_sensor;
  // サブスクライバの作成

  ros::Rate rate(10);
  // ループの頻度を設定。この場合は10Hz、1秒間に10回数、1ループ100ms。

  std_msgs::Float64 target_joint[JOINT_NUM];

  #ifdef MINI_1_5
  pub_joint[0] = nh.advertise<std_msgs::Float64>("/mini/torso_upper_joint_position_controller/command", 100);
  pub_joint[1] = nh.advertise<std_msgs::Float64>("/mini/head_joint_position_controller/command", 100);
  pub_joint[2] = nh.advertise<std_msgs::Float64>("/mini/shoulder_joint_position_controller/command", 100);
  pub_joint[3] = nh.advertise<std_msgs::Float64>("/mini/elbow_joint_position_controller/command", 100);
  pub_joint[4] = nh.advertise<std_msgs::Float64>("/mini/wrist_pitch_joint_position_controller/command", 100);
  pub_joint[5] = nh.advertise<std_msgs::Float64>("/mini/wrist_roll_joint_position_controller/command", 100);
  pub_joint[6] = nh.advertise<std_msgs::Float64>("/mini/finger_right_joint_position_controller/command", 100);
  pub_joint[7] = nh.advertise<std_msgs::Float64>("/mini/finger_left_joint_position_controller/command", 100);
  #else
  pub_joint[0] = nh.advertise<std_msgs::Float64>("/mini/torso_upper_joint_position_controller/command", 100);
  pub_joint[1] = nh.advertise<std_msgs::Float64>("/mini/head_joint_position_controller/command", 100);
  pub_joint[2] = nh.advertise<std_msgs::Float64>("/mini/shoulder_joint_position_controller/command", 100);
  pub_joint[3] = nh.advertise<std_msgs::Float64>("/mini/wrist_pitch_joint_position_controller/command", 100);
  pub_joint[4] = nh.advertise<std_msgs::Float64>("/mini/finger_right_joint_position_controller/command", 100);
  pub_joint[5] = nh.advertise<std_msgs::Float64>("/mini/finger_left_joint_position_controller/command", 100);
  #endif

  sub_joints = nh.subscribe("/mini/joint_states", 100, monitorJointState);

  for (int i=0; i < JOINT_NUM;  i++) {
    target_joint[i].data = 0;
  }

  int loop = 0;
  while (ros::ok()) { // このノードが使える間は無限ループ
    char key;  // 入力キーの値                                                                                           

    #ifdef MINI_1_5
     ROS_INFO("Input a,s,d,f,g,h,j,k,l,;, b,n,m,c,.,x");
    #else
     ROS_INFO("[Input] a,s,d,f,g,h,j,k,l,;");
    #endif

    cin >> key;
    cout << key << endl;

    switch (key) {
    #ifdef MINI_1_5
    case 'h': target_joint[0].data  +=  0.02; break;
    case 'g': target_joint[0].data  -=  0.02; break;
    case 'j': target_joint[1].data  +=  5 * M_PI/180.0; break;
    case 'f': target_joint[1].data  -=  5 * M_PI/180.0; break;
    case 'k': target_joint[2].data  +=  5 * M_PI/180.0; break;
    case 'd': target_joint[2].data  -=  5 * M_PI/180.0; break;
    case 'l': target_joint[3].data  +=  5 * M_PI/180.0; break;
    case 's': target_joint[3].data  -=  5 * M_PI/180.0; break;
    case ';': target_joint[4].data  +=  5 * M_PI/180.0; break;
    case 'a': target_joint[4].data  -=  5 * M_PI/180.0; break;
    case 'n': target_joint[5].data  +=  5 * M_PI/180.0; break;
    case 'b': target_joint[5].data  -=  5 * M_PI/180.0; break;
    case 'm': target_joint[6].data  +=  5 * M_PI/180.0; break;
    case 'c': target_joint[6].data  -=  5 * M_PI/180.0; break;
    case ',': target_joint[7].data  +=  5 * M_PI/180.0; break;
    case 'x': target_joint[7].data  -=  5 * M_PI/180.0; break;
    default: ROS_INFO("Input a,s,d,f,g,h,j,k,l,;, b,n,m,c,.,x");
    #else
    case 'h': target_joint[0].data  +=  0.02; break;
    case 'g': target_joint[0].data  -=  0.02; break;
    case 'j': target_joint[1].data  +=  5 * M_PI/180.0; break;
    case 'f': target_joint[1].data  -=  5 * M_PI/180.0; break;
    case 'k': target_joint[2].data  +=  5 * M_PI/180.0; break;
    case 'd': target_joint[2].data  -=  5 * M_PI/180.0; break;
    case 'l': target_joint[3].data  +=  5 * M_PI/180.0; break;
    case 's': target_joint[3].data  -=  5 * M_PI/180.0; break;
    case ';': target_joint[4].data  +=  5 * M_PI/180.0; break;
    case 'a': target_joint[4].data  -=  5 * M_PI/180.0; break;
    default: ROS_INFO("Input a,s,d,f,g,h,j,k,l,;");
    #endif
    }

    #ifdef MINI_1_5
      target_joint[7].data = - target_joint[6].data;
    #else
      target_joint[5].data = - target_joint[4].data;
    #endif

    ROS_INFO("Taget:");
    for (int i=0; i < JOINT_NUM; i++) {
      pub_joint[i].publish(target_joint[i]); // 角度を送信    
      ROS_INFO("Joint[%d]=%f ", i, target_joint[i].data);
    }

    usleep(1000*1000);
    ros::spinOnce(); // コールバック関数を呼ぶ
    ROS_INFO("Tmp:   ");
    for (int i=0; i< JOINT_NUM; i++) {
      ROS_INFO("Joint[%d]=%f ", i, tmp_joint[i].data);
    }
    //rate.sleep();     // 指定した周期でループするよう寝て待つ
  }
  
  return 0;
}
