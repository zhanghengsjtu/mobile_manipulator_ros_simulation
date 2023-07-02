
// 程序的作用是根据gazebo中的关节位置接口，模拟出一个cmd_vel速度接口，便于和建图导航等模块对接

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "cmath"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"

using namespace std;

class agv_vel_controller
{
public:
    agv_vel_controller()
    {
        joint_names_ = { "x_joint", "y_joint", "angle_joint"};
        current_joint_values_.resize(joint_names_.size());

        joint_state_sub_ = nh_.subscribe("joint_states", 1, &agv_vel_controller::jointStatesCallback, this);
        joint_state_flag_ = false;
        while(joint_state_flag_ == false);

        joint_pos_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("agv_joint_group_controller/command", 1000);
        twist_sub_ = nh_.subscribe("cmd_vel", 1, &agv_vel_controller::twistCallback, this);
        pre_time_ = ros::Time::now();
    }

private:
    ros::NodeHandle nh_;
    // 机器人状态相关的变量
    std::vector<string> joint_names_;
    std::vector<double> current_joint_values_;
    ros::Subscriber joint_state_sub_;
    bool joint_state_flag_;

    // 机器人关节速度和位置控制相关的变量
    ros::Publisher joint_pos_pub_;
    ros::Subscriber twist_sub_;
    ros::Time pre_time_;

    // 回调函数
    void twistCallback(const geometry_msgs::TwistConstPtr& twistPtr);
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states);
};

void agv_vel_controller::twistCallback(const geometry_msgs::TwistConstPtr& twistPtr)
{
    double delta_t = (ros::Time::now() - pre_time_).toSec();
    // 防止长时间没有速度输入，突然间来了一个速度输入，机器人会不正常的运动
    if(delta_t > 0.5)  
    {
        // ROS_ERROR("The robot will move!");
        delta_t = 0;
    }

    double cos_angle = cos(current_joint_values_[2]);
    double sin_angle = sin(current_joint_values_[2]);
    std::vector<double> global_vel(3,0);
    global_vel[0] = cos_angle * twistPtr->linear.x - sin_angle * twistPtr->linear.y;
    global_vel[1] = sin_angle * twistPtr->linear.x + cos_angle * twistPtr->linear.y;
    global_vel[2] = twistPtr->angular.z;

    std_msgs::Float64MultiArray joint_pos_msgs;
    joint_pos_msgs.data.resize(3);
    for(int i = 0; i < 3; ++i)
    {
        joint_pos_msgs.data[i] = current_joint_values_[i] + global_vel[i] * delta_t;
    }
    joint_pos_pub_.publish(joint_pos_msgs);
    pre_time_ = ros::Time::now();
}

void agv_vel_controller::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_state_ptr)
{
    for(int i = 0; i < joint_state_ptr->name.size(); ++i)
    {
        for(int j = 0; j < joint_names_.size(); ++j)
        {
            if(joint_names_[j] == joint_state_ptr->name[i])
            {
                current_joint_values_[j] = joint_state_ptr->position[i];
                break;
            }
        }
    }
    if(joint_state_flag_ == false) joint_state_flag_ = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "agv_vel_controller");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    agv_vel_controller controller;

    ros::waitForShutdown();
    return 0;
}