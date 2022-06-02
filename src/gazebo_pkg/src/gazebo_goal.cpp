#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std::chrono_literals;
using std::placeholders::_1;

class GazeboGoal: public rclcpp::Node{
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_w;

        rclcpp::TimerBase::SharedPtr timer;

        double x_goal,y_goal,v,w,tol,K,v_max,w_max;

        geometry_msgs::msg::Twist msg;

        void callback_pose(const nav_msgs::msg::Odometry::SharedPtr odom);
    public:
        GazeboGoal(): Node("gazebo_goal"){
            x_goal = 1;
            y_goal = 10;
            
            K = 10;
        
            tol = 0.1;

            v_max = 1;
            w_max = 1;

            RCLCPP_INFO(this->get_logger(),"Starting Gazebo_Goal!");
            pub_w = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel",10);
            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/demo/odom", 10, std::bind(&GazeboGoal::callback_pose, this, _1));
            timer = this->create_wall_timer(10ms, std::bind(&GazeboGoal::publish, this));
        }

        void publish();

};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<GazeboGoal>());
    rclcpp::shutdown();
    return 0;
}

void GazeboGoal::callback_pose(const nav_msgs::msg::Odometry::SharedPtr odom){
    double ex,ey, et, theta, roll, pitch, yaw;
    tf2::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    
    m.getRPY(roll, pitch, yaw);

    ex = x_goal - odom->pose.pose.position.x;
    ey = y_goal - odom->pose.pose.position.y;
    theta = atan2(ey,ex);
    et = theta - yaw;

    if(abs(ex)<tol && abs(ey)<tol){
        v = 0;
        w = 0;
    }
    else{
        v = sqrt(pow(ex,2)+pow(ey,2));
        w = K*et;
    if(abs(v)>v_max){
        if(v>0)
            v = v_max;
        else    
            v = -v_max;}
    if(abs(w)>w_max){
        if(w > 0)
            w = w_max;
        else    
            w = -w_max;}
}

    msg.linear.x = v;
    msg.angular.z = w;

    std::cout<<"x = "<<odom->pose.pose.position.x<<" y = "<<odom->pose.pose.position.y<<" v = "<<v<<" w = "<<w<<std::endl;
}


void GazeboGoal::publish(){
    pub_w->publish(msg);
}
