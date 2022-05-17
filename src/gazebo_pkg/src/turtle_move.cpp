#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "termios.h"
#include <unistd.h>

#include <iostream>
using namespace std::chrono_literals;

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_SPACE 0x20

int kfd = 0;
char c; 
struct termios cooked,raw;


class TurtleMove: public rclcpp::Node{
    private:
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
        rclcpp::TimerBase::SharedPtr timer;

        double linear,angular;
    public:
        TurtleMove()
        : Node("turtle_move"){
            pub = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel",10);
            timer = this->create_wall_timer(10ms, std::bind(&TurtleMove::publish, this));

            tcgetattr(kfd, &cooked);
  	        memcpy(&raw, &cooked, sizeof(struct termios));
  	        raw.c_lflag &=~ (ICANON | ECHO);
  	        // Setting a new line, then end of file                         
 	        raw.c_cc[VEOL] = 1;
 	        raw.c_cc[VEOF] = 2;
 	        tcsetattr(kfd, TCSANOW, &raw);

            puts("Reading from keyboard");
  	        puts("---------------------------");
  	        puts("Use arrow keys to move Hulk.");
        }  

        void publish(){
          geometry_msgs::msg::Twist msg;
          if(read(kfd, &c, 1) < 0){
            perror("read():");
            exit(-1);
         }
            switch(c){
	            case KEYCODE_RIGHT:
		            RCLCPP_INFO(this->get_logger(),"Move to right");
		            angular=-1;
		            linear=0.0;
	            break;
	            case KEYCODE_LEFT:
		            RCLCPP_INFO(this->get_logger(),"Move to left");
		            angular=1;
		            linear=0.0;
	            break;
	            case KEYCODE_UP:
		            RCLCPP_INFO(this->get_logger(),"Move to up");
		            linear=1;
		            angular=0.0;
	            break;
	            case KEYCODE_DOWN:
		            RCLCPP_INFO(this->get_logger(),"Move to down");
		            linear=-1;
		            angular=0.0;
	            break;
	            case KEYCODE_SPACE: 
		            RCLCPP_INFO(this->get_logger(),"Stop");
		            linear=0;	
		            angular=0;	
	            break;
 
	            }
             msg.linear.x = linear;
             msg.angular.z = angular;

             pub->publish(msg);
        }

        ~TurtleMove(){
            tcsetattr(kfd, TCSADRAIN, &cooked);
            //To fix this part. Allow the terminal show what you write on it.
        }


};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TurtleMove>());
    rclcpp::shutdown();
    return 0;
} 
 