// Include ROS packages
#include <ros/ros.h>
#include "std_msgs/Char.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <math.h>

#define _USE_MATH_DEFINES

// Bumper States
#define LEFT 0
#define CENTER 1
#define RIGHT 2
#define RELEASED 0
#define PRESSED 1

class Teleop
{
private:
    ros::NodeHandle n_;
    
    ros::Subscriber bumper_sub_;
    ros::Subscriber key_sub_;
    
    ros::Publisher twist_pub_;
private:    
    float max_vel_x;
    float min_vel_x;
    float max_vel_theta;
    float min_vel_theta;
    
    uint8_t bumper_state;
public:
    Teleop()
    {
        n_.param("max_vel_x", max_vel_x, 0.2f);
        n_.param("min_vel_x", min_vel_x, -0.2f);
        n_.param("max_vel_theta", max_vel_theta, 1.0f);
        n_.param("min_vel_theta", min_vel_theta, -1.0f);
        
        bumper_state = RELEASED;
        
        key_sub_ = n_.subscribe("arrow_keys", 1000, &Teleop::keyCallback, this);
        bumper_sub_ = n_.subscribe("robot_4/mobile_base/events/bumper", 1000, &Teleop::bumperCallback, this);
        twist_pub_ = n_.advertise<geometry_msgs::Twist>("/robot_4/mobile_base/commands/velocity", 1000);
    }
    
    void keyCallback(const std_msgs::Char::ConstPtr& msg)
    { // Set new direction to the robot depending on the arrow pressed
        if (bumper_state == RELEASED)
        {
            std::string output; 
            geometry_msgs::Twist twist;
            
            // twist.linear.x - speed v;
            // twist.angular.z - rotation rate (omega w);
            
            switch(msg->data) 
            {
            case 65: // arrow up
                output = "Up";
                twist.linear.x = max_vel_x; // arrow up => drive forward, positive speed
                break;
                
                
            case 66: // arrow down
                output = "Down";
                twist.linear.x = min_vel_x; // arrow down =>  drive backwords, negative speed
                break;
                
                
            case 67: // arrow right
                output = "Right";
                twist.angular.z = min_vel_theta; // arrow right => clockwise rotation, negative angle changing
                break;
                
                
            case 68: // arrow left
                output = "Left";
                twist.angular.z = max_vel_theta; // arrow right => counterclockwise rotation, positive angle changing
                break;
            
            default:
                output = "Unsuppported button. Please, use arrows.";
            }
            
            ROS_INFO("Text: [%s]", output.c_str());
            
            //create publisher to send updated info to robot
            twist_pub_.publish(twist); 
        }
        else
        {
            return;
        }
    }
     
    void bumperCallback(const kobuki_msgs::BumperEvent& bumper)
    {
        std::string output = "Collision is on the ";
        geometry_msgs::Twist twist;
        
        if (bumper.state == PRESSED)
        {
            bumper_state = PRESSED;
            
            switch(bumper.bumper)
            {
        
            case LEFT:
                output += "LEFT";
                
                // drive back and right
                twist.linear.x = min_vel_x;
                twist.angular.z = min_vel_theta;
                
                break;
            case RIGHT:
                output += "RIGHT";
                
                // drive back and left
                twist.linear.x = min_vel_x;
                twist.angular.z = max_vel_theta;
                
                break;
            case CENTER:
                output += "CENTER";
                
                // drive back
                twist.linear.x = min_vel_x;
                
                break;
            default:
                output = "INVALID BUMPER MESSAGE";
            }
            
            ROS_WARN("%s", output.c_str());

            // we have to set a duration to the robot so it can drive back
            ros::Time begin = ros::Time::now();
            ros::Duration drive_back_time(1.5);
            
            while(ros::Time::now() < begin + drive_back_time)
            {
                twist_pub_.publish(twist);
            }
        }
        else
        {
            bumper_state = RELEASED;
        }
    }
};



// =============== Main function =================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_node");
  
    Teleop bot_controls;
    
    ros::spin();
  
    return 0;
}
