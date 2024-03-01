// Include ROS packages
#include <ros/ros.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>

const uint8_t LEFT = 0;
const uint8_t CENTER = 1;
const uint8_t RIGHT = 2;

class Bumper
{
private:
    ros::NodeHandle n_;
    ros::Subscriber bumper_sub_;
    
    // send command to the wheels if bumper was pressed
    ros::Publisher twist_pub_;
private:    
    float max_vel_x;
    float min_vel_x;
    float max_vel_theta;
    float min_vel_theta;
public:
    Bumper()
    {
        n_.param("max_vel_x", max_vel_x, 0.2f);
        n_.param("min_vel_x", min_vel_x, -0.2f);
        n_.param("max_vel_theta", max_vel_theta, 1.0f);
        n_.param("min_vel_theta", min_vel_theta, -1.0f);
        
        bumper_sub_ = n_.subscribe("robot_4/mobile_base/events/bumper", 1000, &Bumper::bumperCallback, this);
        twist_pub_ = n_.advertise<geometry_msgs::Twist>("/robot_4/mobile_base/commands/velocity", 1000);
    }
    
    void bumperCallback(const kobuki_msgs::BumperEvent& bumper)
    {
        std::string output = "Collision is on the ";
        geometry_msgs::Twist twist;
        
        if (bumper.state == bumper.PRESSED)
        {
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
            
            twist_pub_.publish(twist);
        }
    }
};


// =============== Main function =================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bumper_node");
  
  Bumper BObject;
  
  ros::spin();
  
  return 0;
}



