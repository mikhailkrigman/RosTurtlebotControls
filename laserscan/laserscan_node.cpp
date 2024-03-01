// Include ROS packages
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>


#include <cmath>

#define _USE_MATH_DEFINES

const float SENSOR_OFFSET = 0.25f; // distance between sensor and turtlebot rand (20cm)
const float D_STOP = 0.3f + SENSOR_OFFSET;
const float D_TURN = 0.4f + SENSOR_OFFSET;
const float D_SAFE = 0.7f + SENSOR_OFFSET;

const float MAX_V = 0.2;
const float MSX_OMEGA = 1.0;

class LaserScan
{
private:
    float max_vel_x;
    float min_vel_x;
    float max_vel_theta;
    float min_vel_theta;
    
    float previous_omega;
private:
    ros::NodeHandle n_;
    
    ros::Subscriber scan_sub_;
    
    ros::Publisher marker_pub_;
    ros::Publisher twist_pub_;
    
public:
    LaserScan()
    {
        n_.param("max_vel_x", max_vel_x, 0.2f);
        n_.param("min_vel_x", min_vel_x, -0.2f);
        n_.param("max_vel_theta", max_vel_theta, 1.0f);
        n_.param("min_vel_theta", min_vel_theta, -1.0f);
        
        previous_omega = 0.0; // to the start robot has no rotation speed
        
        scan_sub_ = n_.subscribe("robot_4/scan", 1000, &LaserScan::scanCallback, this);
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
        twist_pub_ = n_.advertise<geometry_msgs::Twist>("/robot_4/mobile_base/commands/velocity", 1000);

    }

private:
    int sgn(float val)
    {
        //return ((0.0 < val) - (val < 0.0));
        return (val > 0) ? 1 : ((val < 0) ? -1 : 0);
    }
    
    void avoidObstacle(float dmin, float phi)
    {
        geometry_msgs::Twist twist;
        float v; // linear speed
        float omega; // rotation speed;
        
        // set linear speed depending on the distance to the nearest wall
        if (dmin < D_STOP)
        {
            v = 0.0;
        }
        else if (D_STOP <= dmin && dmin <= D_SAFE)
        {
            v = max_vel_x * ((dmin - D_STOP) / (D_SAFE - D_STOP));
        }
        else
        {
            v = max_vel_x;
        }
        
        // set linear speed depending on the distance to the nearest wall
        if (dmin < D_TURN)
        {
            omega = sgn(this->previous_omega) * max_vel_theta;
        }
        else if (D_TURN <= dmin && dmin <= D_SAFE)
        {
            omega = max_vel_theta * ((D_SAFE - dmin) / (D_SAFE - D_TURN));
            ROS_INFO("OMEGA ABS: %f", omega);
            ROS_INFO("-SGN(Phi), %d", -sgn(phi));
            omega *= -sgn(phi);
            ROS_INFO("OMEGA BETWEEN D_SAFE and D_STOP: %f", omega);
            
        }
        else
        {
            omega = 0.0;
        }
        this->previous_omega = omega; // store rotation speed
        
        twist.linear.x = v;
        twist.angular.z = omega;
        
        twist_pub_.publish(twist);
    }
    
    void scanCallback(const sensor_msgs::LaserScan& data)
    {
        // read data from sensor and calculate distance to the nearest wall
        
        int ranges_len = (data.angle_max - data.angle_min) / data.angle_increment;
        float dmin = data.ranges[0];
        int j = 0;
        
        for(int i = 0; i < ranges_len; i++)
        {   
            
            if (!std::isnan(data.ranges[i])) // check if measured data is number
            {
                // if first selected value was NaN redefine it with first incoming float data
                if (std::isnan(dmin))
                    dmin = data.ranges[i];
                 
                // find min element and min index in measured values
                if (data.ranges[i] < dmin)
                {
                    dmin = data.ranges[i];
                    j = i;
                }
                
            }
        }
        
        float phi = data.angle_min + j * data.angle_increment;
        
        float phi_deg = phi * (180.0 / M_PI);
        ROS_INFO("dmin: %f, phi: %f", dmin, phi_deg);
        ROS_INFO("Prev_Omega: %f", this->previous_omega);
        
        // draw line to the nearest wall in rviz
        
        
        geometry_msgs::Point start, end;
        
        // start point is robot position
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0; // 2D-Simulation
        
        // end point are coordinates of nearest wall
        end.x = (double)(dmin) * std::cos((double)(phi));
        end.y = (double)(dmin) * std::sin((double)(phi));
        end.z = 0; // 2D-Simulation
        
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "/robot_4/base_link";
        line_list.scale.x = 0.05;
        //line_list.header.stamp = ros::Time::now();
        
        line_list.color.r = 1.0f;
        line_list.color.a = 1.0;
        
        line_list.points.push_back(start);
        line_list.points.push_back(end);
        
        marker_pub_.publish(line_list);
        avoidObstacle(dmin, phi);
    }
};

// =============== Main function =================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_node");
  
  LaserScan LSObject;
  
  ros::spin();
  
  return 0;
}



