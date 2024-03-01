#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <math.h>

class Homing
{
private:
    float max_vel_x;
    float min_vel_x;
    float max_vel_theta;
    float min_vel_theta;

 private:
    ros::NodeHandle n_;

    ros::Subscriber _odom_sub;
    ros::Subscriber _goal_sub;

    ros::Publisher _twist_pub;
    ros::Publisher marker_pub_;
    
    tf::TransformListener listener;

    
private:
    geometry_msgs::PoseStamped current_pos_odom;
    geometry_msgs::PoseStamped current_pos_map;

    nav_msgs::Path path_msg;
    
 public:
    Homing()
    {
        n_.param("max_vel_x", max_vel_x, 0.2f);
        n_.param("min_vel_x", min_vel_x, -0.2f);
        n_.param("max_vel_theta", max_vel_theta, 1.0f);
        n_.param("min_vel_theta", min_vel_theta, -1.0f);

        _odom_sub  = n_.subscribe("/robot_4/odom", 100, &Homing::odomCallback, this);
        _goal_sub  = n_.subscribe("/robot_4/goal", 100, &Homing::goalCallback, this);
        _twist_pub = n_.advertise<geometry_msgs::Twist>("/robot_4/mobile_base/commands/velocity", 100);
        marker_pub_ = n_.advertise<nav_msgs::Path>("visualization_marker", 100);
        
        path_msg.header.frame_id = "/robot_4/odom";
    }
    
private:
    int sgn(float val)
    {
        //return ((0.0 < val) - (val < 0.0));
        return (val > 0) ? 1 : ((val < 0) ? -1 : 0);
    }

 private:
    void forward(double distance, double velocity)
    {
        // For tracking elapsed time
        ros::Time begin = ros::Time::now();
        ros::Duration elapsed_time;
	double time;

        // calculate forward duration
        time = distance / velocity;
        
        // set velocity
        geometry_msgs::Twist twist;
        twist.linear.x = velocity;
        
        // Rated loop operating at 10 Hz
        ros::Rate loop_rate(10);

        while (elapsed_time.toSec() < time)
        {
            // Process callbacks once
            ros::spinOnce();

            // send velocity commands
            _twist_pub.publish(twist);
            
            // Suspend thread according to the defined rate
            loop_rate.sleep();

            // Keep track of elapsed time
            ros::Time end = ros::Time::now();
            elapsed_time  = end - begin;
        }
    }

    void turn(double angle, double turnrate)
    {
        // For tracking elapsed time
        ros::Time begin = ros::Time::now();
        ros::Duration elapsed_time;
        double time;

        // calculate turning duration
	time = std::abs(angle) / turnrate;
        
        // set rotation speed
        geometry_msgs::Twist twist;
        twist.angular.z = turnrate * sgn(angle);
        
        // Rated loop operating at 10 Hz
        ros::Rate loop_rate(10);
        while (elapsed_time.toSec() < time)
        {
            // Process callbacks once
            ros::spinOnce();

            // send velocity commands
            _twist_pub.publish(twist);
            
            // Suspend thread according to the defined rate
            loop_rate.sleep();

            // Keep track of elapsed time
            ros::Time end = ros::Time::now();
            elapsed_time  = end - begin;
        }
    }

private:
    // get start position (odom)
    void odomCallback(const nav_msgs::Odometry& msg)
    {
        current_pos_odom.header.frame_id = "/robot_4/odom";
        current_pos_odom.pose = msg.pose.pose;

        path_msg.poses.push_back(current_pos_odom);
        marker_pub_.publish(path_msg);
    }

    // get goal position from rviz
    void goalCallback(const geometry_msgs::PoseStamped& msg)
    {
        listener.waitForTransform("/map", "/robot_4/odom", ros::Time::now(), ros::Duration(1, 0.0));
        listener.transformPose("/map", ros::Time(0), current_pos_odom, "/map", current_pos_map);
        
        // goal position coordinates
        float x = msg.pose.position.x;
        float y = msg.pose.position.y;
        
        // current position coordinates
        float prev_x = current_pos_map.pose.position.x; // x[t-1]
        float prev_y = current_pos_map.pose.position.y; // y[t-1]
        float prev_theta = tf::getYaw(current_pos_map.pose.orientation); // theta[t-1]
        
        // calculate translation and rotations to goal position
        float theta = tf::getYaw(msg.pose.orientation);
        
        float delta_rot1 = -prev_theta + std::atan2(y - prev_y, x - prev_x);
        float delta_trans = std::sqrt(std::pow(x - prev_x, 2) + std::pow(y - prev_y, 2));
        float delta_rot2 = theta - prev_theta - delta_rot1;
        
        // send new deltas to the turtlebot
        turn(delta_rot1, max_vel_theta);
        forward(delta_trans, max_vel_x);
        turn(delta_rot2, max_vel_theta);
        
        
    }

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "homing_sequential_node");
    Homing h;

    ros::spin();

    return (0);
}

