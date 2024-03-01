#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <algorithm>

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

class Homing
{

 private:
    double _rho;
    double _alpha;
    bool _hasGoal;

    ros::NodeHandle _nh;

    ros::Subscriber _odom_sub;
    ros::Subscriber _goal_sub;

    ros::Publisher _path_odom_pub;
    ros::Publisher _rho_pub;
    ros::Publisher _alpha_pub;
    ros::Publisher _twist_pub;

    tf::TransformListener _listener;

    geometry_msgs::PoseStamped _goal_map;
    geometry_msgs::PoseStamped _goal_base_link;
    geometry_msgs::PoseStamped _pose_odom;
    
    nav_msgs::Path path_msg;
 public:
    Homing()
    {
        _goal_sub = _nh.subscribe("/robot_4/goal", 100, &Homing::goalCallback, this);
        _odom_sub = _nh.subscribe("/robot_4/odom", 100, &Homing::odomCallback, this);

        _twist_pub = _nh.advertise<geometry_msgs::Twist>("/robot_4/mobile_base/commands/velocity", 100);
        _path_odom_pub = _nh.advertise<nav_msgs::Path>("visualization_marker", 100);
        _alpha_pub = _nh.advertise<std_msgs::Float64>("/robot_4/alpha", 100);
        _rho_pub = _nh.advertise<std_msgs::Float64>("/robot_4/rho", 100);
        
        _hasGoal = false;
        
        _goal_map.header.frame_id = "/map";
        path_msg.header.frame_id = "/robot_4/odom";
    }

    void homingControl(double k_rho, double k_alpha)
    {   
        geometry_msgs::Twist cmd;
        double v     = 0.0;
        double omega = 0.0;

        if (_hasGoal)
        {
            // Calculate v and omega
            v = k_rho * _rho;
            omega = -k_alpha * _alpha;
            
            if (std::abs(v) > 0.2 )
                v = sgn<double>(v) * 0.2;
            
            if (std::abs(omega) > 1.0 )
                omega = sgn<double>(omega) * 1.0;
            
        }

        // Send velocity command
        cmd.linear.x = v;
        cmd.angular.z = omega;
        
        _twist_pub.publish(cmd);
    }

 private:
    void odomCallback(const nav_msgs::Odometry& msg)
    {
        if (!_hasGoal) return;  // No goal, don't do anything;

        // Convert goal from map frame to robotcentric frame
        _listener.waitForTransform("/robot_4/base_link", "/map", ros::Time::now(), ros::Duration(1, 0.0));
        _listener.transformPose("/robot_4/base_link", ros::Time(0), _goal_map, "/map", _goal_base_link);
        
        double x_goal = _goal_base_link.pose.position.x;
        double y_goal = _goal_base_link.pose.position.y;
        
        // Calculate error in position and orientation towards goal, given in robotcentric coordinates
        _alpha = -std::atan2(y_goal, x_goal);
        _rho = std::sqrt(std::pow(x_goal, 2) + std::pow(y_goal, 2));
        
        // Accept goal as reached if within some tolerance
        if (_rho < 0.1)
        {
            _hasGoal = false;
            ROS_INFO("Reached home");
        }

        // Publish errors for visualization in rqt_plot
        std_msgs::Float64 alpha_msg, rho_msg;
        alpha_msg.data = _alpha;
        rho_msg.data = _rho;
        
        _alpha_pub.publish(alpha_msg);
        _rho_pub.publish(rho_msg);

        // Publish odom path for visualization in rviz
        geometry_msgs::PoseStamped current_pos_odom;
        current_pos_odom.header.frame_id = "/robot_4/odom";
        current_pos_odom.pose = msg.pose.pose;
        path_msg.poses.push_back(current_pos_odom);
        
        _path_odom_pub.publish(path_msg);
        
    }

    void goalCallback(const geometry_msgs::PoseStamped& msg)
    {
        _goal_map.pose = msg.pose;
        _hasGoal = true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "homing_node");
    Homing h;
    ros::Rate r(10);

    while (ros::ok())
    {
        ros::spinOnce();
        
        h.homingControl(1.0, 1.0);
        
        r.sleep();
    }
    return (0);
}
