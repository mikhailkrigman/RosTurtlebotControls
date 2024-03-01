// Below is the version that finds lookahead point on the closest path segment to the robot
// The previous one was looking for the nearest point to the first segment and after
// to the closest point on segment step by step

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include <math.h>

class PurePursuit
{
 private:
    ros::NodeHandle _n;
    
    ros::Subscriber _goal_sub;
    ros::Subscriber _odom_sub;
    
    ros::Publisher _path_pub;
    ros::Publisher _twist_pub;
    
private:
    visualization_msgs::Marker _path_viz_msg;
    std::vector<geometry_msgs::Point> _path;
    
    geometry_msgs::PoseStamped current_pos_odom;
    geometry_msgs::PoseStamped current_pos_map;
    
    tf::TransformListener listener;
private:
    geometry_msgs::Point p_la;
    double lookahead_dist;
    
 public:
    PurePursuit()
        : _n("~"),
          _goal_sub(_n.subscribe("/robot_4/waypoint", 1, &PurePursuit::newGoalCallback, this)),
          _odom_sub(_n.subscribe("/robot_4/odom", 1000, &PurePursuit::odomCallback, this)),
          _path_pub(_n.advertise<visualization_msgs::Marker>("/robot_4/path_visualization", 1000)),
          _twist_pub(_n.advertise<geometry_msgs::Twist>("/robot_4/mobile_base/commands/velocity", 1000))
    {

        _path_viz_msg.header.frame_id  = "/map";
        _path_viz_msg.header.stamp = ros::Time::now();
        _path_viz_msg.ns = "waypoints";
        _path_viz_msg.action  = visualization_msgs::Marker::ADD;
        _path_viz_msg.type = visualization_msgs::Marker::LINE_STRIP;
        _path_viz_msg.pose.orientation.w =  1.0;
        _path_viz_msg.scale.x = 0.01;
        _path_viz_msg.color.b = 1.0;
        _path_viz_msg.color.a =  1.0;
        _path_viz_msg.points  = _path;

        current_pos_odom.header.frame_id = "/robot_4/odom";
        current_pos_map.header.frame_id = "/map";
        lookahead_dist = 0.1;
    }

    void odomCallback(const nav_msgs::Odometry& msg)
    { // get current position in odom and map coordinate systems
        current_pos_odom.pose = msg.pose.pose;
        
        listener.waitForTransform("/map", "/robot_4/odom", ros::Time::now(), ros::Duration(1, 0.0));
        listener.transformPose("/map", ros::Time(0), current_pos_odom, "/map", current_pos_map);
    }
    
    void newGoalCallback(const geometry_msgs::PointStamped& msg)
    {
        // Add new pose from goal command to path
        _path.push_back(msg.point);
    }
    
    void publishControls(){
        
        // ROS_WARN("PUBLISH_CONTROLS_START");
        
        // Compute controls based on lookahead point and publish them
        calcLookAheadPoint(_path, current_pos_map, lookahead_dist);
        
        Eigen::Vector2d v_robot_pos(current_pos_odom.pose.position.x, current_pos_odom.pose.position.y);
        ROS_INFO("Robot_Pos in publish controls: (%f, %f)", v_robot_pos.x(), v_robot_pos.y());
        ROS_INFO("Lookahead Point in publish Controls: (%f, %f)", p_la.x, p_la.y);
        
        // check if last path point approached
        double v;
        double omega;
        geometry_msgs::Twist twist;
        
        if (reached_last_path_point())
        {
            
            ROS_WARN("REACHED_LAST");
            _path.clear();
            v = 0.0;
            omega = 0.0;
        }
        else
        {
            // p_la and robot pos are in robotcentric frame
            Eigen::Vector2d v_robot_pos(current_pos_odom.pose.position.x, current_pos_odom.pose.position.y);
            Eigen::Vector2d rho(p_la.x - v_robot_pos.x(), p_la.y - v_robot_pos.y());
            
            ROS_INFO("RHO: %f", rho.norm());
            
            double curvature;
            if (rho.norm() != 0)
            {
                curvature = -2 * rho.x() / rho.squaredNorm();
                v = 0.2;
            }
            else
            {   
                v = 0;
                curvature = 0.0;
            }
            
            omega = curvature * v;
            
            twist.linear.x = v;
            twist.angular.z = omega;
        }
        
        //ROS_INFO("V: %f, Omega: %f", v, omega);
        
        //_twist_pub.publish(twist);

    }
    void publishMarkers()
    {
        _path_pub.publish(_path_viz_msg);
    }
private:                    // i become path, robot position (map frame), and lookahead distance
    void calcLookAheadPoint(const std::vector<geometry_msgs::Point> path, geometry_msgs::PoseStamped robot_pos, double d_la)
    {

        Eigen::Vector2d v_robot_pos(robot_pos.pose.position.x, robot_pos.pose.position.y); // current robot position in map coordinates;
        
        ROS_INFO("Robot_Pos: (%f, %f)", v_robot_pos.x(), v_robot_pos.y());
        
        if (path.size() > 1) // at least two goal points exist
        {
            Eigen::Vector2d v_closest_point(path[0].x, path[0].y);
            Eigen::Vector2d v_next_segment_point(path[1].x, path[1].y); // segment point next after the closest point
            double d_min = (v_closest_point - v_robot_pos).norm();
            
            ROS_WARN("GO VIA PATH");
            // Find the closest to robot point on path
            for (int i = 0; i < path.size() - 1; i++)
            {
                
                // find length of orthogonal projection
                // (1) represent path points and robot position as vectors
                // (2) calculate assistant value t
                // (3) depending on t calculate distance to the nearest point on the segment
                // (4) save distance if it is minimum
                
                Eigen::Vector2d v_segment_start(path[i].x, path[i].y); // x[i]
                Eigen::Vector2d v_segment_end(path[i+1].x, path[i+1].y); // x[i+1]
                
                ROS_INFO("Segment_Start: (%f, %f)", v_segment_start.x(), v_segment_start.y());
                ROS_INFO("Segment_End: (%f, %f)", v_segment_end.x(), v_segment_end.y());
                
                double t = ((v_robot_pos - v_segment_start).dot(v_segment_end - v_segment_start)) /
                            (v_segment_end - v_segment_start).squaredNorm();
                            
                ROS_INFO("t: %f", t);
                
                Eigen::Vector2d v_evtl_closest_point;
                
                if (t >= 1)
                    v_evtl_closest_point = v_segment_end; // x[i+1]
                else if (t <= 0)
                    v_evtl_closest_point = v_segment_start; // x[i]
                else
                    v_evtl_closest_point = v_segment_start + t * (v_segment_end - v_segment_start);
                
                ROS_INFO("Evtl Closest_Point: (%f, %f)", v_evtl_closest_point.x(), v_evtl_closest_point.y());
                ROS_INFO("Closest_Point: (%f, %f)", v_closest_point.x(), v_closest_point.y());
                
                // distance to the closest point on current segment
                double d = (v_evtl_closest_point - v_robot_pos).norm();
                
                ROS_INFO("Distance to the evtl closest Point: %f", d);
                ROS_INFO("Min Distance before: %f", d_min);
                if (d < d_min)
                {
                    v_closest_point = v_evtl_closest_point;
                    d_min = d;
                    v_next_segment_point = v_segment_end;
                }
                
                ROS_INFO("Min Distance after: %f", d_min);
                ROS_INFO("Next_Segment_Point: (%f, %f)", v_next_segment_point.x(), v_next_segment_point.y());
            }
            ROS_ERROR("GO VIA PATH END");
            
            ROS_INFO("Final Min Distance: %f", d_min);
            ROS_INFO("Final Next_Segment_Point: (%f, %f)", v_next_segment_point.x(), v_next_segment_point.y());
            
            // add lookahead distance to the closest point
            // (1) define unit vector
            // (2) multiply lookahead distance by this unit vector
            // (3) check if lookahead point is still on the right segment
            
            Eigen::Vector2d v_unit_segment = (v_next_segment_point - v_closest_point) / (v_next_segment_point - v_closest_point).norm();
            Eigen::Vector2d v_lookahead_dist = d_la * v_unit_segment;
            
            ROS_INFO("Lookahead Vector: (%f, %f)", v_lookahead_dist.x(), v_lookahead_dist.y());
            ROS_INFO("Lookahead dist: %f", v_lookahead_dist.norm());
            ROS_INFO("Dist to the next segment point: %f", (v_next_segment_point - v_closest_point).norm());
            
            if (v_lookahead_dist.norm() < (v_next_segment_point - v_closest_point).norm())
            {
                p_la.x = (v_closest_point + v_lookahead_dist).x();
                p_la.y = (v_closest_point + v_lookahead_dist).y();
            }
            else
            {
                p_la.x = v_next_segment_point.x();
                p_la.y = v_next_segment_point.y();
                
            }
            
            ROS_WARN("Lookahead Point: (%f, %f)", p_la.x, p_la.y);
            
            geometry_msgs::PointStamped source_point, transformed_point;
            source_point.header.frame_id = "/map";
            source_point.point = p_la;
        
            transformed_point.header.frame_id = "/robot_4/base_link";
        
            listener.waitForTransform("/robot_4/base_link", "/map", ros::Time::now(), ros::Duration(1, 0.0));
            listener.transformPoint("/robot_4/base_link", ros::Time(0), source_point, "/map", transformed_point);
        
            p_la = transformed_point.point;
        
        }
        else
        {
            p_la.x = current_pos_odom.pose.position.x;
            p_la.y = current_pos_odom.pose.position.y;
        }
            
    }
    
private:
    void print_path()
    {
        for (int i = 0; i < _path.size(); i++)
            ROS_INFO("Path Point[%d]: (%f, %f)", i, _path[i].x, _path[i].y);
    }
    
    bool reached_last_path_point()
    {
        Eigen::Vector2d v_robot_pos(current_pos_odom.pose.position.x, current_pos_odom.pose.position.y);
        Eigen::Vector2d rho(p_la.x - v_robot_pos.x(), p_la.y - v_robot_pos.y());
        
        return rho.norm() < 0.1 && _path.size() > 1;
    }
    

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "purepursuit_node");

    PurePursuit pp;
    ros::Rate r(10);

    while (ros::ok())
    {
        ros::spinOnce();
        pp.publishControls();
        //pp.publishMarkers();
        r.sleep();
    }
    return 0;
}
