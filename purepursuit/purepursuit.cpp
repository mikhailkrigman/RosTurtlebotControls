#include <algorithm>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include <math.h>

const float SENSOR_OFFSET = 0.25f; // distance between sensor and turtlebot rand (20cm)
const float D_STOP = 0.3f + SENSOR_OFFSET;
const float D_TURN = 0.4f + SENSOR_OFFSET;
const float D_AVOID = 0.4f + SENSOR_OFFSET;

const float MAX_V = 0.2;
const float MAX_OMEGA = 1.0;

const float MAX_BOOST = 1.0;

const float LOOKAHEAD_DIST = 0.3;

#define _USE_MATH_DEFINES



template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

class PathSegment
{
public:
    int start_index;
    int end_index;
public:
    PathSegment(): start_index(0), end_index(0){}
    PathSegment(int i, int i_plus_1): start_index(i), end_index(i_plus_1){}
    
    PathSegment& operator=(const PathSegment& segment)
    {
        if (this == &segment) return * this;
        
        this->start_index = segment.start_index;
        this->end_index = segment.end_index;
        
        return *this;
    }
    
    bool operator==(const PathSegment& other)
    {
        return this->start_index == other.start_index &&
                this->end_index == other.end_index;
    }
    
    PathSegment operator++(int)
    {
        this->start_index += 1;
        this->end_index += 1;
        return PathSegment(start_index, end_index);
    }
};

struct LaserscanData
{
    double angle_min;
    double angle_max;
    double d_min;
    double phi;
};

class Detour
{
private:
    double OBSTACLE_SIZE_X = 0.3;
    double OBSTACLE_SIZE_Y = 0.3;
    
    enum Side
    {
      S_RIGHT = -1,
      S_LEFT = 1
    };
    
    geometry_msgs::Point sideway1;
    geometry_msgs::Point straight;
    geometry_msgs::Point sideway2;
    
public:
    
    std::vector<geometry_msgs::Point> detour;
    Side side;
    
    Detour(LaserscanData data, geometry_msgs::Point robot_pos)
    {
        // decide by which side to avoid
        side = (data.phi > 0) ? S_LEFT : S_RIGHT;
        
        sideway1.x = robot_pos.x;
        sideway1.y = robot_pos.y + OBSTACLE_SIZE_Y * side;
        
        straight.x = sideway1.x + OBSTACLE_SIZE_X;
        straight.y = sideway1.y;
        
        sideway2.x = straight.x;
        sideway2.y = straight.y + OBSTACLE_SIZE_Y * (-side);
        
        detour.push_back(sideway1);
        detour.push_back(straight);
        detour.push_back(sideway2);
    }
    
    std::vector<geometry_msgs::Point> get_detour()
    {
        return this->detour;
    }
    
    std::vector<geometry_msgs::Point>::iterator begin()
    {
        return detour.begin();
    }
};

class PurePursuit
{
 private:
    ros::NodeHandle _n;
    
    ros::Subscriber _goal_sub;
    ros::Subscriber _odom_sub;
    ros::Subscriber _scan_sub;
    
    ros::Publisher _twist_pub;
    
    // visualization publishers
    ros::Publisher _path_pub;
    ros::Publisher la_point_pub;
    ros::Publisher marker_pub;
    
private:
    visualization_msgs::Marker _path_viz_msg;
    std::vector<geometry_msgs::Point> _path;
    
    geometry_msgs::PoseStamped current_pos_odom;
    geometry_msgs::PoseStamped current_pos_map;
    
    tf::TransformListener listener;
private:
    PathSegment current_segment;
    bool go_to_next_segment;
    
    geometry_msgs::Point p_la;
    double lookahead_dist;
    
    LaserscanData laserscan_data;
    float previous_omega;
    
 public:
    PurePursuit()
        : _n("~"),
          _goal_sub(_n.subscribe("/robot_4/waypoint", 1, &PurePursuit::newGoalCallback, this)),
          _odom_sub(_n.subscribe("/robot_4/odom", 1000, &PurePursuit::odomCallback, this)),
          _scan_sub(_n.subscribe("/robot_4/scan", 1000, &PurePursuit::scanCallback, this)),
          _twist_pub(_n.advertise<geometry_msgs::Twist>("/robot_4/mobile_base/commands/velocity", 1000)),
          _path_pub(_n.advertise<visualization_msgs::Marker>("/robot_4/path_visualization", 1000)),
          la_point_pub(_n.advertise<visualization_msgs::Marker>("/robot_4/visualization_marker", 1000)),
          marker_pub(_n.advertise<visualization_msgs::Marker>("/robot_4/visualization_marker", 1000))
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
        
        lookahead_dist = LOOKAHEAD_DIST;
        
        current_segment = PathSegment(0, 1);
        
        go_to_next_segment = false;
    }

public:
    void odomCallback(const nav_msgs::Odometry& msg)
    { // get current position in odom and map coordinate systems
        current_pos_odom.pose = msg.pose.pose;
        
        listener.waitForTransform("/map", "/robot_4/odom", ros::Time::now(), ros::Duration(1, 0.0));
        listener.transformPose("/map", ros::Time(0), current_pos_odom, "/map", current_pos_map);
    }
    
    void newGoalCallback(const geometry_msgs::PointStamped& msg)
    {   
        geometry_msgs::Point new_path_point = msg.point;
        
        // Add new pose from goal command to path
        _path.push_back(new_path_point);
        _path_viz_msg.points.push_back(new_path_point);
    }
    
    void scanCallback(const sensor_msgs::LaserScan& data)
    {
        int ranges_len = (data.angle_max - data.angle_min) / data.angle_increment;
        double dmin = data.ranges[0];
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
        
        laserscan_data.angle_min = data.angle_min;
        laserscan_data.angle_max = data.angle_max;
        laserscan_data.d_min = dmin;
        laserscan_data.phi = data.angle_min + j * data.angle_increment;
        
        //float phi_deg = laserscan_data.phi * (180.0 / M_PI);
        //ROS_INFO("dmin: %f, phi: %f", dmin, phi_deg);
        //ROS_INFO("Prev_Omega: %f", this->previous_omega);
        
        // draw line to the nearest wall in rviz
        
        
        geometry_msgs::Point start, end;
        
        // start point is robot position
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0; // 2D-Simulation
        
        // end point are coordinates of nearest wall
        end.x = dmin * std::cos(laserscan_data.phi);
        end.y = dmin * std::sin(laserscan_data.phi);
        end.z = 0; // 2D-Simulation
        
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "/robot_4/base_link";
        line_list.scale.x = 0.05;
        //line_list.header.stamp = ros::Time::now();
        
        line_list.color.r = 1.0f;
        line_list.color.a = 1.0;
        
        line_list.points.push_back(start);
        line_list.points.push_back(end);
        
        //marker_pub.publish(line_list);
    }
    
public:
    void send_velocity()
    {
        if (laserscan_data.d_min < D_AVOID && std::abs(laserscan_data.phi) < 10.0 * (180.0 / M_PI) )
        {
            ROS_INFO("AVOID OBSTACLE");
            ROS_INFO("D_MIN: %f", laserscan_data.d_min);
            avoid_obstacle(laserscan_data);
        }
        else
        {
            ROS_WARN("PUBLISH CONTROLS");
            publishControls();
        }
    }
    
    void publishControls(){
        
        //ROS_INFO("PATH_SIZE: %zu", _path.size());
        
        if (_path.size() < 2) return;
        
        if (reached_last_path_point())
        {
            ROS_WARN("REACHED_LAST");
            _path.clear();
            _path_viz_msg.points.clear();
            
            current_segment.start_index = 0;
            current_segment.end_index = 1;
            
            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.linear.z = 0.0;
            _twist_pub.publish(twist);
            
            return;
        }
        
        // Compute controls based on lookahead point and publish them
        //ROS_INFO("GO_TO_NEXT_SEGMENT: %d", go_to_next_segment);
        if(go_to_next_segment)
            current_segment++;

        calcLookAheadPoint(current_segment, current_pos_map, lookahead_dist);
        
        visualization_msgs::Marker la_point_marker;
        la_point_marker.header.frame_id = "/robot_4/base_link";
        la_point_marker.scale.x = 0.05;
        la_point_marker.points.push_back(p_la);
        la_point_marker.points.push_back(p_la);
        la_point_marker.color.r = 1.0;
        la_point_marker.color.g = 1.0;
        la_point_marker.color.b = 1.0;
        la_point_marker.color.a = 1.0;
        
        la_point_pub.publish(la_point_marker);
        
        // check if last path point approached
        double v;
        double omega;
        geometry_msgs::Twist twist;
        
        // calculate v and omega for define lookahead point p_la
        Eigen::Vector2d rho(p_la.x, p_la.y);
        
        //ROS_INFO("Rho: (%f, %f)", rho.x(), rho.y());
        
        double curvature;
        if (rho.norm() != 0)
        {
            if (rho.x() < 0 || std::abs(rho.y()) > 0.6) // if lookahead point behind the robot or too far besides
            {
                v = 0.0;
                omega = sgn<double>(rho.y()) * MAX_OMEGA;
            }
            else
            {
                curvature = 2 * rho.y() / rho.squaredNorm();
                v = MAX_V;
                omega = curvature * v;
            }
        }
        else
        {   
            v = 0;
            omega = 0.0;
        }
        
         // recalculate v and omega based on scanner callbacks
        
        //boost(v, omega, rho);
        
        twist.linear.x = v;
        twist.angular.z = omega;
        
        ROS_INFO("V: %f, Omega: %f", v, omega);
        
        _twist_pub.publish(twist);

    }
    
    void publishMarkers()
    {
        _path_pub.publish(_path_viz_msg);
    }
private:
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
    
private:
    void boost(double& v, double& omega, Eigen::Vector2d rho)
    {
        if (v == 0.0) return;
        
        if (std::abs(omega) < 0.05)
        {
            v += MAX_BOOST;
            lookahead_dist = LOOKAHEAD_DIST + 0.4;
        }
        else if(std::abs(omega) < 0.2)
        {
            v += 0.5 * MAX_BOOST;
            lookahead_dist = LOOKAHEAD_DIST + 0.2;
        }
        else if(std::abs(omega) < 0.35)
        {
            v += 0.3 * MAX_BOOST;
            lookahead_dist = LOOKAHEAD_DIST + 0.15;
        }
        else if(std::abs(omega) < 0.55)
        {
            v += 0.1 * MAX_BOOST;
            lookahead_dist = LOOKAHEAD_DIST + 0.1;
        }
        else
        {
            // no boost
            lookahead_dist = LOOKAHEAD_DIST;
        }
    }
    void avoid_obstacle(LaserscanData data)
    {
        double turn_direction = (data.phi < 0) ? 1 : -1;
        turn(M_PI_2 * turn_direction, MAX_OMEGA);
        forward(0.3, MAX_V);
        turn(M_PI_2 * (-turn_direction), MAX_OMEGA);
        forward(0.6, MAX_V);
        //turn(M_PI_2 * (-turn_direction), MAX_OMEGA);
        //forward(0.15, MAX_V);
        //turn(M_PI_2 * turn_direction, MAX_OMEGA);
    }
    
private:                    // i become path, robot position (map frame), and lookahead distance
    void calcLookAheadPoint(const PathSegment& segment, geometry_msgs::PoseStamped robot_pos, double d_la)
    {

        Eigen::Vector2d v_robot_pos(robot_pos.pose.position.x, robot_pos.pose.position.y); // current robot position in map coordinates;
        
        //ROS_INFO("Current_Segment: (%d, %d)", segment.start_index, segment.end_index);
        
        if (_path.size() >= 2) // at least two goal points exist
        {
            Eigen::Vector2d v_closest_point = find_closest_point_on_segment(segment, v_robot_pos);
            Eigen::Vector2d v_next_segment_point(_path[segment.end_index].x, _path[segment.end_index].y);
            
            // add lookahead distance to the closest point
            Eigen::Vector2d v_lookahead = get_la_vector(v_closest_point, v_next_segment_point, d_la);
            
            // check if lookahead point is still on path segment
            if (v_lookahead.norm() < (v_next_segment_point - v_closest_point).norm())
            {
                p_la.x = (v_closest_point + v_lookahead).x();
                p_la.y = (v_closest_point + v_lookahead).y();
            }
            else
            {
                p_la.x = v_next_segment_point.x();
                p_la.y = v_next_segment_point.y();
                
            }
            
            // transform lookahead point to robotcentric coordinates
            geometry_msgs::PointStamped source_point, transformed_point;
            source_point.header.frame_id = "/map";
            source_point.point = p_la;
        
            transformed_point.header.frame_id = "/robot_4/base_link";
        
            listener.waitForTransform("/robot_4/base_link", "/map", ros::Time::now(), ros::Duration(1, 0.0));
            listener.transformPoint("/robot_4/base_link", ros::Time(0), source_point, "/map", transformed_point);
        
            p_la = transformed_point.point;
        
            // go to the next segment if robot is close enough to the end of current segment
            go_to_next_segment = (v_next_segment_point - v_robot_pos).norm() < 0.75 * lookahead_dist && 
                                 (current_segment.end_index != _path.size() - 1);
        }
        else
        {
            p_la.x = 0;// already in robotcentric coords
            p_la.y = 0;// already in robotcentric coords
        }
            
    }
    
    Eigen::Vector2d find_closest_point_on_segment(PathSegment segment, Eigen::Vector2d v_robot_pos)
    { // finds closest point to specified path segment
        
        Eigen::Vector2d v_segment_start(_path[segment.start_index].x, _path[segment.start_index].y); // x[i]
        Eigen::Vector2d v_segment_end(_path[segment.end_index].x, _path[segment.end_index].y); // x[i+1]
        
        //ROS_INFO("Segment_Start: (%f, %f)", v_segment_start.x(), v_segment_start.y());
        //ROS_INFO("Segment_End: (%f, %f)", v_segment_end.x(), v_segment_end.y());
        
        double t = ((v_robot_pos - v_segment_start).dot(v_segment_end - v_segment_start)) /
                    (v_segment_end - v_segment_start).squaredNorm();
        
        Eigen::Vector2d v_segment_closest_point;
        if (t >= 1)
            v_segment_closest_point = v_segment_end; // x[i+1]
        else if (t <= 0)
            v_segment_closest_point = v_segment_start; // x[i]
        else
            v_segment_closest_point = v_segment_start + t * (v_segment_end - v_segment_start);
        
        return v_segment_closest_point;
    }
    
    Eigen::Vector2d get_la_vector(Eigen::Vector2d v_closest_point, Eigen::Vector2d v_next_segment_point, double d_la)
    {
        Eigen::Vector2d v_unit_segment = (v_next_segment_point - v_closest_point) / (v_next_segment_point - v_closest_point).norm();
        
        Eigen::Vector2d v_lookahead = d_la * v_unit_segment;
        
        return v_lookahead;
    }
    
    bool reached_last_path_point()
    {
        if (_path.size() >= 2){
            Eigen::Vector2d v_robot_pos(current_pos_map.pose.position.x, current_pos_map.pose.position.y);
            Eigen::Vector2d v_last_path_point(_path.back().x, _path.back().y);
            
            //ROS_INFO("Distance to the last path point: %f", (v_robot_pos - v_last_path_point).norm());
            return ((v_robot_pos - v_last_path_point).norm() < 0.1) &&
                    current_segment.end_index == _path.size() - 1;
            
        }
        
        return false;
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
        pp.send_velocity();
        pp.publishMarkers();
        r.sleep();
    }
    return 0;
}

