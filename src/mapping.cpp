#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_listener.h>
#include<sensor_msgs/LaserScan.h>

#include"follow_walls.h"
#include"go_to_point.h"
#include"obstacle_avoidance.h"

#include<math.h>       /* atan2 */
#include<sstream>

#define RAD2DEG(x) ((x)*180./M_PI)
// http://gazebosim.org/tutorials?tut=ros_gzplugins#GPULaser
// https://docs.ros.org/api/catkin/html/howto/format1/building_libraries.html

// robot state variables
geometry_msgs::Point position_;
double yaw_ = 0;
// machine state
double state_ = 0;
// goal
geometry_msgs::Point desired_position_;
// parameters
double yaw_precision_ = M_PI / 90; // +/- 2 degree allowed
double dist_precision_ = 0.3;
// publishers
ros::Publisher pub;

int regions = 5;
// machine state

class statemachine{
    public:
        double right=0;
        double fright=0;
        double front=0;
        double fleft=0;
        double left=0;
        double*states = &right;
};

template <class statemachine>
std::string ConverToString(statemachine arr){
    std::ostringstream os;
    for (size_t i=0; i<regions; i++)
        os << std::setprecision(2) << *(arr.states+i) << " ";
    std::string scan_division(os.str());
    return scan_division;
}


void change_state(double state){
    state_ = state;
    ROS_INFO("State changed to %f", state_);
}


void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg){
    statemachine states;
    int div = msg->ranges.size()/regions;
    for(size_t i=0;i<regions;i++){
        double val=1000.;
        for(size_t j=0;j<div;j++){
            double point = abs(msg->ranges[i*div+j]);
            val = std::min(val, std::min( 10., point)); // 0 as minimum
        }
        *(states.states+i) = val;
    }
}

void clbk_odom(const nav_msgs::Odometry::ConstPtr& msg){
    // position
    position_ = msg->pose.pose.position;
    // yaw
    tf::Quaternion quaternion(msg->pose.pose.orientation.x, 
                                msg->pose.pose.orientation.y, 
                                msg->pose.pose.orientation.z, 
                                msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(quaternion);
    double roll_, pitch_;
    m.getRPY(roll_, pitch_, yaw_); // yaw_ is the important 
}

int main(int argc, char **argv){
    desired_position_.x = -1.;
    desired_position_.y = 7.;
    desired_position_.z = 0.;
    ROS_INFO("Desired point");
    ROS_INFO("point x: %f", desired_position_.x);
    ROS_INFO("point y: %f", desired_position_.y);
    ROS_INFO("point z: %f", desired_position_.z);

    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;

    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, clbk_laser);
    ros::Subscriber sub_odom = nh.subscribe("/odom", 1000, &clbk_odom);
    
    ros::service::waitForService("/go_to_point_switch", 100);
    ros::service::waitForService("/wall_follower_switch", 100);
    ros::service::waitForService("/gazebo/set_model_state", 100);

    ros::ServiceClient go_to_point_ = nh.serviceClient<follow_walls>("/go_to_point_switch", true);
    ros::ServiceClient wall_follower_ = nh.serviceClient<go_to_point>("/wall_follower_switch", true);
    ros::ServiceClient set_model_state = nh.serviceClient<obstacle_avoidance>("/gazebo/set_model_state", true);
    

    ros::Rate rate(20); // ROS Rate at 5Hz
    while (ros::ok()) {
        ros::spinOnce();

        rate.sleep();
    }
    return 0;
}
