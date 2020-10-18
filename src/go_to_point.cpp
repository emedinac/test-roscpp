#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_listener.h>

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

void change_state(double state){
    state_ = state;
    ROS_INFO("State changed to %f", state_);
}

void fix_yaw(geometry_msgs::Point des_pos){
    double desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
    double err_yaw = desired_yaw - yaw_;

    geometry_msgs::Twist twist_msg;
    if(abs(err_yaw) > yaw_precision_){
        if(err_yaw > 0)
            twist_msg.angular.z = 0.3;
        else 
            twist_msg.angular.z = -0.3;
    }
    
    pub.publish(twist_msg);
    
    // state change conditions
    if (abs(err_yaw) <= yaw_precision_){
        ROS_INFO("Yaw error: [%f]", err_yaw);
        change_state(1);

    }
}
void go_straight_ahead(geometry_msgs::Point des_pos){
    double desired_yaw = atan2(des_pos.y - position_.y, des_pos.x - position_.x);
    double err_yaw = desired_yaw - yaw_;
    double err_pos = sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2));
    
    if (err_pos > dist_precision_){
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = 0.8;
        pub.publish(twist_msg);        
    }
    else{
        ROS_INFO("Position error: [%f]", err_pos);
        change_state(2);        
    }
    
    // state change conditions
    if (abs(err_yaw) > yaw_precision_){
        ROS_INFO("Yaw error: [%f]", err_yaw);
        change_state(0);     
    }

}
void done(){
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
    pub.publish(twist_msg);
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

    ros::init(argc, argv, "go_to_point");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("/odom", 1000, &clbk_odom);

    ros::Rate rate(20); // ROS Rate at 5Hz
    while (ros::ok()) {
        ros::spinOnce();
        if(state_==0)
            fix_yaw(desired_position_);
        else if(state_==1)
            go_straight_ahead(desired_position_);
        else if(state_==2)
            done();
        else
            ROS_INFO("Unknown state!");
        
        rate.sleep();
    }
    return 0;
}
