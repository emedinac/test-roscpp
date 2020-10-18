#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_listener.h>
#include<sensor_msgs/LaserScan.h>

#include<math.h>       /* atan2 */
#include<sstream>

#define RAD2DEG(x) ((x)*180./M_PI)
// http://gazebosim.org/tutorials?tut=ros_gzplugins#GPULaser
// https://docs.ros.org/api/catkin/html/howto/format1/building_libraries.html

int regions = 5;
ros::Publisher pub;
// machine state
double state_ = 0;

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

const char * state_names[]={"find the wall", "turn left", "follow the wall"};
void change_state(double state){
    state_ = state;
    ROS_INFO("State changed to %s", state_names[int(state_)]);
}

template <class statemachine>
void take_action(statemachine states){
    geometry_msgs::Twist msg;
    std::string string_states;
    std::string state_description = "";

    double dist = 1.5;

    if (states.front > dist && states.fleft > dist && states.fright > dist){
        state_description = "case 1 - nothing";
        change_state(0);
    }
    else if (states.front < dist && states.fleft > dist && states.fright > dist){
        state_description = "case 2 - front";
        change_state(1);
    }
    else if (states.front > dist && states.fleft > dist && states.fright < dist){
        state_description = "case 3 - fright";
        change_state(2);
    }
    else if (states.front > dist && states.fleft < dist && states.fright > dist){
        state_description = "case 4 - fleft";
        change_state(0);
    }
    else if (states.front < dist && states.fleft > dist && states.fright < dist){
        state_description = "case 5 - front && fright";
        change_state(1);
    }
    else if (states.front < dist && states.fleft < dist && states.fright > dist){
        state_description = "case 6 - front && fleft";
        change_state(1);
    }
    else if (states.front < dist && states.fleft < dist && states.fright < dist){
        state_description = "case 7 - front && fleft && fright";
        change_state(1);
    }
    else if (states.front > dist && states.fleft < dist && states.fright < dist){
        state_description = "case 8 - fleft && fright";
        change_state(0);
    }
    else{
        state_description = "unknown case";
        string_states = ConverToString(states);
        ROS_INFO("states : %s", string_states.c_str());

    }
    ROS_INFO("state description : %s", state_description.c_str());
    string_states = ConverToString(states);
    ROS_INFO("states : %s", string_states.c_str());
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
    take_action(states);
}

geometry_msgs::Twist find_wall(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0.2;
    msg.angular.z = -0.3;
    return msg;
}

geometry_msgs::Twist turn_left(){
    geometry_msgs::Twist msg;
    msg.angular.z = 0.3;
    return msg;
}

geometry_msgs::Twist follow_the_wall(){
    geometry_msgs::Twist msg;
    msg.linear.x = -0.5;
    return msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "follow_wall");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("/scan", 1000, clbk_laser);

    ros::Rate rate(20); // ROS Rate at 5Hz
    while (ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Twist msg;
        if(state_==0)
            msg = find_wall();
        else if(state_==1)
            msg = turn_left();
        else if(state_==2)
            msg = follow_the_wall();
        else
            ROS_INFO("Unknown state!");
        pub.publish(msg);
        rate.sleep();
    }
    return 0;
}
