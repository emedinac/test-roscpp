#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<sstream>

#define RAD2DEG(x) ((x)*180./M_PI)
// http://gazebosim.org/tutorials?tut=ros_gzplugins#GPULaser
// https://docs.ros.org/api/catkin/html/howto/format1/building_libraries.html
int regions = 5;
ros::Publisher pub;
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
        os << *(arr.states+i) << " ";
    std::string scan_division(os.str());
    return scan_division;
}

// template <class statemachine>
void take_action(statemachine states){
    geometry_msgs::Twist msg;
    double linear_x = 0;
    double angular_z = 0;
    std::string string_states;
    std::string state_description = "";

    if (states.front > 0.75 && states.fleft > 0.75 && states.fright > 0.25){
        state_description = "case 1 - nothing";
        linear_x = 0.6;
        angular_z = 0;
    }
    else if (states.front < 0.75 && states.fleft > 0.75 && states.fright > 0.25){
        state_description = "case 2 - front";
        linear_x = 0;
        angular_z = 2.0;
    }
    else if (states.front > 0.75 && states.fleft > 0.75 && states.fright < 0.25){
        state_description = "case 3 - fright";
        linear_x = 0;
        angular_z = 2.0;
    }
    else if (states.front > 0.75 && states.fleft < 0.75 && states.fright > 0.25){
        state_description = "case 4 - fleft";
        linear_x = 0;
        angular_z = -2.0;
    }
    else if (states.front < 0.75 && states.fleft > 0.75 && states.fright < 0.25){
        state_description = "case 5 - front && fright";
        linear_x = 0;
        angular_z = 2.0;
    }
    else if (states.front < 0.75 && states.fleft < 0.75 && states.fright > 0.25){
        state_description = "case 6 - front && fleft";
        linear_x = 0;
        angular_z = -2.0;
    }
    else if (states.front < 0.75 && states.fleft < 0.75 && states.fright < 0.25){
        state_description = "case 7 - front && fleft && fright";
        linear_x = 0;
        angular_z = 2.0;
    }
    else if (states.front > 0.75 && states.fleft < 0.75 && states.fright < 0.25){
        state_description = "case 8 - fleft && fright";
        linear_x = 2.0;
        angular_z = 5.0;
    }
    else{
        state_description = "unknown case";
        string_states = ConverToString(states);
        ROS_INFO("states : %s", string_states.c_str());
    }

    ROS_INFO("state description : %s", state_description.c_str());
    string_states = ConverToString(states);
    ROS_INFO("states : %s", string_states.c_str());

    msg.linear.x = -linear_x;
    msg.angular.z = angular_z;
    pub.publish(msg);
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

int main(int argc, char **argv){

    ros::init(argc, argv, "reading_laser");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 1000, clbk_laser);

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    while (pub.getNumSubscribers() < 1) {
        // wait for a connection to publisher
        // you can do whatever you like here or simply do nothing
    }

    ros::spin();
    return 0;
}
