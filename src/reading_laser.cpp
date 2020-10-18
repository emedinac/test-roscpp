#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include <sstream>

#define RAD2DEG(x) ((x)*180./M_PI)

int regions = 16;
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg){
    
    double arr[regions];
    int div = msg->ranges.size()/regions;
    for(size_t i=0;i<regions;i++){
        double val=1000.;
        for(size_t j=0;j<div;j++){
            double point = abs(msg->ranges[i*div+j]);
            val = std::min(val, std::min( 10., point));
        }
        arr[i] = val;
    }
    
    std::ostringstream os;
    for (double i: arr)
        os << i << " ";
    std::string scan_division(os.str());

    ROS_INFO("LaserScan : %s", scan_division.c_str());
}

int main(int argc, char **argv){

    ros::init(argc, argv, "reading_laser");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 1000, clbk_laser);
    ros::spin();
    return 0;
}
