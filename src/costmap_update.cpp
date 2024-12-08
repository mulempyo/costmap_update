#include <dwa_planner_ros/dwa_planner_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_map");

    ros::NodeHandle nh;

    dwa_planner_ros::DWAPlannerROS dwa_;
    
    dwa_.sub_ = nh.subscribe("scan", 10, &dwa_planner_ros::DWAPlannerROS::laserCallback, &dwa_);
   
    ros::spin();

    return 0;
}
