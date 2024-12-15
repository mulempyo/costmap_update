#include <dwa_planner_ros/dwa_planner_ros.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher marker_array_pub;

void markPublish(){
    visualization_msgs::MarkerArray marker_array;

    std::array<float, 7> safe1 = {-0.063431f, -0.031137f, 0.0f, 0.0f, 0.0f, 0.19328f, 0.999903f};
    std::array<float, 7> safe2 = {4.273204f, 0.379562f, 0.0f, 0.0f, 0.0f, -0.998399f, 0.056565f};

    for(int i=0; i<5; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "array_namespace";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        if(i % 2 == 0){
            marker.pose.position.x = safe1[0];
            marker.pose.position.y = safe1[1];
            marker.pose.position.z = safe1[2];
            marker.pose.orientation.x = safe1[3];
            marker.pose.orientation.y = safe1[4];
            marker.pose.orientation.z = safe1[5];
            marker.pose.orientation.w = safe1[6];
        }
        else{
            marker.pose.position.x = safe2[0];
            marker.pose.position.y = safe2[1];
            marker.pose.position.z = safe2[2];
            marker.pose.orientation.x = safe2[3];
            marker.pose.orientation.y = safe2[4];
            marker.pose.orientation.z = safe2[5];
            marker.pose.orientation.w = safe2[6];
        }

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        marker.lifetime = ros::Duration();

        marker_array.markers.push_back(marker);
    }

    marker_array_pub.publish(marker_array);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_map");

    ros::NodeHandle nh;

    dwa_planner_ros::DWAPlannerROS dwa_;
    
    dwa_.sub_ = nh.subscribe("scan", 10, &dwa_planner_ros::DWAPlannerROS::laserCallback, &dwa_);

    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        markPublish();
    });
   
    ros::spin();

    return 0;
}
