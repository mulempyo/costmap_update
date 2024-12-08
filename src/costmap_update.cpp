#include <costmap_update/costmap_update.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <nav_msgs/OccupancyGrid.h>
#include <dwa_planner_ros/dwa_planner_ros.h>
#include <memory>

namespace update{

MapUpdate::MapUpdate(){
    tf_.reset(new tf2_ros::Buffer);
    tf_->setUsingDedicatedThread(true);
    tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

void MapUpdate::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan){
   update_costmap_ = update_costmap_ros_->getCostmap();
   updateCostmap(laser_scan, update_costmap_);
}

void MapUpdate::updateCostmap(const sensor_msgs::LaserScanConstPtr& laser_scan_, costmap_2d::Costmap2D* costmap){  

    if (!laser_scan_) {
      ROS_WARN("laser_scan_ is null");
    }

    if (!costmap) {
      ROS_WARN("costmap is null");
    }

    std::vector<geometry_msgs::Point> obstacles;
    double angle = laser_scan_->angle_min;
    
    for (const auto& range : laser_scan_->ranges) {
        if (range >= laser_scan_->range_min && range <= laser_scan_->range_max) {
            geometry_msgs::Point obstacle;
            obstacle.x = range * std::cos(angle);
            obstacle.y = range * std::sin(angle);
            obstacle.z = 0.0;
            obstacles.push_back(obstacle);
        }
        angle += laser_scan_->angle_increment;
    }

    unsigned int size_x = costmap->getSizeInCellsX();
    unsigned int size_y = costmap->getSizeInCellsY();
 
    for (const auto& obs : obstacles) {
        unsigned int mx, my;
        if (costmap->worldToMap(obs.x, obs.y, mx, my)) {
            costmap->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
 
    for (unsigned int i = 0; i < size_x; ++i) {
        for (unsigned int j = 0; j < size_y; ++j) {
            if (costmap->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
                bool still_obstacle = false;
                for (const auto& obs : obstacles) {
                    unsigned int mx, my;
                    if (costmap->worldToMap(obs.x, obs.y, mx, my) && mx == i && my == j) {
                        still_obstacle = true;
                        break;
                    }
                }
                if (!still_obstacle) {
                    costmap->setCost(i, j, costmap_2d::FREE_SPACE);
                }
            }
        }
    }
   ROS_WARN("success costmap update");
  } 


}//namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_map");

    ros::NodeHandle nh;

    // Initialize TF buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize Costmap2DROS
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros = std::make_shared<costmap_2d::Costmap2DROS>("update_costmap", *tf_buffer);
    costmap_ros->start();

    // Initialize MapUpdate with the same Costmap2DROS instance
    update::MapUpdate update_;
    update_.update_costmap_ros_ = costmap_ros.get(); // Assuming update_costmap_ros_ is a raw pointer
    update_.sub = nh.subscribe("scan", 10, &update::MapUpdate::laserReceived, &update_);

    // Initialize DWAPlannerROS with the same Costmap2DROS instance
    //dwa_planner_ros::DWAPlannerROS dwa_planner;
    //dwa_planner.initialize("dwa_planner", tf_buffer.get(), costmap_ros.get());

    // Optionally, set up publishers/subscribers for DWAPlannerROS
    // For example, subscribe to a global plan and publish velocity commands
    // This depends on your specific application and integration needs

    // Main loop
    ros::spin();

    return 0;
}
