#include <costmap_update/costmap_update.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace update{

void MapUpdate::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan){
   global_costmap_ = global_costmap_ros_->getCostmap();
   updateCostmap(laser_scan, global_costmap_);
}

void MapUpdate::updateCostmap(const sensor_msgs::LaserScanConstPtr& laser_scan_, costmap_2d::Costmap2D* costmap){  //using global planner

    if (!laser_scan_) {
      ROS_WARN("laser_scan_ is null");
    }

    if (!costmap) {
      ROS_WARN("costmap is null");
    }

    ROS_INFO("updateCostmap start");

    std::vector<geometry_msgs::Point> obstacles;
    double angle = laser_scan_->angle_min;
    ROS_INFO("updateCostmap for (const auto& range : laser_scan_->ranges) start");
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
    ROS_WARN("updateCostmap for (const auto& obs : obstacles) start");
    for (const auto& obs : obstacles) {
        unsigned int mx, my;
        if (costmap->worldToMap(obs.x, obs.y, mx, my)) {
            ROS_INFO("make Costmap start.");
            costmap->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
    ROS_WARN("updateCostmap for (unsigned int i = 0; i < size_x; ++i) start");
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
                    ROS_INFO("remove Costmap start");
                    costmap->setCost(i, j, costmap_2d::FREE_SPACE);
                }
            }
        }
    }
  } 


}//namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_map");

    ros::NodeHandle nh;
    update::MapUpdate update_;
   
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    update_.global_costmap_ros_ = new costmap_2d::Costmap2DROS("update_costmap", buffer);
 
    update_.sub = nh.subscribe("scan",10,&update::MapUpdate::laserReceived, &update_);
    update_.pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan",10);

    ros::spin();
    return 0;
}
