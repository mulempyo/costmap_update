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
  buffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  update_costmap_ros_ = new costmap_2d::Costmap2DROS("update_costmap", *buffer_);
  update_costmap_ros_->start(); 
}

MapUpdate::~MapUpdate(){
   delete update_costmap_ros_;
}

void MapUpdate::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan){
   update_costmap_ = update_costmap_ros_->getCostmap();
   updateCostmap(laser_scan, update_costmap_);
}

void MapUpdate::updateCostmap(const sensor_msgs::LaserScanConstPtr& laser_scan_, costmap_2d::Costmap2D* costmap){  

    if (!laser_scan_) {
      ROS_WARN("laser_scan_ is null");
      return;
    }

    if (!costmap) {
      ROS_WARN("costmap is null");
      return;
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
   
  nav_msgs::OccupancyGrid grid;
  grid.header.stamp = ros::Time::now();
  grid.header.frame_id = "odom";
  grid.info.resolution = costmap->getResolution();
  grid.info.width = costmap->getSizeInCellsX();
  grid.info.height = costmap->getSizeInCellsY();
  grid.info.origin.position.x = costmap->getOriginX();
  grid.info.origin.position.y = costmap->getOriginY();
  grid.data.resize(grid.info.width * grid.info.height);

  for (unsigned int i = 0; i < grid.info.width; ++i) {
    for (unsigned int j = 0; j < grid.info.height; ++j) {
        unsigned int index = j * grid.info.width + i;
        grid.data[index] = costmap->getCost(i, j);
      }
  }


  if (!buffer_->canTransform("base_footprint", "odom", ros::Time::now(), ros::Duration(1.0))) {
    ROS_WARN("Cannot transform from base_footprint to odom");
    return;
  }

  geometry_msgs::TransformStamped transformStamped;
  try {
      transformStamped = buffer_->lookupTransform("base_footprint", "odom", ros::Time::now(), ros::Duration(1.0));
  } catch (tf2::TransformException& ex) {
      ROS_WARN("Transform error: %s", ex.what());
      return;
  }

  nav_msgs::OccupancyGrid local_grid = grid;
  local_grid.header.frame_id = "base_footprint";

  geometry_msgs::PointStamped origin_odom, origin_base;
  origin_odom.point.x = grid.info.origin.position.x;
  origin_odom.point.y = grid.info.origin.position.y;
  origin_odom.point.z = 0.0;
  origin_odom.header.frame_id = "odom";

  tf2::doTransform(origin_odom, origin_base, transformStamped);
  local_grid.info.origin.position.x = origin_base.point.x;
  local_grid.info.origin.position.y = origin_base.point.y;
  pub.publish(local_grid); 
} 

} // namespace update

int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_map");

    ros::NodeHandle nh;
   
    update::MapUpdate update_;
  
    update_.sub = nh.subscribe("scan", 10, &update::MapUpdate::laserReceived, &update_);
    update_.pub = nh.advertise<nav_msgs::OccupancyGrid>("update_costmap", 10);

    dwa_planner_ros::DWAPlannerROS dwa_;
    dwa_.costmap_sub_ = nh.subscribe("update_costmap", 10, &dwa_planner_ros::DWAPlannerROS::costmapCallback, &dwa_);

    ros::spin();

    return 0;
}

