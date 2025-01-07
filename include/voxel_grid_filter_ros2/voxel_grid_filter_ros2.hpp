#ifndef VOXEL_GRID_FILTER_ROS2_HPP_
#define VOXEL_GRID_FILTER_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using std::placeholders::_1;

namespace voxel_grid_filter_ros2
{
    class VoxelGridFilterROS2 : public rclcpp::Node
    {
        public:
        explicit VoxelGridFilterROS2(const rclcpp::NodeOptions&option = rclcpp::NodeOptions());

        void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        double leaf_size_, pass_through_min, pass_through_max;
        std::string field_name_;
    };
}

#endif