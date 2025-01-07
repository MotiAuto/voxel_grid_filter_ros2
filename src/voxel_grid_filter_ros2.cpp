#include "voxel_grid_filter_ros2/voxel_grid_filter_ros2.hpp"

namespace voxel_grid_filter_ros2
{
    VoxelGridFilterROS2::VoxelGridFilterROS2(const rclcpp::NodeOptions& option) : Node("VoxelGridFilterROS2", option)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/input",
            qos_settings,
            std::bind(&VoxelGridFilterROS2::topic_callback, this, _1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output", rclcpp::SystemDefaultsQoS());

        this->declare_parameter("leaf_size", 0.05);
        this->get_parameter("leaf_size", leaf_size_);

        this->declare_parameter("pass_through_min", 0.0);
        this->get_parameter("pass_through_min", pass_through_min);

        this->declare_parameter("pass_through_max", 10.0);
        this->get_parameter("pass_through_max", pass_through_max);

        RCLCPP_INFO(this->get_logger(), "Start VoxelGridFilterROS2. leaf_size:%lf, min:%lf, max:%lf", leaf_size_, pass_through_min, pass_through_max);
    }

    void VoxelGridFilterROS2::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud);
        filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        filter.filter(*voxel_grid_filtered);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(voxel_grid_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(pass_through_min, pass_through_max);
        pass.filter(*filtered);

        sensor_msgs::msg::PointCloud2::SharedPtr new_msg(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*filtered, *new_msg);
        pub_->publish(*new_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(voxel_grid_filter_ros2::VoxelGridFilterROS2)