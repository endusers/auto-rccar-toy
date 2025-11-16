/**
 * @file pointcloud_filter.hpp
 * 
 * @brief       pointcloud_filter
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/11/16
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class PointcloudFilter : public rclcpp::Node
{
    private:
        double_t min_angle_deg_;
        double_t max_angle_deg_;
        std::string frame_id_;

        double_t min_angle_;
        double_t max_angle_;

        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub_parameter_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

        void PointCloudCallback( const sensor_msgs::msg::PointCloud2::SharedPtr msg );
        void UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event );

    public:
        PointcloudFilter();
        ~PointcloudFilter();
};
