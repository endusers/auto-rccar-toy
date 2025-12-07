/**
 * @file pointcloud_filter_main.cpp
 * 
 * @brief       pointcloud_filter
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/11/16
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "pointcloud_filter/pointcloud_filter.hpp"

int main( int argc, char *argv[] )
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<PointcloudFilter>() );
    rclcpp::shutdown();

    return 0;
}
