/**
 * @file odometry_filter_main.cpp
 * 
 * @brief       odometry_filter
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/09/28
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "odometry_filter/odometry_filter.hpp"

int main( int argc, char *argv[] )
{
	rclcpp::init( argc, argv );
	rclcpp::spin( std::make_shared<OdometryFilter>() );
	rclcpp::shutdown();

	return 0;
}
