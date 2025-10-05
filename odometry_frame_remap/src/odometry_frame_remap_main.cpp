/**
 * @file odometry_frame_remap_main.cpp
 * 
 * @brief       odometry_frame_remap
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/02/09
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "odometry_frame_remap/odometry_frame_remap.hpp"

int main( int argc, char *argv[] )
{
	rclcpp::init( argc, argv );
	rclcpp::spin( std::make_shared<OdometryFrameRemap>() );
	rclcpp::shutdown();

	return 0;
}
