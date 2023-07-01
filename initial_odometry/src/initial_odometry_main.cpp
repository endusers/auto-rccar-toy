/**
 * @file initial_odometry_main.cpp
 * 
 * @brief       initial_odometry
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2023/03/26
 * 
 * @copyright   (C) 2023 Motoyuki Endo
 */
#include "initial_odometry/initial_odometry.hpp"

int main( int argc, char *argv[] )
{
	rclcpp::init( argc, argv );
	rclcpp::spin( std::make_shared<InitialOdometry>() );
	rclcpp::shutdown();

	return 0;
}
