/**
 * @file imu_orientation_corrector_main.cpp
 * 
 * @brief       imu_orientation_corrector
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/05/11
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "imu_orientation_corrector/imu_orientation_corrector.hpp"

int main( int argc, char *argv[] )
{
	rclcpp::init( argc, argv );
	rclcpp::spin( std::make_shared<ImuOrientationCorrector>() );
	rclcpp::shutdown();

	return 0;
}
