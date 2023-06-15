#include "initial_odometry/initial_odometry.hpp"

int main( int argc, char *argv[] )
{
	rclcpp::init( argc, argv );
	rclcpp::spin( std::make_shared<InitialOdometry>() );
	rclcpp::shutdown();

	return 0;
}
