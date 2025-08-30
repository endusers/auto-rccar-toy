/**
 * @file navsatfix_conditional_relay_main.cpp
 * 
 * @brief       navsatfix_conditional_relay
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/08/30
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "navsatfix_conditional_relay/navsatfix_conditional_relay.hpp"

int main( int argc, char *argv[] )
{
	rclcpp::init( argc, argv );
	rclcpp::spin( std::make_shared<NavSatFixConditionalRelay>() );
	rclcpp::shutdown();

	return 0;
}
