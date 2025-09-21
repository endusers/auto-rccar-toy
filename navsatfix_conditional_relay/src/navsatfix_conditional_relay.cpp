/**
 * @file navsatfix_conditional_relay.cpp
 * 
 * @brief       navsatfix_conditional_relay
 * @note        なし
 * 
 * @version     1.1.0
 * @date        2025/09/21
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "navsatfix_conditional_relay/navsatfix_conditional_relay.hpp"

using namespace std::placeholders;

NavSatFixConditionalRelay::NavSatFixConditionalRelay()
	: Node( "navsatfix_conditional_relay" )
{
	relay_status_threshold_ = this->declare_parameter<int8_t>( "relay_status_threshold", 2 );
	relay_sigma_threshold_ = this->declare_parameter<double_t>( "relay_sigma_threshold", 0.03 );
	enable_status_override_ = this->declare_parameter<bool>( "enable_status_override", false );
	override_status_ = this->declare_parameter<int8_t>( "override_status", 2 );

	parameterSubscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
		"/parameter_events", 10, std::bind( &NavSatFixConditionalRelay::UpdateParameters, this, _1 ) );

	auto qos = rclcpp::SensorDataQoS();

	subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
		"/gnss/in", qos, std::bind( &NavSatFixConditionalRelay::GnssCallback, this, _1 ) );

	publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>( "/gnss/out", 10);
}

NavSatFixConditionalRelay::~NavSatFixConditionalRelay()
{
	// TODO
}

void NavSatFixConditionalRelay::GnssCallback( const sensor_msgs::msg::NavSatFix::SharedPtr msg )
{
	auto navsatfix = *msg;
	bool is_relay = false;
	double sigma_horizontal = 0.0;

	if( enable_status_override_ )
	{
		navsatfix.status.status = override_status_;
	}

	if( navsatfix.status.status >= relay_status_threshold_ )
	{
		is_relay = true;
	}

	sigma_horizontal = std::sqrt( navsatfix.position_covariance[0] + navsatfix.position_covariance[4] );
	if( sigma_horizontal <= relay_sigma_threshold_ )
	{
		is_relay = true;
	}

	if( is_relay )
	{
		publisher_->publish( navsatfix );
	}
}

void NavSatFixConditionalRelay::UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event )
{
	if( event->node == this->get_fully_qualified_name() )
	{
		this->get_parameter( "relay_status_threshold", relay_status_threshold_ );
		this->get_parameter( "relay_sigma_threshold", relay_sigma_threshold_ );
		this->get_parameter( "enable_status_override", enable_status_override_ );
		this->get_parameter( "override_status", override_status_ );
	}
}
