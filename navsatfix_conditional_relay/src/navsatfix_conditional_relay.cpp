/**
 * @file navsatfix_conditional_relay.cpp
 * 
 * @brief       navsatfix_conditional_relay
 * @note        なし
 * 
 * @version     1.3.0
 * @date        2025/10/19
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
	enable_covariance_override_ = this->declare_parameter<bool>( "enable_covariance_override", false );
	override_covariance_sigma_threshold_ = this->declare_parameter<double_t>( "override_covariance_sigma_threshold", 0.03 );
	override_covariance_east_ = this->declare_parameter<double_t>( "override_covariance_east", 1e6 );
	override_covariance_north_ = this->declare_parameter<double_t>( "override_covariance_north", 1e6 );
	override_covariance_up_ = this->declare_parameter<double_t>( "override_covariance_up", 1e6 );
	restore_covariance_sigma_threshold_ = this->declare_parameter<double_t>( "restore_covariance_sigma_threshold", 0.02 );
	restore_covariance_duration_ = this->declare_parameter<double_t>( "restore_covariance_duration", 30.0 );

	is_override_ = false;
	is_restore_checking_ = false;
	tim_restore_ = this->get_clock()->now() + rclcpp::Duration::from_seconds( restore_covariance_duration_ );

	sub_parameter_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
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
	bool is_checking = false;

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

	if( enable_covariance_override_ )
	{
		is_checking = is_restore_checking_;

		if( sigma_horizontal > override_covariance_sigma_threshold_ )
		{
			is_override_ = true;
			is_restore_checking_ = false;
		}

		if( sigma_horizontal <= restore_covariance_sigma_threshold_ )
		{
			is_restore_checking_ = true;
		}


		if( is_checking != is_restore_checking_ )
		{
			if( is_restore_checking_ )
			{
				tim_restore_ = this->get_clock()->now() + rclcpp::Duration::from_seconds( restore_covariance_duration_ );
			}
		}

		if( is_restore_checking_ )
		{
			if( this->get_clock()->now() >= tim_restore_ )
			{
				is_override_ = false;
				is_restore_checking_ = false;
			}
		}

		if( is_override_ )
		{
			navsatfix.position_covariance[0] = override_covariance_east_;
			navsatfix.position_covariance[4] = override_covariance_north_;
			navsatfix.position_covariance[8] = override_covariance_up_;
		}
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
		this->get_parameter( "enable_covariance_override", enable_covariance_override_ );
		this->get_parameter( "override_covariance_sigma_threshold", override_covariance_sigma_threshold_ );
		this->get_parameter( "override_covariance_east", override_covariance_east_ );
		this->get_parameter( "override_covariance_north", override_covariance_north_ );
		this->get_parameter( "override_covariance_up", override_covariance_up_ );
		this->get_parameter( "restore_covariance_sigma_threshold", restore_covariance_sigma_threshold_ );
		this->get_parameter( "restore_covariance_duration", restore_covariance_duration_ );
	}
}
