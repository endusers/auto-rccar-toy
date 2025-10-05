/**
 * @file navsatfix_conditional_relay.hpp
 * 
 * @brief       navsatfix_conditional_relay
 * @note        なし
 * 
 * @version     1.2.0
 * @date        2025/09/28
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/int8.hpp>

class NavSatFixConditionalRelay : public rclcpp::Node
{
	public:
		NavSatFixConditionalRelay();
		~NavSatFixConditionalRelay();

	private:
		int8_t relay_status_threshold_;
		double_t relay_sigma_threshold_;
		bool enable_status_override_;
		int8_t override_status_;
		bool enable_covariance_override_;
		double_t override_covariance_sigma_threshold_;
		double_t override_covariance_east_;
		double_t override_covariance_north_;
		double_t override_covariance_up_;

		rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub_parameter_;
		rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;
		rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

		void GnssCallback( const sensor_msgs::msg::NavSatFix::SharedPtr msg );
		void UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event );
};
