/**
 * @file navsatfix_conditional_relay.hpp
 * 
 * @brief       navsatfix_conditional_relay
 * @note        なし
 * 
 * @version     1.3.0
 * @date        2025/10/19
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
		double_t restore_covariance_sigma_threshold_;
		double_t restore_covariance_duration_;

		bool is_override_;
		bool is_restore_checking_;
		rclcpp::Time tim_restore_;

		rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub_parameter_;
		rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;
		rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

		void GnssCallback( const sensor_msgs::msg::NavSatFix::SharedPtr msg );
		void UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event );
};
