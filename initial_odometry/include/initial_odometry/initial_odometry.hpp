/**
 * @file initial_odometry.hpp
 * 
 * @brief       initial_odometry
 * @note        なし
 * 
 * @version     1.1.0
 * @date        2024/01/05
 * 
 * @copyright   (C) 2023-2024 Motoyuki Endo
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class InitialOdometry : public rclcpp::Node
{
	private:
		static const rclcpp::Duration INVALID_TIME;

		bool isValid_;

		rclcpp::Clock rosclock_;
		rclcpp::Time timeInvalid_;

		std::vector<double> initialPose_;

		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterSubscription_;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

		void MainLoop( void );
		void MainCycle( void );
		void PublishInitialOdom( void );
		void SubscribeGpsOdom( const nav_msgs::msg::Odometry::SharedPtr msg );
		void UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event );

	public:
		InitialOdometry();
		~InitialOdometry();
};
