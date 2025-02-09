/**
 * @file odometry_frame_remap.hpp
 * 
 * @brief       odometry_frame_remap
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/02/09
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

class OdometryFrameRemap : public rclcpp::Node
{
	private:
		rclcpp::Clock rosclock_;

		std::string new_frame_id_;
		std::string new_child_frame_id_;
		bool publish_tf_;

		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterSubscription_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

		void MainLoop( void );
		void MainCycle( void );
		void OdometryCallback( const nav_msgs::msg::Odometry::SharedPtr msg );
		void UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event );

	public:
		OdometryFrameRemap();
		~OdometryFrameRemap();
};
