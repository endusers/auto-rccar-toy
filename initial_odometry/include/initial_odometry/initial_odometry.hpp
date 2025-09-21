/**
 * @file initial_odometry.hpp
 * 
 * @brief       initial_odometry
 * @note        なし
 * 
 * @version     1.3.0
 * @date        2025/09/21
 * 
 * @copyright   (C) 2023-2025 Motoyuki Endo
 */
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class InitialOdometry : public rclcpp::Node
{
	private:
		static const rclcpp::Duration INVALID_TIME;

		bool isValid_;

		rclcpp::Time timeInvalid_;

		std::string frame_id_;
		std::string child_frame_id_;
		std::vector<double> initial_pose_;
		bool publish_tf_;

		bool enable_parent_frame_;
		std::string parent_frame_;
		std::vector<double> parent_initial_pose_;

		std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_br_;
		std::unique_ptr<tf2_ros::StaticTransformBroadcaster> parent_static_tf_br_;

		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterSubscription_;

		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

		void MainLoop( void );
		void MainCycle( void );
		void PublishInitialOdom( void );
		void PublishParentStaticTf( void );
		void SubscribeGpsOdom( const nav_msgs::msg::Odometry::SharedPtr msg );
		void UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event );

	public:
		InitialOdometry();
		~InitialOdometry();
};
