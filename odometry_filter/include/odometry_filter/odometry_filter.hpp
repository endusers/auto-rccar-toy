/**
 * @file odometry_filter.hpp
 * 
 * @brief       odometry_filter
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/09/28
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include <list>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


class OdometryFilter : public rclcpp::Node
{
	private:
		std::string map_frame_;
		std::string odom_frame_;
		std::string base_link_frame_;
		double_t twist_publish_rate_;
		std::vector<double> initial_pose_;
		bool update_yaw_only_;
		bool publish_tf_;

		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Time tim_twist_publish_;
		nav_msgs::msg::Odometry odom_;
		std::list<nav_msgs::msg::Odometry> lst_odom_;

		tf2::Transform offset_tf_;

		rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub_parameter_;
		rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_twist_;

		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

		void MainCycle( void );
		void OdometryCallback( const nav_msgs::msg::Odometry::SharedPtr msg );
		void InitialPoseCallback( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg );
		void SetOffsetTransform( void );
		void UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event );

	public:
		OdometryFilter();
		~OdometryFilter();
};
