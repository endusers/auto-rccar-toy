/**
 * @file odometry_frame_remap.cpp
 * 
 * @brief       odometry_frame_remap
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/02/09
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "odometry_frame_remap/odometry_frame_remap.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

OdometryFrameRemap::OdometryFrameRemap()
	: Node( "odometry_frame_remap" ),
	rosclock_( RCL_ROS_TIME )
{
	new_frame_id_ = this->declare_parameter<std::string>( "new_frame_id", "odom" );
	new_child_frame_id_ = this->declare_parameter<std::string>( "new_child_frame_id", "base_link" );
	publish_tf_ = this->declare_parameter<bool>( "publish_tf", false );

	parameterSubscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
		"/parameter_events", 10, std::bind( &OdometryFrameRemap::UpdateParameters, this, _1 ) );

	subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom/in", 10, std::bind( &OdometryFrameRemap::OdometryCallback, this, _1 ) );

	publisher_ = this->create_publisher<nav_msgs::msg::Odometry>( "odom/out", 10 );

	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>( this );

	timer_ = this->create_wall_timer( 100ms, std::bind( &OdometryFrameRemap::MainCycle, this ) );
}

OdometryFrameRemap::~OdometryFrameRemap()
{
	// TODO
}

void OdometryFrameRemap::MainLoop( void )
{
	// TODO
}

void OdometryFrameRemap::MainCycle( void )
{
	// TODO
}

void OdometryFrameRemap::OdometryCallback( const nav_msgs::msg::Odometry::SharedPtr msg )
{
	auto odom = *msg;

	// odom.header.stamp = rosclock_.now();
	odom.header.frame_id = new_frame_id_;
	odom.child_frame_id = new_child_frame_id_;

	if( publish_tf_ )
	{
		geometry_msgs::msg::TransformStamped odom_tf;

		odom_tf.transform.translation.x = odom.pose.pose.position.x;
		odom_tf.transform.translation.y = odom.pose.pose.position.y;
		odom_tf.transform.translation.z = odom.pose.pose.position.z;
		odom_tf.transform.rotation = odom.pose.pose.orientation;

		odom_tf.header.frame_id = new_frame_id_;
		odom_tf.child_frame_id = new_child_frame_id_;
		odom_tf.header.stamp = rosclock_.now();

		tf_broadcaster_->sendTransform( odom_tf );
	}
	publisher_->publish( odom );
}

void OdometryFrameRemap::UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event )
{
	if( event->node == this->get_fully_qualified_name() )
	{
		this->get_parameter( "new_frame_id", new_frame_id_ );
		this->get_parameter( "new_child_frame_id", new_child_frame_id_ );
		this->get_parameter( "publish_tf", publish_tf_ );
	}
}
