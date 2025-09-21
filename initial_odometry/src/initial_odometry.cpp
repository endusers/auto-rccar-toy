/**
 * @file initial_odometry.cpp
 * 
 * @brief       initial_odometry
 * @note        なし
 * 
 * @version     1.3.0
 * @date        2025/09/21
 * 
 * @copyright   (C) 2023-2025 Motoyuki Endo
 */
#include "initial_odometry/initial_odometry.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define ARRAY_LENGTH(in_array)      (sizeof(in_array)/sizeof(in_array[0]))
#define UNUSED_VARIABLE(in_x)       (void)(in_x)

const rclcpp::Duration InitialOdometry::INVALID_TIME = rclcpp::Duration( 10s );

InitialOdometry::InitialOdometry()
	: Node( "initial_odometry" )
{
	isValid_ = false;

	timeInvalid_ = this->get_clock()->now();

	std::vector<double> default_pose = { 0.0, 0.0, 0.0 };

	frame_id_ = this->declare_parameter<std::string>( "frame_id", "map" );
	child_frame_id_ = this->declare_parameter<std::string>( "child_frame_id", "odom" );
	initial_pose_ = this->declare_parameter( "initial_pose", default_pose );
	publish_tf_ = this->declare_parameter<bool>( "publish_tf", false );

	enable_parent_frame_ = this->declare_parameter<bool>( "enable_parent_frame", false );
	parent_frame_ = this->declare_parameter<std::string>( "parent_frame", "base_link" );
	parent_initial_pose_ = this->declare_parameter( "parent_initial_pose", default_pose );

	static_tf_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>( this );
	parent_static_tf_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>( this );

	parameterSubscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
		"/parameter_events", 10, std::bind( &InitialOdometry::UpdateParameters, this, _1 ) );

	publisher_ = this->create_publisher<nav_msgs::msg::Odometry>( "odometry/initial", 10 );

	subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odometry/gps", 10, std::bind( &InitialOdometry::SubscribeGpsOdom, this, _1 ) );

	timer_ = this->create_wall_timer( 100ms, std::bind( &InitialOdometry::MainCycle, this ) );

	PublishParentStaticTf();
}

InitialOdometry::~InitialOdometry()
{
	// TODO
}

void InitialOdometry::MainLoop( void )
{
	// TODO
}

void InitialOdometry::MainCycle( void )
{
	if( !isValid_ )
	{
		PublishInitialOdom();
	}

	if( this->get_clock()->now() > timeInvalid_ ){
		isValid_ = false;
	}
}

void InitialOdometry::PublishInitialOdom( void )
{
	auto odom = nav_msgs::msg::Odometry();
	double initial_pose_x;
	double initial_pose_y;
	double initial_pose_a;

	initial_pose_x = 0.0;
	initial_pose_y = 0.0;
	initial_pose_a = 0.0;

	if( initial_pose_.size() == 3 )
	{
		initial_pose_x = initial_pose_[0];
		initial_pose_y = initial_pose_[1];
		initial_pose_a = initial_pose_[2] * ( M_PI / 180 );
	}

	odom.header.stamp = this->get_clock()->now();
	odom.header.frame_id = frame_id_;
	odom.child_frame_id = child_frame_id_;

	odom.pose.pose.position.x = initial_pose_x;
	odom.pose.pose.position.y = initial_pose_y;
	odom.pose.pose.position.z = 0.0;

	tf2::Quaternion quat;
	quat.setRPY( 0.0, 0.0, initial_pose_a );
	odom.pose.pose.orientation = tf2::toMsg( quat );

	if( publish_tf_ )
	{
		geometry_msgs::msg::TransformStamped tf_msg;

		tf_msg.transform.translation.x = odom.pose.pose.position.x;
		tf_msg.transform.translation.y = odom.pose.pose.position.y;
		tf_msg.transform.translation.z = odom.pose.pose.position.z;
		tf_msg.transform.rotation = odom.pose.pose.orientation;

		tf_msg.header.frame_id = frame_id_;
		tf_msg.child_frame_id = child_frame_id_;
		tf_msg.header.stamp = this->get_clock()->now();

		static_tf_br_->sendTransform( tf_msg );
	}
	publisher_->publish( odom );
}

void InitialOdometry::PublishParentStaticTf( void )
{
	if( !enable_parent_frame_ )
	{
		return;
	}

	double initial_pose_x;
	double initial_pose_y;
	double initial_pose_a;

	initial_pose_x = 0.0;
	initial_pose_y = 0.0;
	initial_pose_a = 0.0;

	if( parent_initial_pose_.size() == 3 )
	{
		initial_pose_x = parent_initial_pose_[0];
		initial_pose_y = parent_initial_pose_[1];
		initial_pose_a = parent_initial_pose_[2] * ( M_PI / 180 );
	}

	geometry_msgs::msg::TransformStamped tf_msg;

	tf_msg.header.stamp = this->get_clock()->now();
	tf_msg.header.frame_id = parent_frame_;
	tf_msg.child_frame_id = frame_id_;

	tf_msg.transform.translation.x = initial_pose_x;
	tf_msg.transform.translation.y = initial_pose_y;
	tf_msg.transform.translation.z = 0.0;

	tf2::Quaternion quat;
	quat.setRPY( 0.0, 0.0, initial_pose_a );
	tf_msg.transform.rotation = tf2::toMsg( quat );

	parent_static_tf_br_->sendTransform( tf_msg );
}

void InitialOdometry::SubscribeGpsOdom( const nav_msgs::msg::Odometry::SharedPtr msg )
{
	UNUSED_VARIABLE( msg );

	timeInvalid_ = this->get_clock()->now() + INVALID_TIME;
	isValid_ = true;
}

void InitialOdometry::UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event )
{
	if( event->node == this->get_fully_qualified_name() )
	{
		this->get_parameter( "frame_id", frame_id_ );
		this->get_parameter( "child_frame_id", child_frame_id_ );
		this->get_parameter( "initial_pose", initial_pose_ );
		this->get_parameter( "enable_parent_frame", enable_parent_frame_ );
		this->get_parameter( "parent_frame", parent_frame_ );
		this->get_parameter( "parent_initial_pose", parent_initial_pose_ );
	}

	for( const auto &param : event->changed_parameters )
	{
		if( param.name == "parent_initial_pose" )
		{
			PublishParentStaticTf();
		}
	}
}
