/**
 * @file initial_odometry.cpp
 * 
 * @brief       initial_odometry
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2023/03/26
 * 
 * @copyright   (C) 2023 Motoyuki Endo
 */
#include "initial_odometry/initial_odometry.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define ARRAY_LENGTH(in_array)      (sizeof(in_array)/sizeof(in_array[0]))
#define UNUSED_VARIABLE(in_x)       (void)(in_x)

const rclcpp::Duration InitialOdometry::INVALID_TIME = rclcpp::Duration( 10s );

InitialOdometry::InitialOdometry()
	: Node( "initial_odometry" ),
	rosclock_( RCL_ROS_TIME )
{
	isValid_ = false;

	timeInvalid_ = rosclock_.now();

	std::vector<double> default_pose = { 0.0, 0.0, 0.0 };
	initialPose_ = this->declare_parameter( "initial_pose", default_pose );

	parameterSubscription_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
		"/parameter_events", 10, std::bind( &InitialOdometry::UpdateParameters, this, _1 ) );

	publisher_ = this->create_publisher<nav_msgs::msg::Odometry>( "odometry/initial", 10 );

	subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odometry/gps", 10, std::bind( &InitialOdometry::SubscribeGpsOdom, this, _1 ) );

	timer_ = this->create_wall_timer( 100ms, std::bind( &InitialOdometry::MainCycle, this ) );
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

	if( rosclock_.now() > timeInvalid_ ){
		isValid_ = false;
	}
}

void InitialOdometry::PublishInitialOdom(void)
{
	auto odom = nav_msgs::msg::Odometry();
	double initial_pose_x;
	double initial_pose_y;
	double initial_pose_a;

	initial_pose_x = 0.0;
	initial_pose_y = 0.0;
	initial_pose_a = 0.0;

	if( initialPose_.size() == 3 )
	{
		initial_pose_x = initialPose_[0];
		initial_pose_y = initialPose_[1];
		initial_pose_a = initialPose_[2] * ( M_PI / 180 );
	}

	odom.header.stamp = rosclock_.now();
	odom.header.frame_id = "map";
	odom.child_frame_id = "odom";

	odom.pose.pose.position.x = initial_pose_x;
	odom.pose.pose.position.y = initial_pose_y;
	odom.pose.pose.position.z = 0.0;

	tf2::Quaternion quat;
	quat.setRPY( 0.0, 0.0, initial_pose_a );
	odom.pose.pose.orientation = tf2::toMsg( quat );

	publisher_->publish( odom );
}

void InitialOdometry::SubscribeGpsOdom( const nav_msgs::msg::Odometry::SharedPtr msg )
{
	UNUSED_VARIABLE( msg );

	timeInvalid_ = rosclock_.now() + INVALID_TIME;
	isValid_ = true;
}

void InitialOdometry::UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event )
{
	if( event->node == this->get_fully_qualified_name() )
	{
		this->get_parameter( "initial_pose", initialPose_ );
	}
}
