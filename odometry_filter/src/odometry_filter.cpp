/**
 * @file odometry_filter.cpp
 * 
 * @brief       odometry_filter
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/09/28
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "odometry_filter/odometry_filter.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

OdometryFilter::OdometryFilter()
	: Node( "odometry_filter" )
{
	std::vector<double_t> default_pose = { 0.0, 0.0, 0.0 };

	map_frame_ = this->declare_parameter<std::string>( "map_frame", "map" );
	odom_frame_ = this->declare_parameter<std::string>( "odom_frame", "odom" );
	base_link_frame_ = this->declare_parameter<std::string>( "base_link_frame", "base_link" );
	twist_publish_rate_ = this->declare_parameter<double_t>( "twist_publish_rate", 1.0 );
	initial_pose_ = this->declare_parameter( "initial_pose", default_pose );
	update_yaw_only_ = this->declare_parameter<bool>( "update_yaw_only", false );
	publish_tf_ = this->declare_parameter<bool>( "publish_tf", false );
	enable_odom_update_covariance_check_ = this->declare_parameter<bool>( "enable_odom_update_covariance_check", false );
	odom_update_covariance_sigma_threshold_ = this->declare_parameter<double_t>( "odom_update_covariance_sigma_threshold", 3.0 );

	tim_twist_publish_ = this->get_clock()->now() + rclcpp::Duration::from_seconds( 1.0 / twist_publish_rate_ );
	odom_ = nav_msgs::msg::Odometry();
	odom_.pose.pose.orientation.w = 1.0;
	lst_odom_ = {};

	SetOffsetTransform();

	is_odom_update_ = false;

	sub_parameter_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
		"/parameter_events", 10, std::bind( &OdometryFilter::UpdateParameters, this, _1 ) );
	sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/initialpose", 10, std::bind( &OdometryFilter::InitialPoseCallback, this, _1 ) );

	sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom/in", 10, std::bind( &OdometryFilter::OdometryCallback, this, _1 ) );
	sub_check_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom/odom_update_check", 10, std::bind( &OdometryFilter::OdometryUpdateCheckCallback, this, _1 ) );

	pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>( "odom/out", 10 );
	pub_twist_ = this->create_publisher<nav_msgs::msg::Odometry>( "odom/out_twist_resampler", 10 );

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>( this->get_clock() );
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>( *tf_buffer_ );
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>( this );

	timer_ = this->create_wall_timer( 10ms, std::bind( &OdometryFilter::MainCycle, this ) );
}

OdometryFilter::~OdometryFilter()
{
	// TODO
}

void OdometryFilter::MainCycle( void )
{
	auto odom = odom_;

	if( this->get_clock()->now() >= tim_twist_publish_ )
	{
		tim_twist_publish_ = this->get_clock()->now() + rclcpp::Duration::from_seconds( 1.0 / twist_publish_rate_ );

		if( lst_odom_.size() > 0 )
		{
			nav_msgs::msg::Odometry &prev = lst_odom_.back();

			double_t dt = ( rclcpp::Time(odom.header.stamp) - rclcpp::Time(prev.header.stamp) ).seconds();

			tf2::Transform t_previous;
			tf2::fromMsg( prev.pose.pose, t_previous );

			tf2::Transform t_current;
			tf2::fromMsg( odom.pose.pose, t_current );

			tf2::Transform t_relative = t_previous.inverse() * t_current;

			double_t dx = t_relative.getOrigin().x();
			double_t dy = t_relative.getOrigin().y();
			double_t dz = t_relative.getOrigin().z();

			tf2::Quaternion q_previous;
			tf2::fromMsg( prev.pose.pose.orientation, q_previous );

			tf2::Quaternion q_current;
			tf2::fromMsg( odom.pose.pose.orientation, q_current );

			tf2::Quaternion q_relative = q_previous.inverse() * q_current;

			double_t axis_x = q_relative.getAxis().x();
			double_t axis_y = q_relative.getAxis().y();
			double_t axis_z = q_relative.getAxis().z();

			double_t angle = q_relative.getAngle();

			double_t vx = dx / dt;
			double_t vy = dy / dt;
			double_t vz = dz / dt;

			double_t wx = axis_x * angle / dt;
			double_t wy = axis_y * angle / dt;
			double_t wz = axis_z * angle / dt;


			odom.header.stamp = this->get_clock()->now();

			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.linear.z = vz;
			odom.twist.twist.angular.x = wx;
			odom.twist.twist.angular.y = wy;
			odom.twist.twist.angular.z = wz;
		}

		pub_twist_->publish( odom );

		if( lst_odom_.size() >= 3 )
		{
			lst_odom_.pop_front();
		}
		lst_odom_.push_back( odom );
	}
}

void OdometryFilter::OdometryCallback( const nav_msgs::msg::Odometry::SharedPtr msg )
{
	auto odom = *msg;

	geometry_msgs::msg::TransformStamped odom_to_base_msg;
	try
	{
		odom_to_base_msg = tf_buffer_->lookupTransform( odom_frame_, base_link_frame_, tf2::TimePointZero );
	}
	catch( tf2::TransformException &ex )
	{
		RCLCPP_WARN( this->get_logger(), "TF not available yet: %s", ex.what() );
		return;
	}

	geometry_msgs::msg::TransformStamped map_to_base_msg;
	if( !is_odom_update_ )
	{
		try
		{
			map_to_base_msg = tf_buffer_->lookupTransform( map_frame_, base_link_frame_, tf2::TimePointZero );
		}
		catch( tf2::TransformException &ex )
		{
			RCLCPP_WARN( this->get_logger(), "TF not available yet: %s", ex.what() );
			return;
		}
	}

	tf2::Transform map_to_base;
	tf2::fromMsg( odom.pose.pose, map_to_base );

	tf2::Transform corrected_map_to_base;
	if( is_odom_update_ )
	{
		corrected_map_to_base = map_to_base * offset_tf_;
	}
	else
	{
		tf2::fromMsg( map_to_base_msg.transform, corrected_map_to_base );
	}

	tf2::Transform odom_to_base;
	tf2::fromMsg( odom_to_base_msg.transform, odom_to_base );

	tf2::Transform map_to_odom;
	map_to_odom = corrected_map_to_base * odom_to_base.inverse();

	{
		odom.header.stamp = this->get_clock()->now();
		odom.header.frame_id = map_frame_;
		odom.child_frame_id = base_link_frame_;

		geometry_msgs::msg::Pose corrected_pose;
		tf2::toMsg( corrected_map_to_base, corrected_pose );
		odom.pose.pose = corrected_pose;
	}

	if( publish_tf_ )
	{
		geometry_msgs::msg::TransformStamped tf_msg;

		tf_msg.transform = tf2::toMsg( map_to_odom );

		tf_msg.header.frame_id = map_frame_;
		tf_msg.child_frame_id = odom_frame_;
		tf_msg.header.stamp = this->get_clock()->now();

		tf_broadcaster_->sendTransform( tf_msg );
	}

	pub_odom_->publish( odom );

	odom_ = odom;
}

void OdometryFilter::OdometryUpdateCheckCallback( const nav_msgs::msg::Odometry::SharedPtr msg )
{
	auto odom = *msg;
	bool is_update = true;
	double sigma_horizontal = 0.0;

	sigma_horizontal = std::sqrt( odom.pose.covariance[0] + odom.pose.covariance[7] );

	if( enable_odom_update_covariance_check_ )
	{
		if( sigma_horizontal >= odom_update_covariance_sigma_threshold_ )
		{
			is_update = false;
		}
	}

	is_odom_update_ = is_update;
}

void OdometryFilter::SetOffsetTransform( void )
{
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

	tf2::Quaternion q;
	q.setRPY( 0.0, 0.0, initial_pose_a );
	offset_tf_.setOrigin( tf2::Vector3( initial_pose_x, initial_pose_y, 0.0 ) );
	offset_tf_.setRotation( q );
}

void OdometryFilter::InitialPoseCallback( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg )
{
	auto odom = *msg;

	geometry_msgs::msg::TransformStamped map_to_base_msg;
	try
	{
		map_to_base_msg = tf_buffer_->lookupTransform( map_frame_, base_link_frame_, tf2::TimePointZero );
	}
	catch( tf2::TransformException &ex )
	{
		RCLCPP_WARN( this->get_logger(), "TF not available yet: %s", ex.what() );
		return;
	}

	tf2::Transform map_to_base;
	tf2::fromMsg( map_to_base_msg.transform, map_to_base );

	tf2::Transform update_pose;
	tf2::fromMsg( odom.pose.pose, update_pose );

	tf2::Transform corrected_offset;
	corrected_offset = update_pose * map_to_base.inverse();
	corrected_offset = corrected_offset * offset_tf_.inverse();

	geometry_msgs::msg::Pose corrected_pose;
	tf2::toMsg( corrected_offset, corrected_pose );

	double x = corrected_pose.position.x;
	double y = corrected_pose.position.y;
	double yaw = tf2::getYaw( corrected_pose.orientation ) * ( 180 / M_PI );

	if( update_yaw_only_ )
	{
		x = initial_pose_[0];
		y = initial_pose_[y];
	}

	std::vector<double_t> update_offset = { x, y, yaw };

	this->set_parameter( rclcpp::Parameter( "initial_offset", update_offset ) );
}

void OdometryFilter::UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event )
{
	if( event->node == this->get_fully_qualified_name() )
	{
		this->get_parameter( "map_frame", map_frame_ );
		this->get_parameter( "odom_frame", odom_frame_ );
		this->get_parameter( "base_link_frame", base_link_frame_ );
		this->get_parameter( "twist_publish_rate", twist_publish_rate_ );
		this->get_parameter( "initial_pose", initial_pose_ );
		this->get_parameter( "update_yaw_only", update_yaw_only_ );
		this->get_parameter( "publish_tf", publish_tf_ );
		this->get_parameter( "enable_odom_update_covariance_check", enable_odom_update_covariance_check_ );
		this->get_parameter( "odom_update_covariance_sigma_threshold", odom_update_covariance_sigma_threshold_ );
	}

	for( const auto &param : event->changed_parameters )
	{
		if( param.name == "initial_pose" )
		{
			SetOffsetTransform();
		}
	}
}
