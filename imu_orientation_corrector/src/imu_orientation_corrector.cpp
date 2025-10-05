/**
 * @file imu_orientation_corrector.cpp
 * 
 * @brief       imu_orientation_corrector
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/05/11
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "imu_orientation_corrector/imu_orientation_corrector.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

ImuOrientationCorrector::ImuOrientationCorrector()
	: Node( "imu_orientation_corrector" )
{
	base_frame_ = this->declare_parameter<std::string>( "base_frame", "base_link");
	imu_frame_ = this->declare_parameter<std::string>( "imu_frame", "sensor_imu_frame" );
	use_average_calibration_ = this->declare_parameter<bool>( "use_average_calibration", true );
	collection_duration_sec_ = this->declare_parameter<double>( "collection_duration_sec", 3.0 );

	if( use_average_calibration_ )
	{
		RCLCPP_INFO( this->get_logger(),
			"Using average calibration (collect data for %.1f sec)",
			collection_duration_sec_
		);
	}
	else
	{
		RCLCPP_INFO( this->get_logger(),
			"Using single sample calibration (collect data for %.1f sec)",
			collection_duration_sec_
		);
	}

	is_transform_initialized_ = false;
	q_transform_ = tf2::Quaternion( 0.0, 0.0, 0.0, 1.0 );

	is_collecting_ = true;
	is_calibrated_ = false;
	collected_time_ = 0.0;
	last_imu_time_ = rclcpp::Time( 0 );

	roll_samples_.clear();
	pitch_samples_.clear();
	yaw_samples_.clear();

	initial_roll_ = 0.0;
	initial_pitch_ = 0.0;
	initial_yaw_ = 0.0;

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>( this->get_clock() );
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

	auto qos = rclcpp::SensorDataQoS();

	sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
		"/imu", qos, std::bind( &ImuOrientationCorrector::ImuCallback, this, _1 ) );

	pub_corrected_ = this->create_publisher<sensor_msgs::msg::Imu>( "/imu_corrected", 10);
}

ImuOrientationCorrector::~ImuOrientationCorrector()
{
	// TODO
}

void ImuOrientationCorrector::ImuCallback( const sensor_msgs::msg::Imu::SharedPtr msg )
{
	if( !is_transform_initialized_ )
	{
		try
		{
			geometry_msgs::msg::TransformStamped transform;

			transform = tf_buffer_->lookupTransform( base_frame_, imu_frame_, tf2::TimePointZero );
			tf2::fromMsg( transform.transform.rotation, q_transform_ );
			is_transform_initialized_ = true;
		}
		catch( tf2::TransformException &ex )
		{
			RCLCPP_WARN( this->get_logger(), "TF not available yet: %s", ex.what() );
			return;
		}
	}

	if( is_collecting_ )
	{
		tf2::Quaternion q_original;
		tf2::Quaternion q_rotated;
		double roll;
		double pitch;
		double yaw;

		tf2::fromMsg( msg->orientation, q_original );
		q_rotated = q_original * q_transform_.inverse();
		tf2::Matrix3x3( q_rotated ).getRPY( roll, pitch, yaw );

		if( last_imu_time_.nanoseconds() == 0 )
		{
			last_imu_time_ = this->now();
		}
		else
		{
			rclcpp::Duration dt = this->now() - last_imu_time_;
			collected_time_ += dt.seconds();
			last_imu_time_ = this->now();
		}

		if( use_average_calibration_ )
		{
			roll_samples_.push_back( roll );
			pitch_samples_.push_back( pitch );
			yaw_samples_.push_back( yaw );

			if( collected_time_ >= collection_duration_sec_ )
			{
				initial_roll_  = AverageAngle( roll_samples_ );
				initial_pitch_ = AverageAngle( pitch_samples_ );
				initial_yaw_   = AverageAngle( yaw_samples_ );

				is_collecting_ = false;
				is_calibrated_ = true;

				RCLCPP_INFO( this->get_logger(),
					"Calibraqtion done. Initial RPY offset: [%.3f, %.3f, %.3f] rad",
					initial_roll_, initial_pitch_, initial_yaw_
				);

				roll_samples_.clear();
				pitch_samples_.clear();
				yaw_samples_.clear();
			}
		}
		else
		{
			if( collected_time_ >= collection_duration_sec_ )
			{
				initial_roll_  = roll;
				initial_pitch_ = pitch;
				initial_yaw_   = yaw;

				is_collecting_ = false;
				is_calibrated_ = true;

				RCLCPP_INFO( this->get_logger(),
					"Single sample calibration done. Initial RPY offset: [%.3f, %.3f, %.3f] rad",
					initial_roll_, initial_pitch_, initial_yaw_
				);
			}
		}
	}

	if( is_calibrated_ )
	{
		tf2::Quaternion q_original;
		double roll;
		double pitch;
		double yaw;

		tf2::fromMsg( msg->orientation, q_original );
		tf2::Matrix3x3( q_original ).getRPY( roll, pitch, yaw );

		double corrected_roll  = NormalizeAngle( roll  - initial_roll_ );
		double corrected_pitch = NormalizeAngle( pitch - initial_pitch_ );
		double corrected_yaw   = NormalizeAngle( yaw   - initial_yaw_ );

		tf2::Quaternion q_corrected;
		q_corrected.setRPY( corrected_roll, corrected_pitch, corrected_yaw );
		q_corrected.normalize();

		auto corrected_msg = sensor_msgs::msg::Imu();
		corrected_msg.header = msg->header;
		corrected_msg.orientation.x = q_corrected.x();
		corrected_msg.orientation.y = q_corrected.y();
		corrected_msg.orientation.z = q_corrected.z();
		corrected_msg.orientation.w = q_corrected.w();

		corrected_msg.angular_velocity = msg->angular_velocity;
		corrected_msg.linear_acceleration = msg->linear_acceleration;

		corrected_msg.orientation_covariance = msg->orientation_covariance;
		corrected_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
		corrected_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;

		pub_corrected_->publish( corrected_msg );
	}
}

double ImuOrientationCorrector::NormalizeAngle( double angle )
{
	while( angle >  M_PI ) angle -= 2.0 * M_PI;
	while( angle < -M_PI ) angle += 2.0 * M_PI;
	return angle;
}

double ImuOrientationCorrector::AverageAngle( const std::vector<double>& angles )
{
	double sum_sin = 0.0;
	double sum_cos = 0.0;

	for( auto a : angles )
	{
		sum_sin += std::sin( a );
		sum_cos += std::cos( a );
	}

	return std::atan2( sum_sin / angles.size(), sum_cos / angles.size() );
}
