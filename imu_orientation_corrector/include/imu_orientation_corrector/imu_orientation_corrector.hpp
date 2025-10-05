/**
 * @file imu_orientation_corrector.hpp
 * 
 * @brief       imu_orientation_corrector
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/05/11
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class ImuOrientationCorrector : public rclcpp::Node
{
	public:
		ImuOrientationCorrector();
		~ImuOrientationCorrector();

	private:
		std::string base_frame_;
		std::string imu_frame_;
		bool use_average_calibration_;
		double collection_duration_sec_;

		bool is_transform_initialized_;
		tf2::Quaternion q_transform_;

		bool is_collecting_;
		bool is_calibrated_;
		double collected_time_;
		rclcpp::Time last_imu_time_;

		std::vector<double> roll_samples_;
		std::vector<double> pitch_samples_;
		std::vector<double> yaw_samples_;

		double initial_roll_;
		double initial_pitch_;
		double initial_yaw_;

		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_corrected_;

		void ImuCallback( const sensor_msgs::msg::Imu::SharedPtr msg );
		double NormalizeAngle( double angle );
		double AverageAngle( const std::vector<double>& angles );
};
