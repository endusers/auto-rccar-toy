/**
 * @file pointcloud_filter.cpp
 * 
 * @brief       pointcloud_filter
 * @note        なし
 * 
 * @version     1.0.0
 * @date        2025/11/16
 * 
 * @copyright   (C) 2025 Motoyuki Endo
 */
#include "pointcloud_filter/pointcloud_filter.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

PointcloudFilter::PointcloudFilter()
    : Node( "pointcloud_filter" )
{
    min_angle_deg_ = this->declare_parameter<double_t>( "min_angle_deg", -135.0 );
    max_angle_deg_ = this->declare_parameter<double_t>( "max_angle_deg", 135.0 );
    frame_id_ = this->declare_parameter<std::string>( "frame_id", "" );

    min_angle_ = min_angle_deg_ * M_PI / 180.0;
    max_angle_ = max_angle_deg_ * M_PI / 180.0;

    auto qos = rclcpp::SensorDataQoS();

    sub_parameter_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10, std::bind( &PointcloudFilter::UpdateParameters, this, _1 ) );

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points/in", qos, std::bind( &PointcloudFilter::PointCloudCallback, this, _1 ) );

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/points/out", 10);
}

PointcloudFilter::~PointcloudFilter()
{
    // TODO
}

void PointcloudFilter::PointCloudCallback( const sensor_msgs::msg::PointCloud2::SharedPtr msg )
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointCloud<pcl::PointXYZI> filtered;
    sensor_msgs::msg::PointCloud2 output;

    pcl::fromROSMsg( *msg, cloud );
    filtered.reserve( cloud.size() );

    for( const auto &p : cloud.points )
    {
        bool is_inside = true;
        float angle = std::atan2( p.y, p.x );

        if( min_angle_ <= max_angle_ )
        {
            is_inside =  (angle >= min_angle_) and (angle <= max_angle_);
        }
        else
        {
            is_inside = (angle >= min_angle_) or (angle <= max_angle_);
        }

        if( is_inside )
        {
            filtered.push_back(p);
        }
    }

    pcl::toROSMsg( filtered, output );
    output.header = msg->header;

    if( !frame_id_.empty() )
    {
        output.header.frame_id = frame_id_;
    }

    pub_->publish( output );
}

void PointcloudFilter::UpdateParameters( const rcl_interfaces::msg::ParameterEvent::SharedPtr event )
{
    if( event->node == this->get_fully_qualified_name() )
    {
        this->get_parameter( "min_angle_deg", min_angle_deg_ );
        this->get_parameter( "max_angle_deg", max_angle_deg_ );
        this->get_parameter( "frame_id", frame_id_ );
    }

    for( const auto &param : event->changed_parameters )
    {
        if( param.name == "min_angle_deg" )
        {
            min_angle_ = min_angle_deg_ * M_PI / 180.0;
        }

        if( param.name == "max_angle_deg" )
        {
            max_angle_ = max_angle_deg_ * M_PI / 180.0;
        }
    }
}
