#include "ouster_point_type_adapter/ouster_point_type_adapter_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "ouster_ros/os_point.h"
#include <autoware/point_types/types.hpp>
#include <cmath>

namespace ouster_point_type_adapter
{
  OusterPointTypeAdapter::OusterPointTypeAdapter(const rclcpp::NodeOptions &options)
      : Node("ouster_point_type_adapter", options)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS{}.keep_last(1), std::bind(&OusterPointTypeAdapter::pointCloudCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::SensorDataQoS());
    this->declare_parameter("intensity_scale", (int64_t) 255);
    this->declare_parameter("reflectivity_as_intensity", (bool) false);
    this->declare_parameter("gamma", 1.0);
  }

  // based on https://github.com/autowarefoundation/autoware.universe/issues/4978#issuecomment-1971777511
  void OusterPointTypeAdapter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl::PointCloud<ouster_ros::Point>::Ptr input_pointcloud(
        new pcl::PointCloud<ouster_ros::Point>);
    pcl::fromROSMsg(*msg, *input_pointcloud);

    pcl::PointCloud<autoware::point_types::PointXYZIRCAEDT>::Ptr output_pointcloud(
        new pcl::PointCloud<autoware::point_types::PointXYZIRCAEDT>);
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = input_pointcloud->height;
    output_pointcloud->width = input_pointcloud->width;
    output_pointcloud->reserve(input_pointcloud->points.size());

    const bool reflectivity_as_intensity = this->get_parameter("reflectivity_as_intensity").as_bool();
    const double gamma = this->get_parameter("gamma").as_double();

    // Find max intensity/reflectivity for normalization
    float max_val = 0.0f;
    for (const auto &point_in : input_pointcloud->points)
    {
      float val = reflectivity_as_intensity ? point_in.reflectivity : point_in.intensity;
      if (val > max_val) max_val = val;
    }
    if (max_val == 0.0f) max_val = 1.0f;

    int64_t scale_param = this->get_parameter("intensity_scale").as_int();
    if (scale_param < 0) scale_param = 0;
    if (scale_param > 255) scale_param = 255;
    const uint8_t max_scale = static_cast<uint8_t>(scale_param);

    autoware::point_types::PointXYZIRCAEDT point_out{};
    for (const auto &point_in : input_pointcloud->points)
    {
      point_out.x = point_in.x;
      point_out.y = point_in.y;
      point_out.z = point_in.z;

      float raw_val = reflectivity_as_intensity ? point_in.reflectivity : point_in.intensity;
      float normalized = raw_val / max_val;
      if (gamma != 1.0) {
        normalized = std::pow(normalized, static_cast<float>(gamma));
      }
      point_out.intensity = static_cast<uint8_t>(normalized * max_scale);

      point_out.return_type = 0;
      point_out.channel = point_in.ring;
      point_out.azimuth = -std::atan2(point_in.y, point_in.x);
      point_out.elevation = std::atan2(point_in.z, std::sqrt(point_in.x * point_in.x + point_in.y * point_in.y));
      point_out.distance = static_cast<float>(point_in.range) / 1000.0f;
      point_out.time_stamp = point_in.t;
      output_pointcloud->points.emplace_back(point_out);
    }

    pcl::toROSMsg(*output_pointcloud, *pointcloud_msg);
    pointcloud_msg->header.stamp = this->now();
    publisher_->publish(*pointcloud_msg);
  }

} // namespace ouster_point_type_adapter
RCLCPP_COMPONENTS_REGISTER_NODE(ouster_point_type_adapter::OusterPointTypeAdapter)
