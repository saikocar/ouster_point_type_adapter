#include "ouster_point_type_adapter/ouster_point_type_adapter_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "ouster_ros/include/ouster_ros/os_point.h"
#include "autoware_point_types/types.hpp"
#include <cmath>
#include <sstream>

namespace ouster_point_type_adapter
{
  OusterPointTypeAdapter::OusterPointTypeAdapter(const rclcpp::NodeOptions &options)
      : Node("ouster_point_type_adapter", options)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS{}.keep_last(1), std::bind(&OusterPointTypeAdapter::pointCloudCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::SensorDataQoS());
    this->declare_parameter("intensity_scale", (int64_t) 255);
    this->declare_parameter("reflectivity_as_intensity", (bool) false);
  }

  // based on https://github.com/autowarefoundation/autoware.universe/issues/4978#issuecomment-1971777511
  void OusterPointTypeAdapter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    // Instantiate output messages
    auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Instantiate pcl pointcloud message for the input point cloud
    pcl::PointCloud<ouster_ros::Point>::Ptr input_pointcloud(
        new pcl::PointCloud<ouster_ros::Point>);

    // Convert ros message to pcl
    pcl::fromROSMsg(*msg, *input_pointcloud);

    // Instantiate pcl pointcloud message for the output point cloud
    pcl::PointCloud<autoware_point_types::PointXYZIRADRT>::Ptr output_pointcloud(
        new pcl::PointCloud<autoware_point_types::PointXYZIRADRT>);
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = input_pointcloud->height;
    output_pointcloud->width = input_pointcloud->width;
    output_pointcloud->reserve(input_pointcloud->points.size());

    bool reflectivity_as_intensity = this->get_parameter("reflectivity_as_intensity").as_bool();

    bool first = true;
    float max_intensity = 0.0;
    float min_intensity = 0.0;
    uint32_t count = 0;
    for (const auto &point_in : input_pointcloud->points)
    {
      if (first) {
        if (reflectivity_as_intensity) {
        max_intensity = point_in.reflectivity;
        min_intensity = point_in.reflectivity;
        } else {
        max_intensity = point_in.intensity;
        min_intensity = point_in.intensity;
        }

        count++;
        first = false;
        continue;
      }

      if (reflectivity_as_intensity) {
        if (point_in.reflectivity > max_intensity) max_intensity = point_in.reflectivity;
        if (point_in.reflectivity < min_intensity) min_intensity = point_in.reflectivity;
      } else {
        if (point_in.intensity > max_intensity) max_intensity = point_in.intensity;
        if (point_in.intensity < min_intensity) min_intensity = point_in.intensity;
      }
      count++;
    }
    //std::stringstream ss;
    //ss << min_intensity << "~" <<max_intensity<< "c:"<<count<<"\n";
    //RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
    // Convert pcl from ouster to pcl autoware format

    uint8_t max_scale = 255;
    int64_t scale_param = this->get_parameter("intensity_scale").as_int();
    if (scale_param < 0) scale_param = 0;
    if (scale_param > 255) scale_param = 255;
    max_scale = scale_param;
    autoware_point_types::PointXYZIRADRT point_out{};
    for (const auto &point_in : input_pointcloud->points)
    {
      point_out.x = point_in.x;
      point_out.y = point_in.y;
      point_out.z = point_in.z;
      if (reflectivity_as_intensity) {
        point_out.intensity = uint8_t((point_in.reflectivity/max_intensity)*max_scale);
      } else {
        point_out.intensity = uint8_t((point_in.intensity/max_intensity)*max_scale);
      }
      point_out.return_type = 0;
      point_out.ring = point_in.ring;
      point_out.azimuth = std::atan2(point_in.y, point_in.x);
      point_out.distance = float(point_in.range) / 1000.0;
      point_out.time_stamp = point_in.t;
      output_pointcloud->points.emplace_back(point_out);
    }

    // Convert pcl to ros message
    pcl::toROSMsg(*output_pointcloud, *pointcloud_msg);
    pointcloud_msg->header.stamp = this->now();

    // Publish updated pointcloud message
    publisher_->publish(*pointcloud_msg);
  }

} // namespace ouster_point_type_adapter
RCLCPP_COMPONENTS_REGISTER_NODE(ouster_point_type_adapter::OusterPointTypeAdapter)
