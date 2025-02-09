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
    scale_param = this->get_parameter("intensity_scale").as_int();
    if (scale_param < 0) scale_param = 0;
    if (scale_param > 255) scale_param = 255;

    this->declare_parameter("reflectivity_as_intensity", (bool) false);
    reflectivity_as_intensity = this->get_parameter("reflectivity_as_intensity").as_bool();

    this->declare_parameter("intensity_calculation_frameno", (uint16_t) 10);
    calc_frame_no = this->get_parameter("intensity_calculation_frameno").as_int();

    current_frame_no = 0;
    max_intensity = 0.0f;
    first_run = true;
    alloc_size = 0;
  }

  // based on https://github.com/autowarefoundation/autoware.universe/issues/4978#issuecomment-1971777511
  void OusterPointTypeAdapter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr input_msg)
  {
    // instantiate input pointcloud
    pcl::PointCloud<ouster_ros::Point>::Ptr input_pointcloud(new pcl::PointCloud<ouster_ros::Point>);
    pcl::fromROSMsg(*input_msg, *input_pointcloud);

    // maximum intensity range calculation only every x frame as defined as calc_frame_no (default is 1s if lidar is at 10Hz, for example)
    if (current_frame_no == 0) {
      bool first_point_in_cloud = true;
      max_intensity = 0.0;
      for (const auto &point_in : input_pointcloud->points)
      {
        float intensity = (reflectivity_as_intensity) ? point_in.reflectivity : point_in.intensity;
        if (first_point_in_cloud) {
          max_intensity = intensity;
          first_point_in_cloud = false;
          continue;
        }
        if (intensity > max_intensity) max_intensity = intensity;
      }
    }
    current_frame_no++;
    if (current_frame_no >= calc_frame_no) current_frame_no = 0;

    // instantiate output point cloud
    pcl::PointCloud<autoware_point_types::PointXYZIRADRT>::Ptr output_pointcloud(new pcl::PointCloud<autoware_point_types::PointXYZIRADRT>);
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = input_pointcloud->height;
    output_pointcloud->width = input_pointcloud->width; // we're copying the size because otherwise pcl computes it every frame
    if (first_run) { // determine allocation size of vector only upon first frame
      alloc_size = input_pointcloud->points.size();
      first_run = false;
    }
    output_pointcloud->reserve(alloc_size);

    // convert from ouster to autoware format: X, Y, Z, intensity, ring, azimuth, distance, return type(unused, only for dual return), timestamp
    autoware_point_types::PointXYZIRADRT point_out{}; // PointXYZIRCAEDT -> PointXYZIRADRT
    for (const auto &point_in : input_pointcloud->points)
    {
      float intensity = (reflectivity_as_intensity) ? point_in.reflectivity : point_in.intensity;
      point_out.x = point_in.x;
      point_out.y = point_in.y;
      point_out.z = point_in.z;
      // this calculation is necessary because velodyne models return uint8 even though autoware's intensity type is float
      // i.e. this is for centerpoint compatibility
      point_out.intensity = uint8_t((intensity/max_intensity)*scale_param);
      point_out.ring = point_in.ring;
      point_out.azimuth = std::atan2(point_in.y, point_in.x);
      point_out.distance = float(point_in.range) / 1000.0;
      point_out.time_stamp = point_in.t;
      output_pointcloud->points.push_back(point_out);
    }

    // Convert pcl to ros message & publish
    auto output_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*output_pointcloud, *output_msg);
    output_msg->header = input_msg->header;
    publisher_->publish(std::move(output_msg));
  }

} // namespace ouster_point_type_adapter
RCLCPP_COMPONENTS_REGISTER_NODE(ouster_point_type_adapter::OusterPointTypeAdapter)
