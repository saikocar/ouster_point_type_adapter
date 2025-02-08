#ifndef ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_
#define ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "autoware_point_types/types.hpp"

namespace ouster_point_type_adapter
{
class OusterPointTypeAdapter : public rclcpp::Node
{
public:
  explicit OusterPointTypeAdapter(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  //pcl::PointCloud<autoware_point_types::PointXYZIRADRT>::Ptr *output_pointcloud_;
  uint16_t current_frame_no;
  float max_intensity;
  bool first_run;
  uint64_t alloc_size;
  // parameters
  uint16_t calc_frame_no;
  int64_t scale_param;
  bool reflectivity_as_intensity;
};
}  // namespace ouster_point_type_adapter

#endif  // ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_
