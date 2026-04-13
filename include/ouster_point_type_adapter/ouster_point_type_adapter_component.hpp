#ifndef ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_
#define ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace ouster_point_type_adapter
{
class OusterPointTypeAdapter : public rclcpp::Node
{
public:
  explicit OusterPointTypeAdapter(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

  // Diagnostic callback function
  void checkAll(diagnostic_updater::DiagnosticStatusWrapper & stat);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  std::map<std::string, std::string> field_mappings_;

  // Diagnostic updater
  diagnostic_updater::Updater updater_;

  // Diagnostic parameters (thresholds)
  double expected_frequency_;
  double frequency_tolerance_;
  int min_point_count_;
  double processing_time_warn_ms_;
  double pipeline_latency_warn_ms_;
  double pipeline_latency_error_ms_;
  double nan_inf_rate_warn_;
  double timestamp_diff_warn_ms_;

  // Diagnostic statistics (updated in callback, read in diagnostic functions)
  mutable std::mutex diag_mutex_;
  rclcpp::Time last_callback_time_;
  double current_frequency_hz_{0.0};
  int callback_count_{0};

  int input_point_count_{0};
  int output_point_count_{0};
  double pass_rate_{1.0};

  int nan_inf_count_{0};
  double nan_inf_rate_{0.0};

  double processing_time_ms_{0.0};
  double pipeline_latency_ms_{0.0};

  float intensity_max_val_{0.0f};
  bool intensity_all_zero_{false};

  bool timestamp_monotonic_{true};
  int timestamp_reversal_count_{0};
  double timestamp_range_ms_{0.0};

  bool fields_valid_{true};
  std::vector<std::string> missing_fields_;

  bool data_received_{false};
};
}  // namespace ouster_point_type_adapter

#endif  // ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_
