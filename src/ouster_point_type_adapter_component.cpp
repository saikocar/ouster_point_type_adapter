#include "ouster_point_type_adapter/ouster_point_type_adapter_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "ouster_ros/os_point.h"
#include <autoware/point_types/types.hpp>
#include <algorithm>
#include <cmath>

namespace ouster_point_type_adapter
{
  OusterPointTypeAdapter::OusterPointTypeAdapter(const rclcpp::NodeOptions &options)
      : Node("ouster_point_type_adapter", options),
        updater_(this, 0.1),
        last_callback_time_(this->now())
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS{}.keep_last(1), std::bind(&OusterPointTypeAdapter::pointCloudCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::SensorDataQoS());
    this->declare_parameter("intensity_scale", (int64_t) 255);
    this->declare_parameter("reflectivity_as_intensity", (bool) false);
    this->declare_parameter("gamma", 1.0);

    // Diagnostic parameters
    // 期待する受信周波数 [Hz]（例: Ouster OS1は10Hz or 20Hz）
    this->declare_parameter("expected_frequency", 10.0);
    // 受信周波数の許容誤差率（0.2 = ±20%）
    this->declare_parameter("frequency_tolerance", 0.2);
    // 入力点群の最小点数閾値。これ以下でWARN
    this->declare_parameter("min_point_count", 100);
    // pointCloudCallbackの処理時間WARN閾値 [ms]
    this->declare_parameter("processing_time_warn_ms", 50.0);
    // header.stampから現在時刻までのパイプライン遅延WARN閾値 [ms]
    this->declare_parameter("pipeline_latency_warn_ms", 150.0);
    // header.stampから現在時刻までのパイプライン遅延ERROR閾値 [ms]
    this->declare_parameter("pipeline_latency_error_ms", 200.0);
    // 入力点群中のNaN/Inf点の割合WARN閾値（0.1 = 10%）
    this->declare_parameter("nan_inf_rate_warn", 0.1);
    // 1フレーム内のtimestamp最大-最小の範囲WARN閾値 [ms]
    this->declare_parameter("timestamp_diff_warn_ms", 200.0);

    expected_frequency_ = this->get_parameter("expected_frequency").as_double();
    frequency_tolerance_ = this->get_parameter("frequency_tolerance").as_double();
    min_point_count_ = static_cast<int>(this->get_parameter("min_point_count").as_int());
    processing_time_warn_ms_ = this->get_parameter("processing_time_warn_ms").as_double();
    pipeline_latency_warn_ms_ = this->get_parameter("pipeline_latency_warn_ms").as_double();
    pipeline_latency_error_ms_ = this->get_parameter("pipeline_latency_error_ms").as_double();
    nan_inf_rate_warn_ = this->get_parameter("nan_inf_rate_warn").as_double();
    timestamp_diff_warn_ms_ = this->get_parameter("timestamp_diff_warn_ms").as_double();

    // Register diagnostic task
    updater_.setHardwareID(this->get_fully_qualified_name());
    updater_.add("point_type_adapter", this, &OusterPointTypeAdapter::checkAll);
  }

  // based on https://github.com/autowarefoundation/autoware.universe/issues/4978#issuecomment-1971777511
  void OusterPointTypeAdapter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    const auto processing_start = std::chrono::steady_clock::now();

    // --- Field existence check ---
    const std::vector<std::string> required_fields = {
      "x", "y", "z", "intensity", "reflectivity", "ring", "range", "t"};
    std::vector<std::string> local_missing_fields;
    for (const auto & field_name : required_fields) {
      bool found = false;
      for (const auto & field : msg->fields) {
        if (field.name == field_name) {
          found = true;
          break;
        }
      }
      if (!found) {
        local_missing_fields.push_back(field_name);
      }
    }

    if (!local_missing_fields.empty()) {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      fields_valid_ = false;
      missing_fields_ = local_missing_fields;
      data_received_ = true;
      return;
    }

    // --- Frequency measurement ---
    const auto now = this->now();
    {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      fields_valid_ = true;
      missing_fields_.clear();

      if (callback_count_ > 0) {
        const double dt = (now - last_callback_time_).seconds();
        if (dt > 0.0) {
          current_frequency_hz_ = 1.0 / dt;
        }
      }
      last_callback_time_ = now;
      callback_count_++;
    }

    // --- Pipeline latency ---
    const double local_pipeline_latency_ms =
      (now - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;

    auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl::PointCloud<ouster_ros::Point>::Ptr input_pointcloud(
        new pcl::PointCloud<ouster_ros::Point>);
    pcl::fromROSMsg(*msg, *input_pointcloud);

    const int local_input_count = static_cast<int>(input_pointcloud->points.size());

    // --- NaN/Inf check ---
    int local_nan_inf_count = 0;
    for (const auto & point : input_pointcloud->points) {
      if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
          std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z)) {
        local_nan_inf_count++;
      }
    }
    const double local_nan_inf_rate =
      (local_input_count > 0)
        ? static_cast<double>(local_nan_inf_count) / local_input_count
        : 0.0;

    pcl::PointCloud<autoware::point_types::PointXYZIRCAEDT>::Ptr output_pointcloud(
        new pcl::PointCloud<autoware::point_types::PointXYZIRCAEDT>);
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = input_pointcloud->points.size();
    output_pointcloud->reserve(output_pointcloud->width);

    const bool reflectivity_as_intensity = this->get_parameter("reflectivity_as_intensity").as_bool();
    const double gamma = this->get_parameter("gamma").as_double();

    // Find max intensity/reflectivity for normalization
    float max_val = 0.0f;
    for (const auto &point_in : input_pointcloud->points)
    {
      float val = reflectivity_as_intensity ? point_in.reflectivity : point_in.intensity;
      if (val > max_val) max_val = val;
    }
    const bool local_intensity_all_zero = (max_val == 0.0f);
    if (max_val == 0.0f) max_val = 1.0f;

    int64_t scale_param = this->get_parameter("intensity_scale").as_int();
    if (scale_param < 0) scale_param = 0;
    if (scale_param > 255) scale_param = 255;
    const uint8_t max_scale = static_cast<uint8_t>(scale_param);

    // --- Timestamp monotonicity check (pre-sort) ---
    int local_reversal_count = 0;
    double local_timestamp_range_ms = 0.0;
    bool local_timestamp_monotonic = true;
    if (input_pointcloud->points.size() > 1) {
      uint32_t min_ts = input_pointcloud->points[0].t;
      uint32_t max_ts = input_pointcloud->points[0].t;
      for (size_t i = 1; i < input_pointcloud->points.size(); i++) {
        if (input_pointcloud->points[i].t < input_pointcloud->points[i - 1].t) {
          local_reversal_count++;
        }
        if (input_pointcloud->points[i].t < min_ts) min_ts = input_pointcloud->points[i].t;
        if (input_pointcloud->points[i].t > max_ts) max_ts = input_pointcloud->points[i].t;
      }
      local_timestamp_range_ms = static_cast<double>(max_ts - min_ts) / 1e6;  // ns -> ms
      local_timestamp_monotonic = (local_reversal_count == 0);
    }

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

    // Sort by time_stamp so the distortion corrector sees monotonically increasing timestamps.
    // Without this, the organized cloud's row-major order causes timestamp jumps backward
    // at each ring boundary, which breaks the cumulative distortion correction.
    std::sort(output_pointcloud->points.begin(), output_pointcloud->points.end(),
      [](const autoware::point_types::PointXYZIRCAEDT &a,
         const autoware::point_types::PointXYZIRCAEDT &b) {
        return a.time_stamp < b.time_stamp;
      });

    const int local_output_count = static_cast<int>(output_pointcloud->points.size());

    pcl::toROSMsg(*output_pointcloud, *pointcloud_msg);
    pointcloud_msg->header.stamp = msg->header.stamp;
    publisher_->publish(*pointcloud_msg);

    // --- Processing time measurement ---
    const auto processing_end = std::chrono::steady_clock::now();
    const double local_processing_time_ms =
      std::chrono::duration<double, std::milli>(processing_end - processing_start).count();

    // --- Update diagnostic statistics ---
    {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      data_received_ = true;

      input_point_count_ = local_input_count;
      output_point_count_ = local_output_count;
      pass_rate_ = (local_input_count > 0)
        ? static_cast<double>(local_output_count) / local_input_count
        : 0.0;

      nan_inf_count_ = local_nan_inf_count;
      nan_inf_rate_ = local_nan_inf_rate;

      processing_time_ms_ = local_processing_time_ms;
      pipeline_latency_ms_ = local_pipeline_latency_ms;

      intensity_max_val_ = max_val;
      intensity_all_zero_ = local_intensity_all_zero;

      timestamp_monotonic_ = local_timestamp_monotonic;
      timestamp_reversal_count_ = local_reversal_count;
      timestamp_range_ms_ = local_timestamp_range_ms;
    }
  }

  // ==================== Diagnostic Callback ====================

  void OusterPointTypeAdapter::checkAll(
    diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(diag_mutex_);

    if (!data_received_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No data received");
      return;
    }

    int worst_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string worst_message = "OK";

    auto update_worst = [&](int level, const std::string & message) {
      if (level > worst_level) {
        worst_level = level;
        worst_message = message;
      }
    };

    // --- Topic frequency ---
    const double elapsed = (this->now() - last_callback_time_).seconds();
    const double timeout = (1.0 / expected_frequency_) * 3.0;
    stat.addf("Current frequency", "%.2f Hz", current_frequency_hz_);
    stat.addf("Expected frequency", "%.2f Hz", expected_frequency_);
    stat.addf("Time since last callback", "%.3f s", elapsed);

    if (elapsed > timeout) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "No data received for " + std::to_string(elapsed) + " s");
    } else if (
      current_frequency_hz_ < expected_frequency_ * (1.0 - frequency_tolerance_) ||
      current_frequency_hz_ > expected_frequency_ * (1.0 + frequency_tolerance_))
    {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Frequency out of tolerance: " + std::to_string(current_frequency_hz_) + " Hz");
    }

    // --- Input point count ---
    stat.addf("Input point count", "%d", input_point_count_);
    stat.addf("Minimum point threshold", "%d", min_point_count_);

    if (input_point_count_ == 0) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Input point cloud is empty");
    } else if (input_point_count_ < min_point_count_) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Input point count below threshold: " + std::to_string(input_point_count_));
    }

    // --- Pass rate ---
    stat.addf("Output point count", "%d", output_point_count_);
    stat.addf("Pass rate", "%.4f", pass_rate_);

    if (output_point_count_ == 0 && input_point_count_ > 0) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No valid output points");
    } else if (pass_rate_ < 0.5 && input_point_count_ > 0) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Low pass rate: " + std::to_string(pass_rate_));
    }

    // --- NaN/Inf rate ---
    stat.addf("NaN/Inf count", "%d", nan_inf_count_);
    stat.addf("NaN/Inf rate", "%.4f", nan_inf_rate_);
    stat.addf("NaN/Inf warn threshold", "%.4f", nan_inf_rate_warn_);

    if (nan_inf_rate_ > nan_inf_rate_warn_) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "High NaN/Inf rate: " + std::to_string(nan_inf_rate_));
    }

    // --- Processing time ---
    stat.addf("Processing time", "%.3f ms", processing_time_ms_);
    stat.addf("Processing time warn threshold", "%.3f ms", processing_time_warn_ms_);

    if (processing_time_ms_ > processing_time_warn_ms_) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Processing time exceeded threshold: " + std::to_string(processing_time_ms_) + " ms");
    }

    // --- Pipeline latency ---
    stat.addf("Pipeline latency", "%.3f ms", pipeline_latency_ms_);
    stat.addf("Pipeline latency warn threshold", "%.3f ms", pipeline_latency_warn_ms_);
    stat.addf("Pipeline latency error threshold", "%.3f ms", pipeline_latency_error_ms_);

    if (pipeline_latency_ms_ > pipeline_latency_error_ms_) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Pipeline latency critical: " + std::to_string(pipeline_latency_ms_) + " ms");
    } else if (pipeline_latency_ms_ > pipeline_latency_warn_ms_) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Pipeline latency high: " + std::to_string(pipeline_latency_ms_) + " ms");
    }

    // --- Intensity normalization ---
    stat.addf("Max intensity/reflectivity value", "%.2f", intensity_max_val_);
    stat.add("Intensity all zero", intensity_all_zero_ ? "true" : "false");

    if (intensity_all_zero_) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "All intensity/reflectivity values are zero");
    }

    // --- Timestamp monotonicity ---
    stat.add("Timestamp monotonic (pre-sort)", timestamp_monotonic_ ? "true" : "false");
    stat.addf("Timestamp reversal count", "%d", timestamp_reversal_count_);
    stat.addf("Timestamp range", "%.3f ms", timestamp_range_ms_);
    stat.addf("Timestamp range warn threshold", "%.3f ms", timestamp_diff_warn_ms_);

    if (timestamp_range_ms_ > timestamp_diff_warn_ms_) {
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Timestamp range too large: " + std::to_string(timestamp_range_ms_) + " ms");
    }

    // --- Input fields ---
    stat.add("Input fields valid", fields_valid_ ? "true" : "false");

    if (!fields_valid_) {
      std::string missing;
      for (size_t i = 0; i < missing_fields_.size(); i++) {
        if (i > 0) missing += ", ";
        missing += missing_fields_[i];
      }
      stat.add("Missing fields", missing);
      update_worst(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Missing required fields: " + missing);
    }

    stat.summary(worst_level, worst_message);
  }

} // namespace ouster_point_type_adapter
RCLCPP_COMPONENTS_REGISTER_NODE(ouster_point_type_adapter::OusterPointTypeAdapter)
