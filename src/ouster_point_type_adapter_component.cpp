#include "ouster_point_type_adapter/ouster_point_type_adapter_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
// ouster_ros の install のヘッダの置き場は版で違う(実機は 2026-04-03 の入れ替えから include/ouster_ros/os_point.h)
#if __has_include("ouster_ros/os_point.h")
#include "ouster_ros/os_point.h"
#else
#include "ouster_ros/include/ouster_ros/os_point.h"
#endif
#include "autoware_point_types/types.hpp"
#include "dw_version.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace ouster_point_type_adapter
{
  OusterPointTypeAdapter::OusterPointTypeAdapter(const rclcpp::NodeOptions &options)
      : Node("ouster_point_type_adapter", options)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS{}.keep_last(1), std::bind(&OusterPointTypeAdapter::pointCloudCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::SensorDataQoS());
    this->declare_parameter("gamma", (double) 1.0);
    // 点を時刻順に並べ替えて出す(既定 true)。
    // ouster-ros は点群をリングごと(行優先)に詰めるので、そのままだと 1 スキャンの中で時刻が (リング数 - 1) 回巻き戻る。
    // Autoware の歪み補正(distortion_corrector)は点を並び順に、前の点との時刻差で車の動きを積算するため、
    // 巻き戻りのたびに横のずれが残り、横加速度に比例して点群が横へずれる(128 リングで 1 m/s^2 あたり平均 0.3〜0.4 m)。
    // また歪み補正の基準の時刻は配列の先頭の点の時刻なので、先頭を最も早い点にしておく。
    this->declare_parameter("sort_by_time", true);
    RCLCPP_INFO(this->get_logger(), "version %s", DW_VERSION);
    RCLCPP_INFO(this->get_logger(), "sort_by_time: %s", this->get_parameter("sort_by_time").as_bool() ? "true" : "false");
    //this->declare_parameter("reflectivity_as_intensity", (bool) false);
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
    output_pointcloud->is_dense = true;

    /*bool reflectivity_as_intensity = this->get_parameter("reflectivity_as_intensity").as_bool();

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
    }*/
    //std::stringstream ss;
    //ss << min_intensity << "~" <<max_intensity<< "c:"<<count<<"\n";
    //RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
    // Convert pcl from ouster to pcl autoware format

    //uint8_t max_scale = 255;
    //int64_t scale_param = this->get_parameter("intensity_scale").as_int();
    //if (scale_param < 0) scale_param = 0;
    //if (scale_param > 255) scale_param = 255;
    //max_scale = scale_param;
    float gamma = (float) this->get_parameter("gamma").as_double();
    bool gamma_adjust = (gamma == 1.0) ? false : true;
    autoware_point_types::PointXYZIRADRT point_out{};
    size_t points_count = 0;
    double prev_stamp = 0;
    double stamp_sum = 0;
    for (const auto &point_in : input_pointcloud->points)
    { 
      if (!std::isfinite(point_in.x) || !std::isfinite(point_in.y) || !std::isfinite(point_in.z)) continue; //remove NaNs
      point_out.x = point_in.x;
      point_out.y = point_in.y;
      point_out.z = point_in.z;
      point_out.intensity = (gamma_adjust) ? uint8_t(std::clamp(std::pow(255.0*(point_in.reflectivity/255.0), gamma), 0.0, 255.0)+0.5) : point_in.reflectivity;
      //if (reflectivity_as_intensity) {
      //  point_out.intensity = uint8_t((point_in.reflectivity/max_intensity)*max_scale);
      //} else {
      //  point_out.intensity = uint8_t((point_in.intensity/max_intensity)*max_scale);
      //}
      point_out.return_type = 0;
      point_out.ring = point_in.ring;
      point_out.azimuth = std::atan2(point_in.y, point_in.x);
      point_out.distance = float(point_in.range) / 1000.0;
      point_out.time_stamp = static_cast<double>(point_in.t) / 1e9 + rclcpp::Time(msg->header.stamp).seconds(); // convert nsec to sec
      output_pointcloud->points.emplace_back(point_out);
      points_count++;

      stamp_sum += static_cast<double>(point_in.t) / 1e9 - prev_stamp;
      prev_stamp = static_cast<double>(point_in.t) / 1e9;
    }
    if (this->get_parameter("sort_by_time").as_bool()) {
      // 同じ時刻(同じ列)の点は元の順(リング順)を保つ
      std::stable_sort(
        output_pointcloud->points.begin(), output_pointcloud->points.end(),
        [](const autoware_point_types::PointXYZIRADRT & a, const autoware_point_types::PointXYZIRADRT & b) {
          return a.time_stamp < b.time_stamp;
        });
    }
    if (output_pointcloud->size() != points_count) {
      output_pointcloud->resize(points_count);
      output_pointcloud->height = 1;
      output_pointcloud->width = points_count;
    }

    // Convert pcl to ros message
    pcl::toROSMsg(*output_pointcloud, *pointcloud_msg);
    pointcloud_msg->header.stamp = msg->header.stamp;//this->now();
    pointcloud_msg->height = 1;
    pointcloud_msg->width = points_count;

    // Publish updated pointcloud message
    publisher_->publish(*pointcloud_msg);

    RCLCPP_DEBUG_STREAM(get_logger(), "stamp_sum : " << stamp_sum);  // 毎スキャン INFO で出していたのを DEBUG に
  }

} // namespace ouster_point_type_adapter
RCLCPP_COMPONENTS_REGISTER_NODE(ouster_point_type_adapter::OusterPointTypeAdapter)