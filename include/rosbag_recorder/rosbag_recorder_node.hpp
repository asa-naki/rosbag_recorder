#ifndef ROSBAG_RECORDER__ROSBAG_RECORDER_NODE_HPP_
#define ROSBAG_RECORDER__ROSBAG_RECORDER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <regex>
#include <chrono>
#include <thread>
#include <atomic>
#include <map>
#include <filesystem>
#include <cstdlib>
#include <signal.h>
#include <sys/wait.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>

// ROS2 Distro specific includes
#ifdef ROS_DISTRO_JAZZY
#include <rosbag2_transport/record_options.hpp>
#elif defined(ROS_DISTRO_HUMBLE)
// Humble doesn't have service recording in rosbag2_transport
#endif

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace rosbag_recorder
{

  struct RecordingConfig
  {
    std::string output_directory = "/media/ssd/rosbag_recordings";
    std::string bag_name_prefix = "recording";
    int split_duration_minutes = 1;
    std::string storage_id = "sqlite3";
    std::string serialization_format = "cdr";
    int max_bagfile_size = 0;               // 0 means no size limit
    int max_cache_size = 100 * 1024 * 1024; // 100MB
    std::vector<std::string> topics;
    std::vector<std::string> topic_regex_patterns;
    bool include_hidden_topics = false;

#ifdef ROS_DISTRO_JAZZY
    std::vector<std::string> services;
    bool record_services = true;
#elif defined(ROS_DISTRO_HUMBLE)
    // Services recording not supported in Humble
    bool record_services = false;
#endif
  };

  class RosbagRecorderNode : public rclcpp::Node
  {
  public:
    explicit RosbagRecorderNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~RosbagRecorderNode();

  private:
    // Service callbacks
    void start_recording_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void stop_recording_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Configuration methods
    void load_config();
    void load_config_from_yaml(const std::string &yaml_path);

    // Recording methods
    void start_recording();
    void stop_recording();
    void recording_thread_function();
    void split_bag_file();

    // Health monitoring methods
    void publish_status();
    void check_storage_health();
    bool is_storage_available();
    void publish_diagnostics();

    // Topic discovery methods
    std::vector<std::string> discover_topics();
    std::vector<std::string> filter_topics_by_regex(const std::vector<std::string> &all_topics);
    bool matches_any_pattern(const std::string &topic, const std::vector<std::string> &patterns);

    // Utility methods
    std::string generate_bag_filename();
    void create_output_directory();

    // ROS2 services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

    // Publishers for status and diagnostics
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recording_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recording_info_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    // Recording state
    std::atomic<bool> is_recording_{false};
    std::atomic<bool> should_stop_recording_{false};
    std::atomic<bool> storage_healthy_{true};
    std::thread recording_thread_;

    // Rosbag writer
    std::unique_ptr<rosbag2_cpp::Writer> writer_;

#ifdef ROS_DISTRO_JAZZY
    // Process-based recording using subprocess
    std::unique_ptr<std::thread> recording_process_thread_;
    std::atomic<bool> recording_process_running_{false};
#endif

    // Configuration
    RecordingConfig config_;

    // Timer for splitting bags and health checks
    rclcpp::TimerBase::SharedPtr split_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr health_check_timer_;
    std::chrono::steady_clock::time_point recording_start_time_;

    // Current recording session info
    std::string current_bag_path_;
    int bag_file_counter_;
    std::string last_error_message_;
    uint64_t total_bytes_written_;
    size_t total_messages_recorded_;

#ifdef ROS_DISTRO_JAZZY
    // Process management for external rosbag record
    pid_t recording_process_pid_;
    std::atomic<bool> recording_process_active_{false};
#endif
  };

} // namespace rosbag_recorder

#endif // ROSBAG_RECORDER__ROSBAG_RECORDER_NODE_HPP_
