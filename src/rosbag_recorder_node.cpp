#include "rosbag_recorder/rosbag_recorder_node.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <fstream>

namespace rosbag_recorder
{

  RosbagRecorderNode::RosbagRecorderNode(const rclcpp::NodeOptions &options)
      : Node("rosbag_recorder", options), bag_file_counter_(0),
        total_bytes_written_(0), total_messages_recorded_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing ROS2 Rosbag Recorder Node");

    // Load configuration
    load_config();

    // Create services
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/start_recording",
        std::bind(&RosbagRecorderNode::start_recording_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stop_recording",
        std::bind(&RosbagRecorderNode::stop_recording_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Create publishers for status monitoring
    recording_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "~/recording_status", 10);
    recording_info_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/recording_info", 10);
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);

    // Create timers for status updates and health checks
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&RosbagRecorderNode::publish_status, this));

    health_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&RosbagRecorderNode::check_storage_health, this));

    RCLCPP_INFO(this->get_logger(), "ROS2 Rosbag Recorder Node initialized");
    RCLCPP_INFO(this->get_logger(), "Output directory: %s", config_.output_directory.c_str());
    RCLCPP_INFO(this->get_logger(), "Split duration: %d minutes", config_.split_duration_minutes);
  }

  RosbagRecorderNode::~RosbagRecorderNode()
  {
    if (is_recording_)
    {
      stop_recording();
    }

    // Cancel all timers
    if (status_timer_)
    {
      status_timer_->cancel();
    }
    if (health_check_timer_)
    {
      health_check_timer_->cancel();
    }
  }

  void RosbagRecorderNode::start_recording_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (is_recording_)
    {
      response->success = false;
      response->message = "Recording is already in progress";
      RCLCPP_WARN(this->get_logger(), "Recording start requested but already recording");
      return;
    }

    // Check storage availability before starting
    if (!is_storage_available())
    {
      response->success = false;
      response->message = "Storage not available: " + last_error_message_;
      RCLCPP_ERROR(this->get_logger(), "Cannot start recording: %s", last_error_message_.c_str());
      return;
    }

    try
    {
      start_recording();
      response->success = true;
      response->message = "Recording started successfully";
      RCLCPP_INFO(this->get_logger(), "Recording started via service call");
    }
    catch (const std::exception &e)
    {
      response->success = false;
      response->message = std::string("Failed to start recording: ") + e.what();
      last_error_message_ = e.what();
      RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
    }
  }

  void RosbagRecorderNode::stop_recording_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!is_recording_)
    {
      response->success = false;
      response->message = "No recording in progress";
      RCLCPP_WARN(this->get_logger(), "Recording stop requested but not recording");
      return;
    }

    try
    {
      stop_recording();
      response->success = true;
      response->message = "Recording stopped successfully";
      RCLCPP_INFO(this->get_logger(), "Recording stopped via service call");
    }
    catch (const std::exception &e)
    {
      response->success = false;
      response->message = std::string("Failed to stop recording: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Failed to stop recording: %s", e.what());
    }
  }

  void RosbagRecorderNode::load_config()
  {
    // Load parameters from ROS2 parameter server
    this->declare_parameter<std::string>("output_directory", config_.output_directory);
    this->declare_parameter<std::string>("bag_name_prefix", config_.bag_name_prefix);
    this->declare_parameter<int>("split_duration_minutes", config_.split_duration_minutes);
    this->declare_parameter<std::string>("storage_id", config_.storage_id);
    this->declare_parameter<std::string>("serialization_format", config_.serialization_format);
    this->declare_parameter<int>("max_bagfile_size", config_.max_bagfile_size);
    this->declare_parameter<int>("max_cache_size", config_.max_cache_size);
    this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{"/rosout"});
    this->declare_parameter<std::vector<std::string>>("topic_regex_patterns", std::vector<std::string>());
    this->declare_parameter<bool>("include_hidden_topics", config_.include_hidden_topics);

#ifdef ROS_DISTRO_JAZZY
    this->declare_parameter<std::vector<std::string>>("services", std::vector<std::string>());
    this->declare_parameter<bool>("record_services", config_.record_services);
#elif defined(ROS_DISTRO_HUMBLE)
    // Services recording not supported in Humble
    this->declare_parameter<bool>("record_services", false);
#endif

    this->declare_parameter<std::string>("config_file", std::string(""));

    config_.output_directory = this->get_parameter("output_directory").as_string();
    config_.bag_name_prefix = this->get_parameter("bag_name_prefix").as_string();
    config_.split_duration_minutes = this->get_parameter("split_duration_minutes").as_int();
    config_.storage_id = this->get_parameter("storage_id").as_string();
    config_.serialization_format = this->get_parameter("serialization_format").as_string();
    config_.max_bagfile_size = this->get_parameter("max_bagfile_size").as_int();
    config_.max_cache_size = this->get_parameter("max_cache_size").as_int();
    config_.topics = this->get_parameter("topics").as_string_array();
    config_.topic_regex_patterns = this->get_parameter("topic_regex_patterns").as_string_array();
    config_.include_hidden_topics = this->get_parameter("include_hidden_topics").as_bool();

#ifdef ROS_DISTRO_JAZZY
    config_.services = this->get_parameter("services").as_string_array();
    config_.record_services = this->get_parameter("record_services").as_bool();
#elif defined(ROS_DISTRO_HUMBLE)
    // Services recording not supported in Humble
    config_.record_services = false;
#endif

    // Load from YAML file if specified
    std::string config_file = this->get_parameter("config_file").as_string();
    if (!config_file.empty())
    {
      try
      {
        if (!std::filesystem::path(config_file).is_absolute())
        {
          // Try to find the file in the package share directory
          std::string package_share_dir = ament_index_cpp::get_package_share_directory("rosbag_recorder");
          config_file = std::filesystem::path(package_share_dir) / "config" / config_file;
        }
        load_config_from_yaml(config_file);
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to load config file %s: %s", config_file.c_str(), e.what());
      }
    }
  }

  void RosbagRecorderNode::load_config_from_yaml(const std::string &yaml_path)
  {
    RCLCPP_INFO(this->get_logger(), "Loading configuration from: %s", yaml_path.c_str());

    YAML::Node config = YAML::LoadFile(yaml_path);

    if (config["recording"])
    {
      auto recording_config = config["recording"];

      if (recording_config["output_directory"])
      {
        config_.output_directory = recording_config["output_directory"].as<std::string>();
      }
      if (recording_config["bag_name_prefix"])
      {
        config_.bag_name_prefix = recording_config["bag_name_prefix"].as<std::string>();
      }
      if (recording_config["split_duration_minutes"])
      {
        config_.split_duration_minutes = recording_config["split_duration_minutes"].as<int>();
      }
      if (recording_config["storage_id"])
      {
        config_.storage_id = recording_config["storage_id"].as<std::string>();
      }
      if (recording_config["serialization_format"])
      {
        config_.serialization_format = recording_config["serialization_format"].as<std::string>();
      }
      if (recording_config["max_bagfile_size"])
      {
        config_.max_bagfile_size = recording_config["max_bagfile_size"].as<int>();
      }
      if (recording_config["max_cache_size"])
      {
        config_.max_cache_size = recording_config["max_cache_size"].as<int>();
      }
      if (recording_config["include_hidden_topics"])
      {
        config_.include_hidden_topics = recording_config["include_hidden_topics"].as<bool>();
      }
      if (recording_config["record_services"])
      {
        config_.record_services = recording_config["record_services"].as<bool>();
      }

      if (recording_config["topics"])
      {
        config_.topics.clear();
        for (const auto &topic : recording_config["topics"])
        {
          config_.topics.push_back(topic.as<std::string>());
        }
      }

      if (recording_config["topic_regex_patterns"])
      {
        config_.topic_regex_patterns.clear();
        for (const auto &pattern : recording_config["topic_regex_patterns"])
        {
          config_.topic_regex_patterns.push_back(pattern.as<std::string>());
        }
      }

#ifdef ROS_DISTRO_JAZZY
      if (recording_config["services"])
      {
        config_.services.clear();
        for (const auto &service : recording_config["services"])
        {
          config_.services.push_back(service.as<std::string>());
        }
      }
#endif
    }
  }

  void RosbagRecorderNode::start_recording()
  {
    if (is_recording_)
    {
      RCLCPP_WARN(this->get_logger(), "Recording already in progress");
      return;
    }

    create_output_directory();

    // Reset state
    should_stop_recording_ = false;
    bag_file_counter_ = 0;
    recording_start_time_ = std::chrono::steady_clock::now();

    // Start recording in separate thread
    recording_thread_ = std::thread(&RosbagRecorderNode::recording_thread_function, this);

    is_recording_ = true;

    RCLCPP_INFO(this->get_logger(), "Recording started to directory: %s", config_.output_directory.c_str());
  }

  void RosbagRecorderNode::stop_recording()
  {
    if (!is_recording_)
    {
      RCLCPP_WARN(this->get_logger(), "No recording in progress");
      return;
    }

    should_stop_recording_ = true;

    if (recording_thread_.joinable())
    {
      recording_thread_.join();
    }

#ifdef ROS_DISTRO_JAZZY
    // Stop the recording process
    if (recording_process_active_)
    {
      // Kill all ros2 bag record processes
      system("pkill -f 'ros2 bag record'");
      recording_process_active_ = false;
    }
#endif

    if (writer_)
    {
      writer_.reset();
    }

    if (split_timer_)
    {
      split_timer_->cancel();
      split_timer_.reset();
    }

    is_recording_ = false;

    RCLCPP_INFO(this->get_logger(), "Recording stopped");
  }

  void RosbagRecorderNode::recording_thread_function()
  {
    try
    {
      // Create new bag file
      split_bag_file();

      // Set up timer for splitting bags
      auto split_duration = std::chrono::minutes(config_.split_duration_minutes);
      split_timer_ = this->create_wall_timer(
          split_duration,
          [this]()
          {
            if (is_recording_ && !should_stop_recording_)
            {
              split_bag_file();
            }
          });

      // Keep the recording thread alive
      while (!should_stop_recording_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Recording thread error: %s", e.what());
      is_recording_ = false;
    }
  }

  void RosbagRecorderNode::split_bag_file()
  {
#ifdef ROS_DISTRO_JAZZY
    // Stop current recording process if running
    if (recording_process_active_ && recording_process_pid_ > 0)
    {
      kill(recording_process_pid_, SIGTERM);
      int status;
      waitpid(recording_process_pid_, &status, 0);
      recording_process_active_ = false;
    }
#endif

    // Generate new bag filename
    current_bag_path_ = generate_bag_filename();

    // Discover and filter topics
    auto all_topics = discover_topics();
    auto regex_filtered_topics = filter_topics_by_regex(all_topics);

    // Combine explicitly listed topics and regex-filtered topics
    std::set<std::string> topics_to_record;
    for (const auto &topic : config_.topics)
    {
      topics_to_record.insert(topic);
    }
    for (const auto &topic : regex_filtered_topics)
    {
      topics_to_record.insert(topic);
    }

#ifdef ROS_DISTRO_JAZZY
    if (!topics_to_record.empty())
    {
      // Build ros2 bag record command
      std::string command = "ros2 bag record";
      command += " -o '" + current_bag_path_ + "'";
      command += " -s " + config_.storage_id;

      if (config_.max_bagfile_size > 0)
      {
        command += " -b " + std::to_string(config_.max_bagfile_size);
      }

      if (config_.include_hidden_topics)
      {
        command += " --include-hidden-topics";
      }

      // Add topics directly with proper quoting
      for (const auto &topic : topics_to_record)
      {
        command += " '" + topic + "'";
      }

      // Add services if enabled (note: services may not be supported in all ROS2 versions)
      // For now, let's comment this out to ensure the basic topic recording works
      /*
      if (config_.record_services && !config_.services.empty()) {
        command += " --services";
        for (const auto & service : config_.services) {
          command += " " + service;
        }
      }
      */

      command += " > /dev/null 2>&1 &"; // Run in background with output redirection

      RCLCPP_INFO(this->get_logger(), "Starting recording with command: %s", command.c_str());

      // Execute command
      int ret = system(command.c_str());
      if (ret == 0)
      {
        recording_process_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Started new bag file: %s", current_bag_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Recording %zu topics", topics_to_record.size());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to start recording process");
      }
    }

#elif defined(ROS_DISTRO_HUMBLE)
    // For Humble, use the simpler Writer-based approach
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = current_bag_path_;
    storage_options.storage_id = config_.storage_id;
    storage_options.max_bagfile_size = config_.max_bagfile_size;
    storage_options.max_cache_size = config_.max_cache_size;

    writer_->open(storage_options.uri);

    // For Humble, we need to manually create topics
    for (const auto &topic : topics_to_record)
    {
      // Get topic type information
      auto topic_names_and_types = this->get_topic_names_and_types();
      for (const auto &topic_info : topic_names_and_types)
      {
        if (topic_info.first == topic && !topic_info.second.empty())
        {
          rosbag2_storage::TopicMetadata topic_metadata;
          topic_metadata.name = topic;
          topic_metadata.type = topic_info.second[0]; // Use first type
          topic_metadata.serialization_format = config_.serialization_format;
          writer_->create_topic(topic_metadata);
          break;
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "Started new bag file: %s", current_bag_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Recording %zu topics (services not supported in Humble)",
                topics_to_record.size());
#endif

    bag_file_counter_++;
  }

  std::vector<std::string> RosbagRecorderNode::discover_topics()
  {
    std::vector<std::string> topics;
    auto topic_names_and_types = this->get_topic_names_and_types();

    for (const auto &topic_info : topic_names_and_types)
    {
      topics.push_back(topic_info.first);
    }

    return topics;
  }

  std::vector<std::string> RosbagRecorderNode::filter_topics_by_regex(
      const std::vector<std::string> &all_topics)
  {
    std::vector<std::string> filtered_topics;

    for (const auto &topic : all_topics)
    {
      if (matches_any_pattern(topic, config_.topic_regex_patterns))
      {
        filtered_topics.push_back(topic);
      }
    }

    return filtered_topics;
  }

  bool RosbagRecorderNode::matches_any_pattern(
      const std::string &topic,
      const std::vector<std::string> &patterns)
  {
    for (const auto &pattern : patterns)
    {
      try
      {
        std::regex regex_pattern(pattern);
        if (std::regex_match(topic, regex_pattern))
        {
          return true;
        }
      }
      catch (const std::regex_error &e)
      {
        RCLCPP_WARN(this->get_logger(), "Invalid regex pattern '%s': %s", pattern.c_str(), e.what());
      }
    }
    return false;
  }

  std::string RosbagRecorderNode::generate_bag_filename()
  {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << config_.bag_name_prefix << "_";
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

    if (bag_file_counter_ > 0)
    {
      ss << "_" << std::setfill('0') << std::setw(3) << bag_file_counter_;
    }

    std::filesystem::path bag_path = std::filesystem::path(config_.output_directory) / ss.str();

    return bag_path.string();
  }

  void RosbagRecorderNode::create_output_directory()
  {
    std::filesystem::path output_path(config_.output_directory);

    if (!std::filesystem::exists(output_path))
    {
      try
      {
        std::filesystem::create_directories(output_path);
        RCLCPP_INFO(this->get_logger(), "Created output directory: %s", config_.output_directory.c_str());
      }
      catch (const std::filesystem::filesystem_error &e)
      {
        throw std::runtime_error("Failed to create output directory: " + std::string(e.what()));
      }
    }

    if (!std::filesystem::is_directory(output_path))
    {
      throw std::runtime_error("Output path exists but is not a directory: " + config_.output_directory);
    }
  }

  void RosbagRecorderNode::publish_status()
  {
    // Publish recording status
    auto status_msg = std_msgs::msg::Bool();
    status_msg.data = is_recording_;
    recording_status_pub_->publish(status_msg);

    // Publish recording info
    auto info_msg = std_msgs::msg::String();
    if (is_recording_)
    {
      auto now = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - recording_start_time_);

      std::stringstream ss;
      ss << "Recording: " << current_bag_path_
         << " | Duration: " << duration.count() << "s"
         << " | Files: " << bag_file_counter_
         << " | Bytes: " << total_bytes_written_
         << " | Messages: " << total_messages_recorded_;
      info_msg.data = ss.str();
    }
    else
    {
      info_msg.data = "Not recording";
      if (!last_error_message_.empty())
      {
        info_msg.data += " | Last error: " + last_error_message_;
      }
    }
    recording_info_pub_->publish(info_msg);

    // Publish diagnostics
    publish_diagnostics();
  }

  void RosbagRecorderNode::check_storage_health()
  {
    storage_healthy_ = is_storage_available();

    if (!storage_healthy_ && is_recording_)
    {
      RCLCPP_ERROR(this->get_logger(), "Storage health check failed during recording! Stopping...");
      try
      {
        stop_recording();
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Error stopping recording: %s", e.what());
      }
    }
  }

  bool RosbagRecorderNode::is_storage_available()
  {
    try
    {
      std::filesystem::path output_path(config_.output_directory);

      // Check if directory exists
      if (!std::filesystem::exists(output_path))
      {
        last_error_message_ = "Output directory does not exist: " + config_.output_directory;
        return false;
      }

      // Check if it's a directory
      if (!std::filesystem::is_directory(output_path))
      {
        last_error_message_ = "Output path is not a directory: " + config_.output_directory;
        return false;
      }

      // Check write permissions by creating a test file
      std::string test_file = (output_path / ".rosbag_test_write").string();
      std::ofstream test_stream(test_file);
      if (!test_stream.is_open())
      {
        last_error_message_ = "Cannot write to output directory: " + config_.output_directory;
        return false;
      }
      test_stream.close();
      std::filesystem::remove(test_file);

      // Check available space
      auto space_info = std::filesystem::space(output_path);
      const uint64_t min_space_bytes = 1024 * 1024 * 100; // 100MB minimum
      if (space_info.available < min_space_bytes)
      {
        last_error_message_ = "Insufficient disk space in output directory";
        return false;
      }

      last_error_message_.clear();
      return true;
    }
    catch (const std::exception &e)
    {
      last_error_message_ = "Storage check error: " + std::string(e.what());
      return false;
    }
  }

  void RosbagRecorderNode::publish_diagnostics()
  {
    auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
    diag_array.header.stamp = this->get_clock()->now();

    // Main recorder status
    diagnostic_msgs::msg::DiagnosticStatus recorder_status;
    recorder_status.name = "rosbag_recorder";
    recorder_status.hardware_id = "rosbag_recorder_node";

    if (is_recording_)
    {
      if (storage_healthy_)
      {
        recorder_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        recorder_status.message = "Recording normally";
      }
      else
      {
        recorder_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        recorder_status.message = "Recording with storage issues: " + last_error_message_;
      }
    }
    else
    {
      if (storage_healthy_)
      {
        recorder_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        recorder_status.message = "Ready to record";
      }
      else
      {
        recorder_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        recorder_status.message = "Storage not available: " + last_error_message_;
      }
    }

    // Add key-value pairs
    diagnostic_msgs::msg::KeyValue kv;

    kv.key = "recording";
    kv.value = is_recording_ ? "true" : "false";
    recorder_status.values.push_back(kv);

    kv.key = "output_directory";
    kv.value = config_.output_directory;
    recorder_status.values.push_back(kv);

    if (is_recording_)
    {
      kv.key = "current_bag";
      kv.value = current_bag_path_;
      recorder_status.values.push_back(kv);

      kv.key = "file_count";
      kv.value = std::to_string(bag_file_counter_);
      recorder_status.values.push_back(kv);

      auto now = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - recording_start_time_);
      kv.key = "recording_duration_sec";
      kv.value = std::to_string(duration.count());
      recorder_status.values.push_back(kv);

      kv.key = "total_bytes";
      kv.value = std::to_string(total_bytes_written_);
      recorder_status.values.push_back(kv);

      kv.key = "total_messages";
      kv.value = std::to_string(total_messages_recorded_);
      recorder_status.values.push_back(kv);

#ifdef ROS_DISTRO_JAZZY
      kv.key = "services_count";
      kv.value = std::to_string(config_.services.size());
      recorder_status.values.push_back(kv);
#elif defined(ROS_DISTRO_HUMBLE)
      kv.key = "services_count";
      kv.value = "0";
      recorder_status.values.push_back(kv);
#endif
    }

    // Storage status
    diagnostic_msgs::msg::DiagnosticStatus storage_status;
    storage_status.name = "rosbag_recorder/storage";
    storage_status.hardware_id = config_.output_directory;

    if (storage_healthy_)
    {
      storage_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      storage_status.message = "Storage healthy";

      try
      {
        auto space_info = std::filesystem::space(config_.output_directory);
        kv.key = "available_bytes";
        kv.value = std::to_string(space_info.available);
        storage_status.values.push_back(kv);

        kv.key = "total_bytes";
        kv.value = std::to_string(space_info.capacity);
        storage_status.values.push_back(kv);

        double usage_percent = 100.0 * (space_info.capacity - space_info.available) / space_info.capacity;
        kv.key = "usage_percent";
        kv.value = std::to_string(usage_percent);
        storage_status.values.push_back(kv);
      }
      catch (const std::exception &e)
      {
        // Ignore space check errors for now
      }
    }
    else
    {
      storage_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      storage_status.message = last_error_message_;
    }

    diag_array.status.push_back(recorder_status);
    diag_array.status.push_back(storage_status);

    diagnostics_pub_->publish(diag_array);
  }

} // namespace rosbag_recorder

// Register the component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rosbag_recorder::RosbagRecorderNode)
