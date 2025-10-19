# RosbagRecorderStatus.msg
# Status message for Foxglove visualization

# Basic recording state
bool is_recording
string current_bag_path
string output_directory

# Recording session info
int32 file_count
int64 recording_duration_sec
uint64 total_bytes_written
uint64 total_messages_recorded

# Health information
bool storage_healthy
string last_error_message
uint64 available_disk_space_bytes
float64 disk_usage_percent

# Configuration
string bag_name_prefix
int32 split_duration_minutes
string[] topics
string[] topic_regex_patterns
string[] services

# Timestamps
builtin_interfaces/Time recording_started_at
builtin_interfaces/Time last_split_at
