// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosbag2_composable_recorder/composable_recorder.hpp"

#include <stdio.h>

#include <chrono>
#include <iomanip>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>
#include <rosbag2_composable_recorder/srv/start_recording.hpp>
#include <filesystem>
#include <system_error>

bool createFolder(const std::string& path) {
    std::filesystem::path dirPath(path);
    
    if (std::filesystem::exists(dirPath)) {
        return true;  // Folder already exists
    }
    
    std::error_code ec;
    if (std::filesystem::create_directories(dirPath, ec)) {
        return true;  // Folder created successfully
    } else {
        std::cerr << "Error creating directory: " << ec.message() << std::endl;
        return false;  // Failed to create folder
    }
}

namespace rosbag2_composable_recorder
{
static std::string get_time_stamp()
{
  std::stringstream datetime;
  auto now = std::chrono::system_clock::now();
  auto t_now = std::chrono::system_clock::to_time_t(now);
  datetime << std::put_time(std::localtime(&t_now), "%Y-%m-%d-%H-%M-%S");
  return (datetime.str());
}

ComposableRecorder::ComposableRecorder(const rclcpp::NodeOptions & options)
: rosbag2_transport::Recorder(
    std::make_shared<rosbag2_cpp::Writer>(), rosbag2_storage::StorageOptions(),
    rosbag2_transport::RecordOptions(), "recorder",
    rclcpp::NodeOptions(options).start_parameter_event_publisher(false))
{
  // set storage options were originally here

  // set storage options were orginially here

  // set recorder options
  #ifdef USE_GET_RECORD_OPTIONS
    rosbag2_transport::RecordOptions & ropt = get_record_options();
  #else
  rosbag2_transport::RecordOptions & ropt = record_options_;
  #endif

  #ifdef USE_ALL_TOPICS
    ropt.all_topics = false;
  #else
    ropt.all = false;
  #endif
    ropt.is_discovery_disabled = declare_parameter<bool>("disable_discovery", false);
    ropt.rmw_serialization_format = declare_parameter<std::string>("serialization_format", "cdr");
    ropt.topic_polling_interval = std::chrono::milliseconds(100);
    

  storage_id = declare_parameter<std::string>("storage_id", "sqlite3");
  max_cache_size = declare_parameter<int>("max_cache_size", 100 * 1024 * 1024);


  bag_path = declare_parameter<std::string>("bag_path", "/data/");
  bag_name = declare_parameter<std::string>("bag_name", "_test_bag");
  std::string a = get_time_stamp();


  if (declare_parameter<bool>("start_recording_immediately", false)) {
    record();
  } else {
    std::string service_name_start = std::string(get_name()) + "/start_recording";
    service_start_ = create_service<rosbag2_composable_recorder::srv::StartRecording>(
      service_name_start,
      std::bind(
        &ComposableRecorder::startRecording, this, std::placeholders::_1, std::placeholders::_2));
  }
  std::string service_name_stop = std::string(get_name()) + "/stop_recording";
  service_stop_ = create_service<std_srvs::srv::Trigger>(
      service_name_stop,
      std::bind(
        &ComposableRecorder::stopRecording, this, std::placeholders::_1, std::placeholders::_2));

}

bool ComposableRecorder::stopRecording(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  
  if (isRecording_) {
    RCLCPP_WARN(get_logger(), "stop recording!");
    stop();
    res->message = "stoped recording!";
    res->success = true;
    // RCLCPP_WARN(get_logger(), "manually starting recording!");
    // storage_options_.uri = bag_path + "restart/manual_" + bag_name;
    // record();
    // RCLCPP_WARN(get_logger(), "stopping recording again!");
    // stop();
    // RCLCPP_WARN(get_logger(), "stopped recording again!");
  } else {
    RCLCPP_INFO(get_logger(), "cannot stop recording - we are not recording...");
    res->success = false;
    res->message = "cannot stop recording - we are not recording!";
  }
  isRecording_ = false;
  return (true);
}

bool ComposableRecorder::startRecording(
  const std::shared_ptr<rosbag2_composable_recorder::srv::StartRecording::Request> req,
  std::shared_ptr<rosbag2_composable_recorder::srv::StartRecording::Response> res)
{
  (void)req;
    std::vector<std::string> topics = req->topics;
    for (const auto & topic : topics) {
      RCLCPP_INFO_STREAM(get_logger(), "recording topic: " << topic);
    }

    // set recorder options
    // #ifdef USE_GET_RECORD_OPTIONS
      // rosbag2_transport::RecordOptions & ropt = get_record_options();
    // #else
      rosbag2_transport::RecordOptions & ropt = record_options_;
    // #endif

    // Remove all previous topics before adding new ones.
    ropt.topics.clear();
    ropt.topics.insert(ropt.topics.end(), topics.begin(), topics.end());

    if (ropt.is_discovery_disabled) {
      #ifdef USE_STOP_DISCOVERY
          stop_discovery();
      #else
          stop_discovery_ = ropt.is_discovery_disabled;
      #endif
    }

  res->success = false;
  if (isRecording_) {
    RCLCPP_WARN(get_logger(), "already recording!");
    res->message = "already recording!";
  } else {
    RCLCPP_INFO(get_logger(), "starting recording...");
    try {
      // update the storage options
      #ifdef USE_GET_STORAGE_OPTIONS
        rosbag2_storage::StorageOptions & sopt = get_storage_options();
      #else
        rosbag2_storage::StorageOptions & sopt = storage_options_;
      #endif
      
      sopt.storage_id = storage_id;
      sopt.max_cache_size = max_cache_size;
      sopt.uri = bag_path + req->timestamp + "/" + req->timestamp + bag_name;
      
      createFolder(bag_path + req->timestamp);

      record();
      isRecording_ = true;
      RCLCPP_INFO(get_logger(), "started recording successfully");
      res->success = true;
      res->message = "started recoding!";
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "Error: cannot toggle recording! Exception: %s", e.what());
      res->message = "runtime error occurred: " + std::string(e.what());
    }
  }
  return (true);
}

ComposableRecorder::~ComposableRecorder() {}
}  // namespace rosbag2_composable_recorder

RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_composable_recorder::ComposableRecorder)
