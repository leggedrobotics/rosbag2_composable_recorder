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

#ifndef ROSBAG2_COMPOSABLE_RECORDER__COMPOSABLE_RECORDER_HPP_
#define ROSBAG2_COMPOSABLE_RECORDER__COMPOSABLE_RECORDER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rosbag2_composable_recorder/srv/start_recording.hpp>

namespace rosbag2_composable_recorder
{
class ComposableRecorder : public rosbag2_transport::Recorder
{
public:
  explicit ComposableRecorder(const rclcpp::NodeOptions & options);
  ~ComposableRecorder();

private:
  std::string bag_name;
  std::string bag_path;
  // service callback function
  bool startRecording(
    const std::shared_ptr<rosbag2_composable_recorder::srv::StartRecording::Request> req,
    std::shared_ptr<rosbag2_composable_recorder::srv::StartRecording::Response> res);

  bool stopRecording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ---- variables
  rclcpp::Service<rosbag2_composable_recorder::srv::StartRecording>::SharedPtr service_start_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_stop_;
  bool isRecording_{false};
};

}  // namespace rosbag2_composable_recorder
#endif  // ROSBAG2_COMPOSABLE_RECORDER__COMPOSABLE_RECORDER_HPP_
