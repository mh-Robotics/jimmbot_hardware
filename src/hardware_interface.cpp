/**
 * @file jimmbot_hardware_interface.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file contains the implementation of the JimmBotHardwareInterface
 * class. The JimmBotHardwareInterface is a class that implements the ROS
 * RobotHW interface. It provides an interface to interact with the hardware of
 * the JimmBot robot.
 * @version 0.1
 * @date 2021-03-23
 *
 * @copyright Copyright (c) 2020-2023, mhRobotics, Inc. All rights reserved.
 * @license This project is released under the BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "jimmbot_base/hardware_interface.hpp"  // for JimmBotHardwareInterface

#include <chrono>  // for std::chrono::milliseconds
#include <thread>  // for std::thread

namespace {
constexpr auto kOneSecond = 1;
}  // namespace

namespace jimmbot_base {
JimmBotHardwareInterface::JimmBotHardwareInterface(rclcpp::Node::SharedPtr node)
    : lights_(false, false),
      camera_angles_({}, {}),
      control_frequency_(kDefaultControlFrequency),
      max_wheel_speed_(kDefaultMaxAllowedWheelSpeed),
      node_(node) {
  node_->declare_parameter<double>(kControlFrequencyKey, kDefaultControlFrequency);
  node_->get_parameter(kControlFrequencyKey, control_frequency_);

  node_->declare_parameter<double>(kMaxWheelSpeedKey, kDefaultMaxAllowedWheelSpeed);
  node_->get_parameter(kMaxWheelSpeedKey, max_wheel_speed_);

  node_->declare_parameter<std::string>(kCommandFrameIdKey, std::string(kDefaultCommandFrameId));
  node_->get_parameter(kCommandFrameIdKey, frame_id_);

  node_->declare_parameter<std::string>(kLeftWheelFrontKey, std::string(kDefaultLeftWheelFront));
  node_->get_parameter(kLeftWheelFrontKey, left_wheel_front_);

  node_->declare_parameter<std::string>(kLeftWheelBackKey, std::string(kDefaultLeftWheelBack));
  node_->get_parameter(kLeftWheelBackKey, left_wheel_back_);

  node_->declare_parameter<std::string>(kRightWheelFrontKey, std::string(kDefaultRightWheelFront));
  node_->get_parameter(kRightWheelFrontKey, right_wheel_front_);

  node_->declare_parameter<std::string>(kRightWheelBackKey, std::string(kDefaultRightWheelBack));
  node_->get_parameter(kRightWheelBackKey, right_wheel_back_);

  esp32_can_sub_ = node_->create_subscription<jimmbot_msgs::msg::CanFrameStamped>(
      kDefaultFeedbackTopic, 1,
      [this](const jimmbot_msgs::msg::CanFrameStamped::ConstSharedPtr msg) {
        CanFeedbackMsgCallback(msg);
      });
  extn_data_sub_ = node_->create_subscription<jimmbot_msgs::msg::ExtnDataStamped>(
      kDefaultExtendedDataTopic, 1,
      [this](const jimmbot_msgs::msg::ExtnDataStamped::ConstSharedPtr msg) {
        ExtnDataMsgCallback(msg);
      });
  camera_tilt_front_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      kDefaultFrontCameraTiltTopic, 1,
      [this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
        CameraTiltFrontCallback(msg);
      });
  camera_tilt_back_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      kDefaultBackCameraTiltTopic, 1,
      [this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
        CameraTiltBackCallback(msg);
      });
  esp32_can_pub_ = node_->create_publisher<jimmbot_msgs::msg::CanFrameStamped>(
      kDefaultCommandTopic, 1);
}

void JimmBotHardwareInterface::write(const rclcpp::Time& /*time*/,
                                     const rclcpp::Duration& /*period*/) {
  std::vector<CanMsgWrapperCommand> speed_commands{
      CanMsgWrapperCommand{std::ref(front_left_),
                           CanMsgWrapperCommand::Command::kWheelSpeed,
                           joint_elements_[front_left_.TransmitId()].command},
      CanMsgWrapperCommand{std::ref(front_right_),
                           CanMsgWrapperCommand::Command::kWheelSpeed,
                           joint_elements_[front_right_.TransmitId()].command},
      CanMsgWrapperCommand{std::ref(back_left_),
                           CanMsgWrapperCommand::Command::kWheelSpeed,
                           joint_elements_[back_left_.TransmitId()].command},
      CanMsgWrapperCommand{std::ref(back_right_),
                           CanMsgWrapperCommand::Command::kWheelSpeed,
                           joint_elements_[back_right_.TransmitId()].command}};

  for (auto& speed_command : speed_commands) {
    speed_command.Execute();
  }

  RCLCPP_WARN(node_->get_logger(), "C: Velocity: %.2f",
              joint_elements_[back_right_.TransmitId()].command.velocity);

  //@todo(issues/6): Write the angle to AUX kinect
  UpdateSpeedToHardware();
}

void JimmBotHardwareInterface::read(const rclcpp::Time& /*time*/,
                                    const rclcpp::Duration& /*period*/) {
  std::vector<CanMsgWrapperCommand> feedbacks{
      CanMsgWrapperCommand{std::ref(front_left_),
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{std::ref(front_right_),
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{std::ref(back_left_),
                           CanMsgWrapperCommand::Command::kWheelStatus},
      CanMsgWrapperCommand{std::ref(back_right_),
                           CanMsgWrapperCommand::Command::kWheelStatus}};

  for (auto& feedback : feedbacks) {
    feedback.Execute();
  }

  UpdateJointsFromHardware();
}

void JimmBotHardwareInterface::UpdateJointsFromHardware() const {
  {
    joint_elements_[front_left_.TransmitId()].feedback =
        front_left_.GetWheelFeedbackStatus();
    joint_elements_[front_right_.TransmitId()].feedback =
        front_right_.GetWheelFeedbackStatus();
    joint_elements_[back_left_.TransmitId()].feedback =
        back_left_.GetWheelFeedbackStatus();
    joint_elements_[back_right_.TransmitId()].feedback =
        back_right_.GetWheelFeedbackStatus();

    RCLCPP_WARN(node_->get_logger(), "F: Command: %d",
          joint_elements_[back_right_.TransmitId()].feedback.command_id);
    RCLCPP_WARN(node_->get_logger(), "F: Effort: %.2f",
          joint_elements_[back_right_.TransmitId()].feedback.effort);
    RCLCPP_WARN(node_->get_logger(), "F: Position: %.2f",
          joint_elements_[back_right_.TransmitId()].feedback.position);
    RCLCPP_WARN(node_->get_logger(), "F: RPM: %d",
          joint_elements_[back_right_.TransmitId()].feedback.rpm);
    RCLCPP_WARN(node_->get_logger(), "F: Velocity: %.2f",
          joint_elements_[back_right_.TransmitId()].feedback.velocity);
  }
}

void JimmBotHardwareInterface::UpdateSpeedToHardware() const {
  jimmbot_msgs::msg::CanFrameStamped data_frame;

  {
    data_frame.header.stamp = node_->now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = front_left_.GetWheelCommandStatus();
    esp32_can_pub_->publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    data_frame.header.stamp = node_->now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = front_right_.GetWheelCommandStatus();
    esp32_can_pub_->publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    data_frame.header.stamp = node_->now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = back_left_.GetWheelCommandStatus();
    esp32_can_pub_->publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    data_frame.header.stamp = node_->now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = back_right_.GetWheelCommandStatus();
    esp32_can_pub_->publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  {
    data_frame.header.stamp = node_->now();
    data_frame.header.frame_id = frame_id_;
    data_frame.can_frame = CanMsgWrapper::GetLightsInCan(lights_);
    esp32_can_pub_->publish(data_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void JimmBotHardwareInterface::UpdateAngleToKinectCameras() {
  std_msgs::msg::Float64 angle;

  angle.data =
      (std::get<kFirst>(camera_angles_) == std::get<kSecond>(camera_angles_)
           ? std::get<kFirst>(camera_angles_)
           : std::get<kSecond>(camera_angles_));

  // @todo(issues/6): Publish angle to Kinect AUX node topic
}

void JimmBotHardwareInterface::CanFeedbackMsgCallback(
  const jimmbot_msgs::msg::CanFrameStamped::ConstSharedPtr& feedback_msg) {
  std::vector<CanMsgWrapperCommand> update_status_frames_commands{
      CanMsgWrapperCommand{std::ref(front_left_),
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{std::ref(front_right_),
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{std::ref(back_left_),
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame},
      CanMsgWrapperCommand{std::ref(back_right_),
                           CanMsgWrapperCommand::Command::kWheelStatusUpdate,
                           feedback_msg->can_frame}};

  for (auto& status_frame_command : update_status_frames_commands) {
    status_frame_command.Execute();
  }
}

void JimmBotHardwareInterface::ExtnDataMsgCallback(
  const jimmbot_msgs::msg::ExtnDataStamped::ConstSharedPtr& extn_data_msg) {
  lights_ = {extn_data_msg->extn_data.left_light_bulb,
             extn_data_msg->extn_data.right_light_bulb};
}

void JimmBotHardwareInterface::CameraTiltFrontCallback(
  const std_msgs::msg::Float64::ConstSharedPtr& angle) {
  camera_angles_.first = angle->data;
}

void JimmBotHardwareInterface::CameraTiltBackCallback(
  const std_msgs::msg::Float64::ConstSharedPtr& angle) {
  camera_angles_.second = angle->data;
}

void ControlLoopCallback(
    jimmbot_base::JimmBotHardwareInterface& hw,
    rclcpp::Time& last_time) {
  const auto now = hw.GetTimeNow();
  const auto period = hw.GetElapsedTime(last_time);
  last_time = now;
  hw.read(now, period);
  hw.write(now, period);
}

}  // end namespace jimmbot_base

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hardware_interface_node");
  jimmbot_base::JimmBotHardwareInterface hardware_interface(node);

  auto last_time = hardware_interface.GetTimeNow();
  const double control_period_s =
      kOneSecond / hardware_interface.GetControlFrequency();
  auto control_timer = node->create_wall_timer(
      std::chrono::duration<double>(control_period_s),
      [&hardware_interface, &last_time]() {
        jimmbot_base::ControlLoopCallback(hardware_interface, last_time);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
