#include "jimmbot_base/extended_joy.hpp"

namespace jimmbot_base {
/**
 * Internal members of class. This is the pimpl idiom, and allows more
 * flexibility in adding parameters later without breaking ABI compatibility,
 * for robots which link TeleopTwistJoyExtended directly into base nodes.
 */
struct TeleopTwistJoyExtended::Impl {
  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& joy);
  void SendTiltAngleMsg(const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg,
                        const std::string& which_map);
  void SendExtnDataMsg(const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg,
                       const std::string& which_map);
  bool ReturnSwitchStateFromPush(bool state);

  rclcpp::Node* node{nullptr};
  rclcpp::Time time_old{0, 0, RCL_ROS_TIME};
  bool inverse_state = false;
  int inverse_movement = 3;
  int left_light_bulb = 5;
  int right_light_bulb = 4;
  int horn = 0;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_possition_pub_front;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_possition_pub_back;
  rclcpp::Publisher<jimmbot_msgs::msg::ExtnDataStamped>::SharedPtr extn_data_pub;

  std::map<std::string, int> axis_linear_map;
  std::map<std::string, std::map<std::string, double> > scale_linear_map;
};

/**
 * Constructs TeleopTwistJoyExtended.
 * \param node rclcpp::Node to use for pub/sub and parameters.
 */
TeleopTwistJoyExtended::TeleopTwistJoyExtended(rclcpp::Node* node) {
  pimpl_ = new Impl;
  pimpl_->node = node;
  pimpl_->time_old = node->now();

  pimpl_->camera_possition_pub_front =
      node->create_publisher<std_msgs::msg::Float64>("camera_tilt_front", 1);
  pimpl_->camera_possition_pub_back =
      node->create_publisher<std_msgs::msg::Float64>("camera_tilt_back", 1);
  pimpl_->extn_data_pub =
      node->create_publisher<jimmbot_msgs::msg::ExtnDataStamped>("extn_data", 1);
  pimpl_->joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        pimpl_->JoyCallback(msg);
      });

  node->declare_parameter<int>("axis_linear_tilt", 3);
  node->get_parameter("axis_linear_tilt", pimpl_->axis_linear_map["data"]);

  node->declare_parameter<double>("max_angle_tilt", 30.0);
  node->get_parameter("max_angle_tilt",
                      pimpl_->scale_linear_map["normal"]["data"]);

  for (auto it = pimpl_->axis_linear_map.begin();
       it != pimpl_->axis_linear_map.end(); ++it) {
    RCLCPP_INFO(node->get_logger(),
                "Linear axis %s on %i at scale %f.", it->first.c_str(),
                it->second, pimpl_->scale_linear_map["normal"][it->first]);
  }

  node->declare_parameter<int>("inverse_movement", 3);
  node->get_parameter("inverse_movement", pimpl_->inverse_movement);

  node->declare_parameter<int>("left_light_bulb", 5);
  node->get_parameter("left_light_bulb", pimpl_->left_light_bulb);

  node->declare_parameter<int>("right_light_bulb", 4);
  node->get_parameter("right_light_bulb", pimpl_->right_light_bulb);

  node->declare_parameter<int>("horn", 0);
  node->get_parameter("horn", pimpl_->horn);
}

double DegreeToRadian(double degree) { return (degree * M_PI / 180); }

double GetVal(const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg,
              const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map,
              const std::string& fieldname) {
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname)) {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoyExtended::Impl::SendTiltAngleMsg(
    const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg,
    const std::string& which_map) {
  // Initializes with zeros by default.
  std_msgs::msg::Float64 camera_front_possition_msg;
  std_msgs::msg::Float64 camera_back_possition_msg;

  camera_front_possition_msg.data = DegreeToRadian(
      GetVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "data") *
      -1);
  camera_back_possition_msg.data = DegreeToRadian(
      GetVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "data"));

  camera_possition_pub_front->publish(camera_front_possition_msg);
  camera_possition_pub_back->publish(camera_back_possition_msg);
}

bool TeleopTwistJoyExtended::Impl::ReturnSwitchStateFromPush(bool state) {
  const double elapsed =
      (node->now() - time_old).seconds();
  if (elapsed < 0.2) {
    return inverse_state;
  }

  if (state == true) {
    if (inverse_state == false) {
      inverse_state = true;
    } else if (inverse_state == true) {
      inverse_state = false;
    }
  }

  time_old = node->now();

  return inverse_state;
}

void TeleopTwistJoyExtended::Impl::SendExtnDataMsg(
    const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg,
    const std::string& which_map) {
  jimmbot_msgs::msg::ExtnDataStamped extn_data_msg;

  extn_data_msg.header.frame_id = "/jimmbot/extn_data";
  extn_data_msg.header.stamp = node->now();
  extn_data_msg.extn_data.inverse_movement = static_cast<uint8_t>(
      ReturnSwitchStateFromPush(joy_msg->buttons[inverse_movement] != 0));
  extn_data_msg.extn_data.left_light_bulb =
      static_cast<uint8_t>(joy_msg->axes[left_light_bulb] == -1 ? true : false);
  extn_data_msg.extn_data.right_light_bulb = static_cast<uint8_t>(
      joy_msg->axes[right_light_bulb] == -1 ? true : false);
  extn_data_msg.extn_data.horn = joy_msg->buttons[horn];

  extn_data_pub->publish(extn_data_msg);
}

void TeleopTwistJoyExtended::Impl::JoyCallback(
    const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg) {
  if (!joy_msg->buttons.empty()) {
    SendTiltAngleMsg(joy_msg, "normal");
    SendExtnDataMsg(joy_msg, "normal");
  }
}
}  // end namespace jimmbot_base

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("extended_joy_node");
  jimmbot_base::TeleopTwistJoyExtended joy_teleop_extended(node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
