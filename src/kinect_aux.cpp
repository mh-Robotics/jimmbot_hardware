#include <libusb-1.0/libusb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

// VID and PID for Kinect and motor/acc/leds
#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
// Constants for accelerometers
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.
// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree
// increments The control input looks like 2*desired_degrees
#define MAX_TILT_ANGLE 1.
#define MIN_TILT_ANGLE (-60.)

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_tilt_angle;
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_tilt_status;

rclcpp::Node::SharedPtr g_node;

libusb_device_handle* dev(0);

void openAuxDevice(int index = 0) {
  libusb_device**
      devs;  // pointer to pointer of device, used to retrieve a list of devices
  ssize_t cnt = libusb_get_device_list(0, &devs);  // get the list of devices
  if (cnt < 0) {
    RCLCPP_ERROR(g_node->get_logger(), "No device on USB");
    return;
  }

  int nr_mot(0);
  for (int i = 0; i < cnt; ++i) {
    struct libusb_device_descriptor desc;
    const int r = libusb_get_device_descriptor(devs[i], &desc);
    if (r < 0) continue;

    // Search for the aux
    if (desc.idVendor == MS_MAGIC_VENDOR &&
        desc.idProduct == MS_MAGIC_MOTOR_PRODUCT) {
      // If the index given by the user matches our camera index
      if (nr_mot == index) {
        if ((libusb_open(devs[i], &dev) != 0) || (dev == 0)) {
          RCLCPP_ERROR(g_node->get_logger(), "Cannot open aux %d", index);
          return;
        }
        // Claim the aux
        libusb_claim_interface(dev, 0);
        break;
      } else
        nr_mot++;
    }
  }

  libusb_free_device_list(devs, 1);  // free the list, unref the devices in it
}

void publishState(void) {
  uint8_t buf[10];
  const int ret =
      libusb_control_transfer(dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0);
  if (ret != 10) {
    RCLCPP_ERROR(g_node->get_logger(),
                 "Error in accelerometer reading, libusb_control_transfer returned %d", ret);
    rclcpp::shutdown();
  }

  const uint16_t ux = ((uint16_t)buf[2] << 8) | buf[3];
  const uint16_t uy = ((uint16_t)buf[4] << 8) | buf[5];
  const uint16_t uz = ((uint16_t)buf[6] << 8) | buf[7];

  const int16_t accelerometer_x = (int16_t)ux;
  const int16_t accelerometer_y = (int16_t)uy;
  const int16_t accelerometer_z = (int16_t)uz;
  const int8_t tilt_angle = (int8_t)buf[8];
  const uint8_t tilt_status = buf[9];

  // publish IMU
  sensor_msgs::msg::Imu imu_msg;
  if (pub_imu->get_subscription_count() > 0) {
    imu_msg.header.stamp = g_node->now();
    imu_msg.linear_acceleration.x =
        (double(accelerometer_x) / FREENECT_COUNTS_PER_G) * GRAVITY;
    imu_msg.linear_acceleration.y =
        (double(accelerometer_y) / FREENECT_COUNTS_PER_G) * GRAVITY;
    imu_msg.linear_acceleration.z =
        (double(accelerometer_z) / FREENECT_COUNTS_PER_G) * GRAVITY;
    imu_msg.linear_acceleration_covariance[0] =
        imu_msg.linear_acceleration_covariance[4] =
            imu_msg.linear_acceleration_covariance[8] =
                0.01;  // @todo - what should these be?
    imu_msg.angular_velocity_covariance[0] =
        -1;  // indicates angular velocity not provided
    imu_msg.orientation_covariance[0] =
        -1;  // indicates orientation not provided
    pub_imu->publish(imu_msg);
  }

  // publish tilt angle and status
  if (pub_tilt_angle->get_subscription_count() > 0) {
    std_msgs::msg::Float64 tilt_angle_msg;
    tilt_angle_msg.data = double(tilt_angle) / 2.;
    pub_tilt_angle->publish(tilt_angle_msg);
  }
  if (pub_tilt_status->get_subscription_count() > 0) {
    std_msgs::msg::UInt8 tilt_status_msg;
    tilt_status_msg.data = tilt_status;
    pub_tilt_status->publish(tilt_status_msg);
  }
}

void setTiltAngle(const std_msgs::msg::Float64::ConstSharedPtr angleMsg) {
  uint8_t empty[0x1];
  double angle(angleMsg->data);

  angle = (angle < MIN_TILT_ANGLE)
              ? MIN_TILT_ANGLE
              : ((angle > MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
  angle = angle * 2;
  const int ret = libusb_control_transfer(dev, 0x40, 0x31, (uint16_t)angle, 0x0,
                                          empty, 0x0, 0);
  if (ret != 0) {
    RCLCPP_ERROR(g_node->get_logger(),
                 "Error in setting tilt angle, libusb_control_transfer returned %d", ret);
    rclcpp::shutdown();
  }
}

void setLedOption(const std_msgs::msg::UInt16::ConstSharedPtr optionMsg) {
  uint8_t empty[0x1];
  const uint16_t option(optionMsg->data);

  const int ret = libusb_control_transfer(dev, 0x40, 0x06, (uint16_t)option,
                                          0x0, empty, 0x0, 0);
  if (ret != 0) {
    RCLCPP_ERROR(g_node->get_logger(),
                 "Error in setting LED options, libusb_control_transfer returned %d", ret);
    rclcpp::shutdown();
  }
}

int main(int argc, char* argv[]) {
  int ret = libusb_init(0);
  if (ret) {
    fprintf(stderr, "Cannot initialize libusb, error: %d\n", ret);
    return 1;
  }

  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("kinect_aux");

  int deviceIndex;
  g_node->declare_parameter<int>("device_index", 0);
  g_node->get_parameter("device_index", deviceIndex);
  openAuxDevice(deviceIndex);
  if (!dev) {
    RCLCPP_ERROR(g_node->get_logger(), "No valid aux device found");
    libusb_exit(0);
    return 2;
  }

  pub_imu = g_node->create_publisher<sensor_msgs::msg::Imu>("imu", 15);
  pub_tilt_angle = g_node->create_publisher<std_msgs::msg::Float64>("cur_tilt_angle", 15);
  pub_tilt_status = g_node->create_publisher<std_msgs::msg::UInt8>("cur_tilt_status", 15);

  auto sub_tilt_angle = g_node->create_subscription<std_msgs::msg::Float64>(
      "tilt_angle", 1, setTiltAngle);
  auto sub_led_option = g_node->create_subscription<std_msgs::msg::UInt16>(
      "led_option", 1, setLedOption);

  while (rclcpp::ok()) {
    rclcpp::spin_some(g_node);
    publishState();
  }

  libusb_exit(0);
  rclcpp::shutdown();
  return 0;
}
