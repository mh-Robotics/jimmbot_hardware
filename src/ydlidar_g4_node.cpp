#include "jimmbot_sensors/ydlidar_g4_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ydlidar_node");
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);
  
    std::string port;
    int baudrate=230400;
    std::string frame_id;
    bool reversion, resolution_fixed;
    bool auto_reconnect;
    double angle_max,angle_min;
    result_t op_result;
    std::string list;
    std::vector<float> ignore_array;  
    double max_range, min_range;
    double frequency;
    int samp_rate = 5;
    bool inverted = true;
    bool isSingleChannel = false;
    bool isTOFLidar = false;

    auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1000);

    node->declare_parameter<std::string>("port", "/dev/ydlidar");
    node->declare_parameter<int>("baudrate", 230400);
    node->declare_parameter<std::string>("frame_id", "laser_frame");
    node->declare_parameter<bool>("resolution_fixed", true);
    node->declare_parameter<bool>("auto_reconnect", true);
    node->declare_parameter<bool>("reversion", true);
    node->declare_parameter<double>("angle_max", 180.0);
    node->declare_parameter<double>("angle_min", -180.0);
    node->declare_parameter<double>("range_max", 64.0);
    node->declare_parameter<double>("range_min", 0.01);
    node->declare_parameter<double>("frequency", 10.0);
    node->declare_parameter<std::string>("ignore_array", "");
    node->declare_parameter<int>("samp_rate", samp_rate);
    node->declare_parameter<bool>("isSingleChannel", isSingleChannel);
    node->declare_parameter<bool>("isTOFLidar", isTOFLidar);

    node->get_parameter("port", port);
    node->get_parameter("baudrate", baudrate);
    node->get_parameter("frame_id", frame_id);
    node->get_parameter("resolution_fixed", resolution_fixed);
    node->get_parameter("auto_reconnect", auto_reconnect);
    node->get_parameter("reversion", reversion);
    node->get_parameter("angle_max", angle_max);
    node->get_parameter("angle_min", angle_min);
    node->get_parameter("range_max", max_range);
    node->get_parameter("range_min", min_range);
    node->get_parameter("frequency", frequency);
    node->get_parameter("ignore_array", list);
    node->get_parameter("samp_rate", samp_rate);
    node->get_parameter("isSingleChannel", isSingleChannel);
    node->get_parameter("isTOFLidar", isTOFLidar);
 

    ignore_array = split(list ,',');
    if(ignore_array.size()%2){
        RCLCPP_ERROR(node->get_logger(), "ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            RCLCPP_ERROR(node->get_logger(), "ignore array should be between 0 and 360");
        }
    }

    CYdLidar laser;
    if(frequency<3){
       frequency = 7.0; 
    }
    if(frequency>15.7){
        frequency = 15.7;
    }
    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR ROS SDK VERSION:%s .......", ROSVerision);
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setMaxRange(max_range);
    laser.setMinRange(min_range);
    laser.setMaxAngle(angle_max);
    laser.setMinAngle(angle_min);
    laser.setReversion(reversion);
    laser.setFixedResolution(resolution_fixed);
    laser.setAutoReconnect(auto_reconnect);
    laser.setScanFrequency(frequency);
    laser.setIgnoreArray(ignore_array);
    laser.setSampleRate(samp_rate);
    laser.setInverted(inverted);
    laser.setSingleChannel(isSingleChannel);
    laser.setLidarType(isTOFLidar ? TYPE_TOF : TYPE_TRIANGLE);
    bool ret = laser.initialize();
    if (ret) {
        ret = laser.turnOn();
        if (!ret) {
            RCLCPP_ERROR(node->get_logger(), "Failed to start scan mode!!!");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Error initializing YDLIDAR Comms and Status!!!");
    }
    rclcpp::Rate rate(20);

    while (ret && rclcpp::ok()) {
        bool hardError;
        LaserScan scan;
        if(laser.doProcessSimple(scan, hardError )){
            sensor_msgs::msg::LaserScan scan_msg;
            rclcpp::Time start_scan_time(
                static_cast<int32_t>(scan.stamp / 1000000000ul),
                static_cast<uint32_t>(scan.stamp % 1000000000ul),
                RCL_ROS_TIME);
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frame_id;
            scan_msg.angle_min =(scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            scan_msg.angle_increment = (scan.config.angle_increment);
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);
            int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
            scan_msg.ranges.resize(size);
            scan_msg.intensities.resize(size);
            for(int i=0; i < scan.points.size(); i++) {
                int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
                if(index >=0 && index < size) {
                     scan_msg.ranges[index] = scan.points[i].range;
                     scan_msg.intensities[index] = scan.points[i].intensity;
                }
            }
            scan_pub->publish(scan_msg);
        }  
        rate.sleep();
        rclcpp::spin_some(node);
    }

    laser.turnOff();
    RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
    laser.disconnecting();
    rclcpp::shutdown();
    return 0;
}
