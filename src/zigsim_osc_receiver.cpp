#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <thread>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <cctype>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Helper functions to safely get JSON values
template<typename T>
T get_json_value(const json& j, const std::string& key, T default_val) {
    try {
        if (j.contains(key)) {
            return j[key].get<T>();
        }
    } catch (...) {}
    return default_val;
}

// OSC parsing helpers
struct OSCMessage {
    std::string address;
    std::vector<float> args;
};

bool parse_osc_string(const char* data, size_t size, size_t& pos, std::string& result) {
    if (pos >= size) return false;

    size_t start = pos;
    while (pos < size && data[pos] != '\0') pos++;

    if (pos >= size) return false;

    result = std::string(data + start, pos - start);
    pos++;

    // Align to 4-byte boundary
    pos = ((pos + 3) / 4) * 4;
    return true;
}

bool parse_osc_message_data(const char* data, size_t size, OSCMessage& msg) {
    size_t pos = 0;

    // Parse address
    if (!parse_osc_string(data, size, pos, msg.address)) {
        return false;
    }

    if (pos >= size || data[pos] != ',') {
        return false;
    }
    pos++; // Skip comma

    // Parse type tags
    std::string type_tags;
    if (!parse_osc_string(data, size, pos, type_tags)) {
        return false;
    }
    type_tags = type_tags.substr(1); // Remove leading comma

    // Parse arguments
    for (char tag : type_tags) {
        if (pos + 4 > size) break;

        if (tag == 'f') {  // float
            uint32_t raw;
            std::memcpy(&raw, &data[pos], 4);
            raw = ntohl(raw);
            float value;
            std::memcpy(&value, &raw, 4);
            msg.args.push_back(value);
            pos += 4;
        } else if (tag == 'i') {  // int32
            uint32_t raw;
            std::memcpy(&raw, &data[pos], 4);
            int32_t value = ntohl(raw);
            msg.args.push_back(static_cast<float>(value));
            pos += 4;
        } else {
            // Skip unknown types
            pos += 4;
        }
    }

    return true;
}

class ZigSimOSCReceiver : public rclcpp::Node {
public:
    ZigSimOSCReceiver() : Node("osc_node"), running_(false) {
        // Declare parameters
        this->declare_parameter<std::string>("host", "0.0.0.0");
        this->declare_parameter<int>("port", 8000);
        this->declare_parameter<std::string>("frame_id", "iphone");

        host_ = this->get_parameter("host").as_string();
        port_ = this->get_parameter("port").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "zigsim/imu", 10);
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
            "zigsim/magnetic_field", 10);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "zigsim/gps", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "zigsim/pose", 10);
        arkit_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "zigsim/arkit_pose", 10);
        gravity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "zigsim/gravity", 10);
        pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(
            "zigsim/pressure", 10);
        compass_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "zigsim/compass_heading", 10);

        // Start UDP receiver thread
        if (init_socket()) {
            running_ = true;
            receiver_thread_ = std::thread(&ZigSimOSCReceiver::receive_loop, this);
            RCLCPP_INFO(this->get_logger(),
                "ZigSim OSC Receiver started on %s:%d", host_.c_str(), port_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize socket");
        }
    }

    ~ZigSimOSCReceiver() {
        running_ = false;
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    bool init_socket() {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return false;
        }

        // Set socket options
        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to set SO_REUSEADDR");
        }

        // Bind socket
        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket to port %d", port_);
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }

        return true;
    }

    void receive_loop() {
        char buffer[65536];  // Increased to 64KB to handle large ARKit packets
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        RCLCPP_INFO(this->get_logger(), "UDP receiver thread started, waiting for data...");

        while (running_) {
            ssize_t n = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                                (struct sockaddr*)&client_addr, &client_len);

            if (n > 0) {
                char client_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
                RCLCPP_DEBUG(this->get_logger(), "Received %zd bytes from %s:%d",
                            n, client_ip, ntohs(client_addr.sin_port));

                // Detect format based on first character
                if (n > 0 && buffer[0] == '{') {
                    // JSON format
                    process_json_packet(buffer, n);
                } else if (n > 8 && strncmp(buffer, "#bundle", 7) == 0) {
                    // OSC bundle format
                    process_osc_bundle(buffer, n);
                } else if (n > 0 && buffer[0] == '/') {
                    // OSC message format
                    process_osc_message(buffer, n);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                        "Unknown data format (first byte: 0x%02x)", (unsigned char)buffer[0]);
                }
            } else if (n < 0 && running_) {
                RCLCPP_ERROR(this->get_logger(), "recvfrom error: %s", strerror(errno));
            }
        }
    }

    void process_json_packet(const char* data, size_t size) {
        try {
            // Parse JSON
            json j = json::parse(std::string(data, size));

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Received JSON packet with %zu fields", j.size());

            auto now = this->now();

            // ZigSim Pro nests sensor data under "sensordata" key
            json sensor_data = j;
            if (j.contains("sensordata")) {
                sensor_data = j["sensordata"];
            }

            // Process quaternion
            if (sensor_data.contains("quaternion")) {
                auto q = sensor_data["quaternion"];
                if (q.contains("x") && q.contains("y") && q.contains("z") && q.contains("w")) {
                    publish_quaternion_json(q, now);
                }
            }

            // Process attitude (roll, pitch, yaw)
            if (sensor_data.contains("attitude")) {
                auto att = sensor_data["attitude"];
                if (att.contains("roll") && att.contains("pitch") && att.contains("yaw")) {
                    publish_attitude_json(att, now);
                }
            }

            // Process accelerometer
            if (sensor_data.contains("accelerometer") || sensor_data.contains("accel")) {
                auto accel = sensor_data.contains("accelerometer") ? sensor_data["accelerometer"] : sensor_data["accel"];
                if (accel.contains("x") && accel.contains("y") && accel.contains("z")) {
                    publish_accel_json(accel, now);
                }
            }

            // Process gyroscope
            if (sensor_data.contains("gyroscope") || sensor_data.contains("gyro")) {
                auto gyro = sensor_data.contains("gyroscope") ? sensor_data["gyroscope"] : sensor_data["gyro"];
                if (gyro.contains("x") && gyro.contains("y") && gyro.contains("z")) {
                    publish_gyro_json(gyro, now);
                }
            }

            // Process magnetometer (also check compass for magnetic heading)
            if (sensor_data.contains("magnetometer") || sensor_data.contains("mag")) {
                auto mag = sensor_data.contains("magnetometer") ? sensor_data["magnetometer"] : sensor_data["mag"];
                if (mag.contains("x") && mag.contains("y") && mag.contains("z")) {
                    publish_magnetometer_json(mag, now);
                }
            }

            // Process GPS/location
            if (sensor_data.contains("gps") || sensor_data.contains("location")) {
                auto gps = sensor_data.contains("gps") ? sensor_data["gps"] : sensor_data["location"];
                if (gps.contains("latitude") && gps.contains("longitude")) {
                    publish_gps_json(gps, now);
                }
            }

            // Process gravity
            if (sensor_data.contains("gravity")) {
                auto gravity = sensor_data["gravity"];
                if (gravity.contains("x") && gravity.contains("y") && gravity.contains("z")) {
                    publish_gravity_json(gravity, now);
                }
            }

            // Process pressure
            if (sensor_data.contains("pressure")) {
                auto pressure = sensor_data["pressure"];
                if (pressure.contains("pressure")) {
                    publish_pressure_json(pressure, now);
                }
            }

            // Process compass
            if (sensor_data.contains("compass")) {
                auto compass = sensor_data["compass"];
                if (compass.contains("compass")) {
                    publish_compass_json(compass, now);
                }
            }

            // Process ARKit data if available - combine position and rotation into pose
            if (sensor_data.contains("arkit")) {
                auto arkit = sensor_data["arkit"];
                bool has_rotation = arkit.contains("rotation") && arkit["rotation"].is_array() && arkit["rotation"].size() >= 4;
                bool has_position = arkit.contains("position") && arkit["position"].is_array() && arkit["position"].size() >= 3;

                if (has_rotation || has_position) {
                    publish_arkit_pose_json(arkit, now);
                }
            }

        } catch (const json::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Failed to parse JSON packet (%zu bytes): %s", size, e.what());
        }
    }

    void process_osc_message(const char* data, size_t size) {
        OSCMessage msg;
        if (!parse_osc_message_data(data, size, msg)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Failed to parse OSC message");
            return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "OSC: %s [%zu args]", msg.address.c_str(), msg.args.size());

        auto now = this->now();
        handle_osc_message(msg, now);
    }

    void process_osc_bundle(const char* data, size_t size) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Received OSC bundle (%zu bytes)", size);

        // Skip bundle header and timetag
        size_t pos = 16; // "#bundle\0" + 8-byte timetag

        auto now = this->now();

        // Process each element in the bundle
        while (pos + 4 <= size) {
            // Read element size
            uint32_t element_size;
            std::memcpy(&element_size, &data[pos], 4);
            element_size = ntohl(element_size);
            pos += 4;

            if (pos + element_size > size) break;

            // Parse the element as an OSC message
            OSCMessage msg;
            if (parse_osc_message_data(data + pos, element_size, msg)) {
                handle_osc_message(msg, now);
            }

            pos += element_size;
        }
    }

    void handle_osc_message(const OSCMessage& msg, rclcpp::Time stamp) {
        if (msg.address == "/quaternion" && msg.args.size() >= 4) {
            publish_quaternion_osc(msg.args, stamp);
        }
        else if (msg.address == "/attitude" && msg.args.size() >= 3) {
            publish_attitude_osc(msg.args, stamp);
        }
        else if (msg.address == "/gyro" && msg.args.size() >= 3) {
            publish_gyro_osc(msg.args, stamp);
        }
        else if (msg.address == "/accel" && msg.args.size() >= 3) {
            publish_accel_osc(msg.args, stamp);
        }
        else if (msg.address == "/magnetometer" && msg.args.size() >= 3) {
            publish_magnetometer_osc(msg.args, stamp);
        }
        else if ((msg.address == "/gps" || msg.address == "/location") && msg.args.size() >= 3) {
            publish_gps_osc(msg.args, stamp);
        }
    }

    // JSON-based publish functions
    void publish_quaternion_json(const json& q, rclcpp::Time stamp) {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.orientation.x = q["x"].get<double>();
        pose_msg.pose.orientation.y = q["y"].get<double>();
        pose_msg.pose.orientation.z = q["z"].get<double>();
        pose_msg.pose.orientation.w = q["w"].get<double>();
        pose_pub_->publish(pose_msg);
    }

    void publish_attitude_json(const json& att, rclcpp::Time stamp) {
        // Convert Euler angles (roll, pitch, yaw) to quaternion
        double roll = att["roll"].get<double>();
        double pitch = att["pitch"].get<double>();
        double yaw = att["yaw"].get<double>();

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy;
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy;
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy;
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy;
        pose_pub_->publish(pose_msg);
    }

    void publish_gyro_json(const json& gyro, rclcpp::Time stamp) {
        imu_msg_.header.stamp = stamp;
        imu_msg_.header.frame_id = frame_id_;
        imu_msg_.angular_velocity.x = gyro["x"].get<double>();
        imu_msg_.angular_velocity.y = gyro["y"].get<double>();
        imu_msg_.angular_velocity.z = gyro["z"].get<double>();
        imu_pub_->publish(imu_msg_);
    }

    void publish_accel_json(const json& accel, rclcpp::Time stamp) {
        imu_msg_.header.stamp = stamp;
        imu_msg_.header.frame_id = frame_id_;
        imu_msg_.linear_acceleration.x = accel["x"].get<double>();
        imu_msg_.linear_acceleration.y = accel["y"].get<double>();
        imu_msg_.linear_acceleration.z = accel["z"].get<double>();
        imu_pub_->publish(imu_msg_);
    }

    void publish_magnetometer_json(const json& mag, rclcpp::Time stamp) {
        auto mag_msg = sensor_msgs::msg::MagneticField();
        mag_msg.header.stamp = stamp;
        mag_msg.header.frame_id = frame_id_;
        mag_msg.magnetic_field.x = mag["x"].get<double>();
        mag_msg.magnetic_field.y = mag["y"].get<double>();
        mag_msg.magnetic_field.z = mag["z"].get<double>();
        mag_pub_->publish(mag_msg);
    }

    void publish_gps_json(const json& gps, rclcpp::Time stamp) {
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = stamp;
        gps_msg.header.frame_id = frame_id_;
        gps_msg.latitude = gps["latitude"].get<double>();
        gps_msg.longitude = gps["longitude"].get<double>();
        if (gps.contains("altitude")) {
            gps_msg.altitude = gps["altitude"].get<double>();
        }
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        gps_pub_->publish(gps_msg);
    }

    void publish_gravity_json(const json& gravity, rclcpp::Time stamp) {
        auto gravity_msg = geometry_msgs::msg::Vector3Stamped();
        gravity_msg.header.stamp = stamp;
        gravity_msg.header.frame_id = frame_id_;
        gravity_msg.vector.x = gravity["x"].get<double>();
        gravity_msg.vector.y = gravity["y"].get<double>();
        gravity_msg.vector.z = gravity["z"].get<double>();
        gravity_pub_->publish(gravity_msg);
    }

    void publish_pressure_json(const json& pressure, rclcpp::Time stamp) {
        auto pressure_msg = sensor_msgs::msg::FluidPressure();
        pressure_msg.header.stamp = stamp;
        pressure_msg.header.frame_id = frame_id_;
        pressure_msg.fluid_pressure = pressure["pressure"].get<double>() * 100.0; // Convert hPa to Pa
        if (pressure.contains("altitude")) {
            // Note: FluidPressure doesn't have altitude field, just publish pressure
        }
        pressure_pub_->publish(pressure_msg);
    }

    void publish_compass_json(const json& compass, rclcpp::Time stamp) {
        auto compass_msg = std_msgs::msg::Float64();
        compass_msg.data = compass["compass"].get<double>();
        compass_pub_->publish(compass_msg);
    }

    void publish_arkit_pose_json(const json& arkit, rclcpp::Time stamp) {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = frame_id_;

        // Set position if available
        if (arkit.contains("position") && arkit["position"].is_array() && arkit["position"].size() >= 3) {
            auto pos = arkit["position"];
            pose_msg.pose.position.x = pos[0].get<double>();
            pose_msg.pose.position.y = pos[1].get<double>();
            pose_msg.pose.position.z = pos[2].get<double>();
        } else {
            // Default position to origin
            pose_msg.pose.position.x = 0.0;
            pose_msg.pose.position.y = 0.0;
            pose_msg.pose.position.z = 0.0;
        }

        // Set orientation if available (ARKit rotation is [x, y, z, w])
        if (arkit.contains("rotation") && arkit["rotation"].is_array() && arkit["rotation"].size() >= 4) {
            auto rot = arkit["rotation"];
            pose_msg.pose.orientation.x = rot[0].get<double>();
            pose_msg.pose.orientation.y = rot[1].get<double>();
            pose_msg.pose.orientation.z = rot[2].get<double>();
            pose_msg.pose.orientation.w = rot[3].get<double>();
        } else {
            // Default orientation to identity quaternion
            pose_msg.pose.orientation.x = 0.0;
            pose_msg.pose.orientation.y = 0.0;
            pose_msg.pose.orientation.z = 0.0;
            pose_msg.pose.orientation.w = 1.0;
        }

        arkit_pose_pub_->publish(pose_msg);
    }

    // OSC-based publish functions
    void publish_quaternion_osc(const std::vector<float>& args, rclcpp::Time stamp) {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.orientation.x = args[0];
        pose_msg.pose.orientation.y = args[1];
        pose_msg.pose.orientation.z = args[2];
        pose_msg.pose.orientation.w = args[3];
        pose_pub_->publish(pose_msg);
    }

    void publish_attitude_osc(const std::vector<float>& args, rclcpp::Time stamp) {
        double roll = args[0];
        double pitch = args[1];
        double yaw = args[2];

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy;
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy;
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy;
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy;
        pose_pub_->publish(pose_msg);
    }

    void publish_gyro_osc(const std::vector<float>& args, rclcpp::Time stamp) {
        imu_msg_.header.stamp = stamp;
        imu_msg_.header.frame_id = frame_id_;
        imu_msg_.angular_velocity.x = args[0];
        imu_msg_.angular_velocity.y = args[1];
        imu_msg_.angular_velocity.z = args[2];
        imu_pub_->publish(imu_msg_);
    }

    void publish_accel_osc(const std::vector<float>& args, rclcpp::Time stamp) {
        imu_msg_.header.stamp = stamp;
        imu_msg_.header.frame_id = frame_id_;
        imu_msg_.linear_acceleration.x = args[0];
        imu_msg_.linear_acceleration.y = args[1];
        imu_msg_.linear_acceleration.z = args[2];
        imu_pub_->publish(imu_msg_);
    }

    void publish_magnetometer_osc(const std::vector<float>& args, rclcpp::Time stamp) {
        auto mag_msg = sensor_msgs::msg::MagneticField();
        mag_msg.header.stamp = stamp;
        mag_msg.header.frame_id = frame_id_;
        mag_msg.magnetic_field.x = args[0];
        mag_msg.magnetic_field.y = args[1];
        mag_msg.magnetic_field.z = args[2];
        mag_pub_->publish(mag_msg);
    }

    void publish_gps_osc(const std::vector<float>& args, rclcpp::Time stamp) {
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = stamp;
        gps_msg.header.frame_id = frame_id_;
        gps_msg.latitude = args[0];
        gps_msg.longitude = args[1];
        if (args.size() > 2) {
            gps_msg.altitude = args[2];
        }
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        gps_pub_->publish(gps_msg);
    }

    // Parameters
    std::string host_;
    int port_;
    std::string frame_id_;

    // Socket
    int sockfd_ = -1;
    std::atomic<bool> running_;
    std::thread receiver_thread_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arkit_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr compass_pub_;

    // Message cache
    sensor_msgs::msg::Imu imu_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZigSimOSCReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
