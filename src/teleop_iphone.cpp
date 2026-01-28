#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <arx_ros2/msg/cartesian_command.hpp>
#include <arx_ros2/srv/reset_to_home.hpp>

class TeleopIphone : public rclcpp::Node {
   public:
    TeleopIphone() : Node("teleop_iphone") {
        // Declare parameters
        // iPhone TF frames
        this->declare_parameter<std::string>("iphone_source_frame", "iphone_odom");
        this->declare_parameter<std::string>("iphone_target_frame", "iphone_ros");
        // Arm TF frames
        this->declare_parameter<std::string>("arm_base_frame", "base_link");
        this->declare_parameter<std::string>("arm_ee_frame", "link6");
        // Control parameters
        this->declare_parameter<std::string>("command_topic", "/arx5_ros2_node/cartesian_command");
        this->declare_parameter<std::string>("reset_service_name", "/arx5_ros2_node/reset_to_home");
        this->declare_parameter<double>("touch_threshold", 0.3);
        this->declare_parameter<double>("touch_timeout", 0.05);
        this->declare_parameter<double>("position_scale", 1.0);
        this->declare_parameter<double>("rotation_scale", 1.0);
        this->declare_parameter<double>("gripper_open", 0.3);
        this->declare_parameter<double>("gripper_close", 0.0);
        this->declare_parameter<double>("tf_update_rate", 100.0);
        this->declare_parameter<int>("control_rate", 50);

        // Get parameters
        iphone_source_frame_ = this->get_parameter("iphone_source_frame").as_string();
        iphone_target_frame_ = this->get_parameter("iphone_target_frame").as_string();
        arm_base_frame_      = this->get_parameter("arm_base_frame").as_string();
        arm_ee_frame_        = this->get_parameter("arm_ee_frame").as_string();
        touch_threshold_     = this->get_parameter("touch_threshold").as_double();
        touch_timeout_       = this->get_parameter("touch_timeout").as_double();
        position_scale_      = this->get_parameter("position_scale").as_double();
        rotation_scale_      = this->get_parameter("rotation_scale").as_double();
        gripper_open_        = this->get_parameter("gripper_open").as_double();
        gripper_close_       = this->get_parameter("gripper_close").as_double();
        double tf_rate       = this->get_parameter("tf_update_rate").as_double();
        int    control_rate  = this->get_parameter("control_rate").as_int();
        command_topic_       = this->get_parameter("command_topic").as_string();
        reset_service_name_  = this->get_parameter("reset_service_name").as_string();

        // TF2 setup
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        cmd_pub_ = this->create_publisher<arx_ros2::msg::CartesianCommand>(command_topic_, 10);

        // Subscribers
        touch_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/iphone/touch", 10, std::bind(&TeleopIphone::touch_callback, this, std::placeholders::_1));

        // Services
        reset_client_ = this->create_client<arx_ros2::srv::ResetToHome>(reset_service_name_);

        // Timer for TF updates
        auto tf_period = std::chrono::duration<double>(1.0 / tf_rate);
        tf_timer_      = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(tf_period),
                                                 std::bind(&TeleopIphone::tf_timer_callback, this));
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / control_rate)),
            std::bind(&TeleopIphone::control_timer_callback, this));
        last_touch_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "TeleopIphone node started");
        RCLCPP_INFO(this->get_logger(), "  iPhone frames: %s -> %s", iphone_source_frame_.c_str(),
                    iphone_target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Arm frames: %s -> %s", arm_base_frame_.c_str(), arm_ee_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Reset service: %s", reset_service_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Touch threshold: %.2f", touch_threshold_);
        RCLCPP_INFO(this->get_logger(), "  Touch timeout: %.2f s", touch_timeout_);
        RCLCPP_INFO(this->get_logger(), "  Position scale: %.2f", position_scale_);
        RCLCPP_INFO(this->get_logger(), "  Rotation scale: %.2f", rotation_scale_);
        RCLCPP_INFO(this->get_logger(), "  TF update rate: %.1f Hz", tf_rate);
        RCLCPP_INFO(this->get_logger(), "  Control rate: %d Hz", control_rate);
    }

   private:
    void tf_timer_callback() {
        std::lock_guard<std::mutex> lock(tf_mutex_);

        // Update iPhone TF
        try {
            auto iphone_tf =
                tf_buffer_->lookupTransform(iphone_source_frame_, iphone_target_frame_, tf2::TimePointZero);

            iphone_x_ = iphone_tf.transform.translation.x;
            iphone_y_ = iphone_tf.transform.translation.y;
            iphone_z_ = iphone_tf.transform.translation.z;

            tf2::Quaternion q_iphone(iphone_tf.transform.rotation.x, iphone_tf.transform.rotation.y,
                                     iphone_tf.transform.rotation.z, iphone_tf.transform.rotation.w);
            tf2::Matrix3x3(q_iphone).getRPY(iphone_roll_, iphone_pitch_, iphone_yaw_);

            iphone_tf_valid_ = true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not get iPhone transform: %s",
                                 ex.what());
            iphone_tf_valid_ = false;
        }

        // Update arm EE TF
        try {
            auto arm_tf = tf_buffer_->lookupTransform(arm_base_frame_, arm_ee_frame_, tf2::TimePointZero);

            ee_x_ = arm_tf.transform.translation.x;
            ee_y_ = arm_tf.transform.translation.y;
            ee_z_ = arm_tf.transform.translation.z;

            tf2::Quaternion q_ee(arm_tf.transform.rotation.x, arm_tf.transform.rotation.y, arm_tf.transform.rotation.z,
                                 arm_tf.transform.rotation.w);
            tf2::Matrix3x3(q_ee).getRPY(ee_roll_, ee_pitch_, ee_yaw_);

            arm_tf_valid_ = true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not get arm transform: %s",
                                 ex.what());
            arm_tf_valid_ = false;
        }
    }

    void control_timer_callback() {
        const auto now             = this->now();
        bool       has_valid_touch = false;
        size_t     touch_count     = 0;
        bool       has_touch_msg   = false;
        auto       last_touch_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        {
            std::lock_guard<std::mutex> lock(touch_mutex_);
            has_valid_touch = has_valid_touch_;
            touch_count     = touch_count_;
            has_touch_msg   = has_touch_message_;
            last_touch_time = last_touch_time_;
        }

        if (!has_touch_msg || (now - last_touch_time).seconds() > touch_timeout_) {
            if (is_pressing_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Touch data timeout");
            }
            is_pressing_ = false;
            return;
        }

        if (reset_block_active_) {
            if (!has_valid_touch) {
                reset_block_active_ = false;
                reset_sent_         = false;
            }
            if (now < reset_cooldown_until_) {
                return;
            }
            return;
        }

        // Handle release
        if (!has_valid_touch) {
            if (is_pressing_) {
                RCLCPP_INFO(this->get_logger(), "Touch released");
            }
            is_pressing_ = false;
            reset_sent_  = false;
            return;
        }

        if (touch_count >= 3) {
            if (!reset_sent_) {
                send_reset_to_home();
                reset_sent_ = true;
            }
            return;
        }

        // Snapshot TF values
        bool   iphone_tf_valid = false;
        bool   arm_tf_valid    = false;
        double iphone_x        = 0.0;
        double iphone_y        = 0.0;
        double iphone_z        = 0.0;
        double iphone_roll     = 0.0;
        double iphone_pitch    = 0.0;
        double iphone_yaw      = 0.0;
        double ee_x            = 0.0;
        double ee_y            = 0.0;
        double ee_z            = 0.0;
        double ee_roll         = 0.0;
        double ee_pitch        = 0.0;
        double ee_yaw          = 0.0;
        {
            std::lock_guard<std::mutex> lock(tf_mutex_);
            iphone_tf_valid = iphone_tf_valid_;
            arm_tf_valid    = arm_tf_valid_;
            if (iphone_tf_valid) {
                iphone_x     = iphone_x_;
                iphone_y     = iphone_y_;
                iphone_z     = iphone_z_;
                iphone_roll  = iphone_roll_;
                iphone_pitch = iphone_pitch_;
                iphone_yaw   = iphone_yaw_;
            }
            if (arm_tf_valid) {
                ee_x     = ee_x_;
                ee_y     = ee_y_;
                ee_z     = ee_z_;
                ee_roll  = ee_roll_;
                ee_pitch = ee_pitch_;
                ee_yaw   = ee_yaw_;
            }
        }

        if (!iphone_tf_valid) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "iPhone TF not available");
            return;
        }

        // Handle first press - record start poses
        if (!is_pressing_) {
            if (!arm_tf_valid) {
                RCLCPP_WARN(this->get_logger(), "Arm TF not available on press");
                return;
            }

            is_pressing_ = true;
            reset_sent_  = false;

            // Record iPhone start pose
            start_iphone_x_     = iphone_x;
            start_iphone_y_     = iphone_y;
            start_iphone_z_     = iphone_z;
            start_iphone_roll_  = iphone_roll;
            start_iphone_pitch_ = iphone_pitch;
            start_iphone_yaw_   = iphone_yaw;

            // Record current EE pose
            start_ee_x_     = ee_x;
            start_ee_y_     = ee_y;
            start_ee_z_     = ee_z;
            start_ee_roll_  = ee_roll;
            start_ee_pitch_ = ee_pitch;
            start_ee_yaw_   = ee_yaw;

            RCLCPP_INFO(this->get_logger(), "Touch started - iPhone: [%.3f, %.3f, %.3f], EE: [%.3f, %.3f, %.3f]",
                        start_iphone_x_, start_iphone_y_, start_iphone_z_, start_ee_x_, start_ee_y_, start_ee_z_);
        }

        // Calculate relative motion from iPhone start pose
        double delta_x     = (iphone_x - start_iphone_x_) * position_scale_;
        double delta_y     = (iphone_y - start_iphone_y_) * position_scale_;
        double delta_z     = (iphone_z - start_iphone_z_) * position_scale_;
        double delta_roll  = (iphone_roll - start_iphone_roll_) * rotation_scale_;
        double delta_pitch = (iphone_pitch - start_iphone_pitch_) * rotation_scale_;
        double delta_yaw   = (iphone_yaw - start_iphone_yaw_) * rotation_scale_;

        // Apply delta to EE start pose
        double target_x     = start_ee_x_ + delta_x;
        double target_y     = start_ee_y_ + delta_y;
        double target_z     = start_ee_z_ + delta_z;
        double target_roll  = start_ee_roll_ + delta_roll;
        double target_pitch = start_ee_pitch_ + delta_pitch;
        double target_yaw   = start_ee_yaw_ + delta_yaw;

        // Convert RPY to quaternion
        tf2::Quaternion q_target;
        q_target.setRPY(target_roll, target_pitch, target_yaw);

        // Create and publish CartesianCommand
        auto cmd_msg                      = arx_ros2::msg::CartesianCommand();
        cmd_msg.header.stamp              = this->now();
        cmd_msg.header.frame_id           = arm_base_frame_;
        cmd_msg.target_pose.position.x    = target_x;
        cmd_msg.target_pose.position.y    = target_y;
        cmd_msg.target_pose.position.z    = target_z;
        cmd_msg.target_pose.orientation.x = q_target.x();
        cmd_msg.target_pose.orientation.y = q_target.y();
        cmd_msg.target_pose.orientation.z = q_target.z();
        cmd_msg.target_pose.orientation.w = q_target.w();

        // Gripper control: 2 touches = close, otherwise open
        cmd_msg.gripper_position = (touch_count >= 2) ? gripper_open_ : gripper_close_;

        cmd_msg.timestamp = 0.0;

        cmd_pub_->publish(cmd_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Cmd: pos[%.3f, %.3f, %.3f] delta[%.3f, %.3f, %.3f] gripper=%.3f touches=%zu", target_x,
                             target_y, target_z, delta_x, delta_y, delta_z, cmd_msg.gripper_position, touch_count);
    }

    void send_reset_to_home() {
        if (!reset_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "ResetToHome service not available");
            return;
        }

        auto request = std::make_shared<arx_ros2::srv::ResetToHome::Request>();
        (void)reset_client_->async_send_request(
            request, [this](rclcpp::Client<arx_ros2::srv::ResetToHome>::SharedFuture future) {
                const auto& response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Reset to home succeeded");
                    reset_block_active_  = true;
                    reset_cooldown_until_ = this->now() + rclcpp::Duration::from_seconds(1.0);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Reset to home failed: %s", response->message.c_str());
                }
            });
    }

    void touch_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        // Check if any touch has radius > threshold (stored in orientation.w)
        bool   has_valid_touch = false;
        size_t touch_count     = 0;

        if (!msg->poses.empty()) {
            for (const auto& pose : msg->poses) {
                if (pose.orientation.w > touch_threshold_) {
                    has_valid_touch = true;
                    touch_count++;
                }
            }
        }
        {
            std::lock_guard<std::mutex> lock(touch_mutex_);
            has_valid_touch_   = has_valid_touch;
            touch_count_       = touch_count;
            last_touch_time_   = this->now();
            has_touch_message_ = true;
        }
    }

    // Parameters
    std::string iphone_source_frame_;
    std::string iphone_target_frame_;
    std::string arm_base_frame_;
    std::string arm_ee_frame_;
    std::string command_topic_;
    std::string reset_service_name_;
    double      touch_threshold_;
    double      touch_timeout_;
    double      position_scale_;
    double      rotation_scale_;
    double      gripper_open_;
    double      gripper_close_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr                tf_timer_;
    rclcpp::TimerBase::SharedPtr                control_timer_;
    std::mutex                                  tf_mutex_;
    std::mutex                                  touch_mutex_;

    // Current TF values (updated by timer)
    bool   iphone_tf_valid_ = false;
    double iphone_x_        = 0.0;
    double iphone_y_        = 0.0;
    double iphone_z_        = 0.0;
    double iphone_roll_     = 0.0;
    double iphone_pitch_    = 0.0;
    double iphone_yaw_      = 0.0;

    bool   arm_tf_valid_ = false;
    double ee_x_         = 0.0;
    double ee_y_         = 0.0;
    double ee_z_         = 0.0;
    double ee_roll_      = 0.0;
    double ee_pitch_     = 0.0;
    double ee_yaw_       = 0.0;

    // State
    bool is_pressing_ = false;

    // Touch state (updated by touch callback)
    bool         has_valid_touch_   = false;
    size_t       touch_count_       = 0;
    bool         has_touch_message_ = false;
    rclcpp::Time last_touch_time_{0, 0, RCL_SYSTEM_TIME};

    // iPhone start pose (when touch began)
    double start_iphone_x_     = 0.0;
    double start_iphone_y_     = 0.0;
    double start_iphone_z_     = 0.0;
    double start_iphone_roll_  = 0.0;
    double start_iphone_pitch_ = 0.0;
    double start_iphone_yaw_   = 0.0;

    // EE start pose (when touch began)
    double start_ee_x_     = 0.0;
    double start_ee_y_     = 0.0;
    double start_ee_z_     = 0.0;
    double start_ee_roll_  = 0.0;
    double start_ee_pitch_ = 0.0;
    double start_ee_yaw_   = 0.0;

    // Publishers and subscribers
    rclcpp::Publisher<arx_ros2::msg::CartesianCommand>::SharedPtr  cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr touch_sub_;
    rclcpp::Client<arx_ros2::srv::ResetToHome>::SharedPtr          reset_client_;

    bool reset_sent_ = false;
    bool reset_block_active_ = false;
    rclcpp::Time reset_cooldown_until_{0, 0, RCL_SYSTEM_TIME};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopIphone>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
