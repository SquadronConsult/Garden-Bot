#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <mutex>

using std::placeholders::_1;

class MavlinkNode : public rclcpp::Node
{
public:
    MavlinkNode() : Node("mavlink_node"), current_state_{}, current_gps_{}
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&MavlinkNode::state_cb, this, _1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "mavros/global_position/global", 10, std::bind(&MavlinkNode::gps_cb, this, _1));
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MavlinkNode::timer_callback, this));

        last_request_ = this->now();
    }

private:
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = *msg;
    }

    void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(gps_mutex_);
        current_gps_ = *msg;
    }

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!current_state_.connected)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
            // Keep trying to detect connection
            return;
        }

        const int REQUEST_INTERVAL_SECONDS = 5;
        auto now = this->now();

        if (current_state_.mode != "OFFBOARD" &&
            (now - last_request_ > rclcpp::Duration::from_seconds(REQUEST_INTERVAL_SECONDS)))
        {
            if (!set_mode_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_ERROR(this->get_logger(), "Set mode service not available!");
                return;
            }

            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = "OFFBOARD";

            auto result = set_mode_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (result.get()->mode_sent)
                {
                    RCLCPP_INFO(this->get_logger(), "Offboard enabled");
                }
            }
            last_request_ = now;
        }
        else if (!current_state_.armed &&
                 (now - last_request_ > rclcpp::Duration::from_seconds(REQUEST_INTERVAL_SECONDS)))
        {
            if (!arming_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_ERROR(this->get_logger(), "Arming service not available!");
                return;
            }

            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;

            auto result = arming_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (result.get()->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Vehicle armed");
                }
            }
            last_request_ = now;
        }

        std::lock_guard<std::mutex> gps_lock(gps_mutex_);
        RCLCPP_INFO(this->get_logger(), "Current GPS: [Lat: %f, Lon: %f, Alt: %f]",
                    current_gps_.latitude, current_gps_.longitude, current_gps_.altitude);
        RCLCPP_INFO(this->get_logger(), "Current mode: %s, Armed: %s",
                    current_state_.mode.c_str(), current_state_.armed ? "true" : "false");
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_request_;
    mavros_msgs::msg::State current_state_;
    sensor_msgs::msg::NavSatFix current_gps_;
    std::mutex state_mutex_;
    std::mutex gps_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
