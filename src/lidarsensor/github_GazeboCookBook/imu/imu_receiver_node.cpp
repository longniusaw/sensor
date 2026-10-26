
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuReceiver : public rclcpp::Node
{
public:
    ImuReceiver()
        : Node("imu_receiver_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            rclcpp::SensorDataQoS(),
            std::bind(
                &ImuReceiver::imu_callback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "IMU receiver started. Listening to /imu");
    }

private:
    // 判断两个浮点数的差异是否超过阈值
    bool value_changed(double current, double previous, double threshold)
    {
        return std::abs(current - previous) > threshold;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 第一次收到数据：保存并输出
        if (!has_previous_data_)
        {
            previous_msg_ = *msg;
            has_previous_data_ = true;

            print_imu_data(msg);
            return;
        }

        // 检查角速度是否发生明显变化
        const auto &current_angular = msg->angular_velocity;
        const auto &previous_angular = previous_msg_.angular_velocity;

        bool changed =
            value_changed(current_angular.x, previous_angular.x, 0.001) ||
            value_changed(current_angular.y, previous_angular.y, 0.001) ||
            value_changed(current_angular.z, previous_angular.z, 0.001);

        // 检查线性加速度是否发生明显变化
        const auto &current_accel = msg->linear_acceleration;
        const auto &previous_accel = previous_msg_.linear_acceleration;

        changed = changed ||
            value_changed(current_accel.x, previous_accel.x, 0.01) ||
            value_changed(current_accel.y, previous_accel.y, 0.01) ||
            value_changed(current_accel.z, previous_accel.z, 0.01);

        // 检查姿态四元数是否发生明显变化
        const auto &current_orientation = msg->orientation;
        const auto &previous_orientation = previous_msg_.orientation;

        changed = changed ||
            value_changed(current_orientation.x, previous_orientation.x, 0.001) ||
            value_changed(current_orientation.y, previous_orientation.y, 0.001) ||
            value_changed(current_orientation.z, previous_orientation.z, 0.001) ||
            value_changed(current_orientation.w, previous_orientation.w, 0.001);

        // 只有数据发生明显变化时才输出
        if (changed)
        {
            print_imu_data(msg);

            // 更新参考数据，供下一次比较
            previous_msg_ = *msg;
        }
    }

    void print_imu_data(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Timestamp: %d.%09u | Frame: %s",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            msg->header.frame_id.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Angular velocity [rad/s]: x=%.4f, y=%.4f, z=%.4f",
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        RCLCPP_INFO(
            this->get_logger(),
            "Linear acceleration [m/s^2]: x=%.4f, y=%.4f, z=%.4f",
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        RCLCPP_INFO(
            this->get_logger(),
            "Orientation quaternion: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
    }

    bool has_previous_data_ = false;

    // 保存上一次输出的数据
    sensor_msgs::msg::Imu previous_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ImuReceiver>());

    rclcpp::shutdown();
    return 0;
}
