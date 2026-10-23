
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFilterNode : public rclcpp::Node
{
public:
    ScanFilterNode() : Node("scan_filter_node")
    {
        min_range_ = declare_parameter<double>("min_range", 1);
        max_range_ = declare_parameter<double>("max_range", 12.0);

        if (min_range_ < 0.0 || max_range_ <= min_range_) {
            throw std::invalid_argument(
                "Require min_range >= 0 and max_range > min_range");
        }

        subscription_ =
            create_subscription<sensor_msgs::msg::LaserScan>(
                "scan",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &ScanFilterNode::scan_callback,
                    this,
                    std::placeholders::_1));

        publisher_ =
            create_publisher<sensor_msgs::msg::LaserScan>(
                "scan_filtered",
                rclcpp::SensorDataQoS());

        RCLCPP_INFO(
            get_logger(),
            "Scan filter started: min=%.2f m, max=%.2f m",
            min_range_, max_range_);
    }

private:

void scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    auto filtered =
        std::make_unique<sensor_msgs::msg::LaserScan>(*msg);

    const std::size_t total_count = msg->ranges.size();
    std::size_t valid_count = 0;
    std::size_t filtered_count = 0;

    // 更新输出消息中的有效测距范围
    filtered->range_min = static_cast<float>(min_range_);
    filtered->range_max = static_cast<float>(max_range_);

    for (float &range : filtered->ranges) {
        if (!std::isfinite(range) ||
            range < min_range_ ||
            range > max_range_) {
            range = std::numeric_limits<float>::infinity();
            ++filtered_count;
        } else {
            ++valid_count;
        }
    }

    publisher_->publish(std::move(filtered));

    const double keep_ratio =
        total_count > 0
        ? 100.0 * static_cast<double>(valid_count) /
              static_cast<double>(total_count)
        : 0.0;

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Total: %zu, kept: %zu, filtered: %zu, keep ratio: %.1f%%",
        total_count,
        valid_count,
        filtered_count,
        keep_ratio);
}


    double min_range_;
    double max_range_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        subscription_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFilterNode>());
    rclcpp::shutdown();
    return 0;
}