#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include "visualization_msgs/msg/marker_array.hpp"
#include <functional>                          // std::bind, std::placeholders
#include "geometry_msgs/msg/point.hpp"         // geometry_msgs::msg::Point

// 该节点负责把初始数据全部转化成markerarray数据并发布

class lidardatainput : public rclcpp::Node
{
public:
    lidardatainput() : Node("lidar_data_receive")
    {
        // 设置QOS消息机制，quality or speed
        auto qos = rclcpp::SensorDataQoS();

        // this->create_subscription是调用的rclpp的node的基类方法，<sensor_msgs::msg::LaserScan>是消息类型，意思是调用这个方法去订阅这个消息类型的消息
        subscription_ = this->create_subscription
            <sensor_msgs::msg::LaserScan>
            (
                "/bcr_bot/scan",
                qos,
                std::bind(
                    &lidardatainput::forward_receive_callback,
                    this,
                    std::placeholders::_1
                )
            );

        marker_publisher = this->create_publisher
            <visualization_msgs::msg::MarkerArray>
            (
                "/sensor/forward/marker",
                qos
            );

        RCLCPP_INFO(this->get_logger(), "Lidar data forward started");
    }

private:
    void forward_receive_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Lidar data received"
        );
        forward_markerarray(msg);
    }

    void forward_markerarray(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 建立array数组
        visualization_msgs::msg::MarkerArray array;

        // ---- 1. 先放一个 DELETEALL，清掉上一帧所有同 ns 的 marker ----
        // 否则旧帧的点会残留，RViz 里会越积越多
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        array.markers.push_back(clear);

        // ---- 2. 构造本帧的 marker ----
        visualization_msgs::msg::Marker m;
        m.header = msg->header;            // 直接复用雷达的 frame_id + 时间戳
        m.ns     = "scan_points";          // 命名空间，同 ns 同 id 视为同一个 marker
        m.id     = 0;
        m.type   = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.scale.x = m.scale.y = m.scale.z = 0.05;  // 每个球直径 5cm
        // 绿色
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;                  // 透明度，别忘了    // alpha 必须设，0 会完全透明看不见

        // ---- 3. 遍历 ranges，转成点 ----
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];

            // 过滤无效值：inf（超出量程）、nan、小于最小量程
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
                continue;

            float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;

            geometry_msgs::msg::Point p;
            p.x = r * std::cos(angle);
            p.y = r * std::sin(angle);
            p.z = 0.0;
            m.points.push_back(p);
        }
        // 把处理后的点塞进建立的数组
        array.markers.push_back(m);

        // 调用marker_publisher的publish方法把数组的点发出去
        marker_publisher->publish(array);
    }

    // ROS2订阅对象
    rclcpp::Subscription
    <sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<lidardatainput>()
    );

    rclcpp::shutdown();

    return 0;
}