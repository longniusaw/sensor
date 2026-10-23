#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <cmath>

class LidarObstacleDetector : public rclcpp::Node
{
public:
    LidarObstacleDetector()
    : Node("lidar_obstacle_detector"),
      last_obstacle_state_(18, false)
    {
        /*
         * 创建订阅者
         * 订阅: /scan_filtered
         * 消息类型: sensor_msgs::msg::LaserScan
         */
        subscription_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan_filtered",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &LidarObstacleDetector::scan_callback,
                    this,
                    std::placeholders::_1
                )
            );

        marker_publisher_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                 "/obstacle_marker",
                 10
            );

        RCLCPP_INFO(
            this->get_logger(),
            "Lidar obstacle detector started"
        );
    }

private:
    /*
     * 雷达数据回调函数
     * 每收到一次LaserScan数据执行一次
     */
    void scan_callback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg
    )
    {
        // 划分18个方向区域
        const int sector_count = 18;

        /*
         * 保存当前帧检测结果
         * true: 有障碍物
         * false: 无障碍物
         */
        std::vector<bool> current_obstacle_state(sector_count, false);

        // 每个区域有效点数量
        std::vector<int> valid_points(sector_count, 0);

        // 每个区域总点数量
        std::vector<int> total_points(sector_count, 0);

        // 遍历所有激光点
        for(size_t i = 0; i < msg->ranges.size(); i++)
        {
            float range = msg->ranges[i];

            /*
             * 根据索引计算当前点角度
             * angle = angle_min + index * angle_increment
             */
            float angle = msg->angle_min + i * msg->angle_increment;

            // 将角度转换到0~2PI
            if(angle < 0)
            {
                angle += 2 * M_PI;
            }

            /*
             * 根据角度计算所属区域
             * 360° / 18 = 20°
             */
            int sector_id =
                static_cast<int>(angle / (2 * M_PI / sector_count));

            if(sector_id >= sector_count)
            {
                sector_id = sector_count - 1;
            }

            total_points[sector_id]++;

            // 判断当前激光点是否有效
            if(
                !std::isnan(range) &&
                !std::isinf(range) &&
                range >= msg->range_min &&
                range <= msg->range_max
            )
            {
                valid_points[sector_id]++;
            }
        }

        // 根据有效点比例判断障碍物
        for(int i = 0; i < sector_count; i++)
        {
            float valid_ratio = 0.0;

            if(total_points[i] > 0)
            {
                valid_ratio =
                    static_cast<float>(valid_points[i]) / total_points[i];
            }

            /*
             * 当前规则:
             * 有效率 > 70% 认为该区域存在障碍物
             */
            if(valid_ratio > 0.7)
            {
                current_obstacle_state[i] = true;
            }
        }

        // 比较当前状态和上一帧状态
        for(int i = 0; i < sector_count; i++)
        {
            if(current_obstacle_state[i] != last_obstacle_state_[i])
            {
                if(current_obstacle_state[i])
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Sector %d: obstacle detected",
                        i
                    );
                }
                else
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Sector %d: obstacle cleared",
                        i
                    );
                }
            }
        }

        /*
         * 保存当前状态
         * 给下一帧比较使用
         */
        last_obstacle_state_ = current_obstacle_state;
        publish_markers(current_obstacle_state);
    }

    void publish_markers(
    const std::vector<bool>& obstacle_state)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // 删除上一帧所有marker
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "laser_frame";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);


    for(size_t i = 0; i < obstacle_state.size(); i++)
    {

        if(!obstacle_state[i])
        {
            continue;
        }

        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "laser_frame";
        marker.header.stamp = this->get_clock()->now();

        marker.ns = "obstacles";

        marker.id = i;

        marker.type =
            visualization_msgs::msg::Marker::CYLINDER;

        marker.action =
            visualization_msgs::msg::Marker::ADD;

        /*
         * 计算障碍物显示位置
         *
         * 这里先假设距离:
         * 2米
         *
         */
        double angle =
            i * (2 * M_PI / obstacle_state.size())
            +
            (M_PI / obstacle_state.size());

        double distance = 2.0;

        marker.pose.position.x =
            distance * cos(angle);

        marker.pose.position.y =
            distance * sin(angle);

        marker.pose.position.z =
            0.5;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 1.0;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);

    }

    marker_publisher_->publish(marker_array);
    }

private:
    // ROS2订阅对象
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    // 保存上一帧18个区域状态
    std::vector<bool> last_obstacle_state_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LidarObstacleDetector>());

    rclcpp::shutdown();

    return 0;
}