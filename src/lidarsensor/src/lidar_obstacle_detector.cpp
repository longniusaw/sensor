#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>



class LidarObstacleDetector : public rclcpp::Node
{

public:

    // 构造函数
    LidarObstacleDetector()
    : Node("lidar_obstacle_detector")
    {
        /*
         创建订阅者
         订阅:
         /scan_filtered

         消息类型:
         sensor_msgs::msg::LaserScan

         参数10:
         QoS队列长度
         表示最多缓存10条消息
        */
        auto qos = rclcpp::SensorDataQoS();

        subscription_ =
            this->create_subscription
            <sensor_msgs::msg::LaserScan>
            (
                "/scan_filtered",
                qos,
                std::bind(
                    &LidarObstacleDetector::scan_callback,
                    this,
                    std::placeholders::_1
                )
            );
        RCLCPP_INFO(
            this->get_logger(),
            "Lidar obstacle detector started"
        );
    }

private:
    /*
        雷达数据回调函数
        每收到一次LaserScan数据
        这个函数执行一次
    */
    void scan_callback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg
    )
    {

        // 设置区域数量，const是确保该变量不会被改变
        const int sector_count = 18;

        // 创建18个区域的有效点统计，std::vector<int>是指这是一个装int类型的动态数组，valid_points是数组的变量名，sector是数组大小，0是数组初始值
        std::vector<int> valid_points(sector_count,0);

        // 创建18个区域的总点数量统计
        std::vector<int> total_points(sector_count,0);

        /*
            遍历所有雷达扫描点
        */

        for(size_t i=0;i<msg->ranges.size();i++)
        {
            float range = msg->ranges[i];
            /*
                计算当前点角度
                angle =
                起始角度
                +
                index * 角分辨率
            */
            float angle =
                msg->angle_min
                +
                i * msg->angle_increment;
            /*
                将角度转换成0~360范围
            */
            if(angle < 0)
            {
                angle += 2*M_PI;
            }
            /*
                根据角度计算属于哪个区域
                0~360°
                每个区域20°
            */
            int sector_id =
                static_cast<int>
                (
                    angle /
                    (2*M_PI / sector_count)
                );
            if(sector_id >= sector_count)
            {
                sector_id = sector_count-1;
            }
            total_points[sector_id]++;
            /*
                判断这个点是否有效
            */
            if(
                !std::isnan(range)
                &&
                !std::isinf(range)
                &&
                range > msg->range_min
                &&
                range < msg->range_max
            )
            {
                valid_points[sector_id]++;
            }
        }

        /*
            判断每个区域是否存在障碍物
        */

        for(int i=0;i<sector_count;i++)
        {

            float ratio = 0;

            if(total_points[i] > 0)
            {
                ratio =
                static_cast<float>(valid_points[i])
                /
                total_points[i];
            }

            bool obstacle = false;

            /*
                有效率超过70%
                判断为障碍区域
            */

            if(ratio > 0.7)
            {
                obstacle=true;
            }

            RCLCPP_INFO(

                this->get_logger(),

                "sector[%d] valid=%d total=%d ratio=%.2f obstacle=%s",

                i,

                valid_points[i],

                total_points[i],

                ratio,

                obstacle?"YES":"NO"

            );
        }
    }

    // ROS2订阅对象
    rclcpp::Subscription
    <sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

};





int main(int argc,char **argv)
{

    rclcpp::init(argc,argv);

    rclcpp::spin(
        std::make_shared<LidarObstacleDetector>()
    );

    rclcpp::shutdown();

    return 0;

}