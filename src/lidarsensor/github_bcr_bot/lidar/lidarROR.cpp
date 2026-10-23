#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include "visualization_msgs/msg/marker_array.hpp"
#include <functional>                          // std::bind, std::placeholders
#include "geometry_msgs/msg/point.hpp"         // geometry_msgs::msg::Point
#include <algorithm>   // std::clamp（如果后面用渐变）
#include <limits>      // std::numeric_limits

// ============================================================================
// 半径离群点去除算法（Radius Outlier Removal, ROR）
//
// 核心思想：
//   对点云里的每一个点，以它自己为圆心，画一个半径为 R 的圆（2D 雷达就是圆，
//   3D 就是球），数一数这个范围内有多少个其他点（邻居）。
//   如果邻居数量少于设定的最小数量 K，说明这个点周围很"空"，
//   大概率是噪声/离群点，就把它删掉（标记为无效）。
//
// 本节点的流程：
//   订阅 /bcr_bot/scan（原始 LaserScan）
//        ↓ 回调
//   把 ranges 转成 (x, y) 点集
//        ↓
//   对每个点统计半径 R 内的邻居数，< K 的标记为离群点
//        ↓
//   发布 MarkerArray 到 /sensor/ROR：
//       内点画绿球，离群点画红球（方便在 RViz 里对比调参）
// ============================================================================

// 当前代码并未删除离群点，而是标记为红色，正常点标记为绿色

class ROR : public rclcpp::Node
{
    public:
    // 构造函数：节点名 "lidarROR"，创建订阅者和发布者
    ROR():Node("lidarROR")
    {
        // 传感器数据用 SensorDataQoS（best_effort），适合高频、可丢帧的雷达数据
        auto qos = rclcpp::SensorDataQoS();

        // ---- 创建订阅者：订阅原始激光扫描 ----
        // 每收到一帧 /bcr_bot/scan，就调用 ROR_receive_callback
        // std::bind 把成员函数包装成回调：this 表示当前对象，_1 代表收到的消息指针
        subscription = this->create_subscription<sensor_msgs::msg::LaserScan>
        (
            "/bcr_bot/scan",
            qos,
            std::bind(
                    &ROR::ROR_receive_callback,
                    this,
                    std::placeholders::_1
                     )  
        );

        // ---- 创建发布者：发布可视化用的 MarkerArray ----
        // 注意：这里沿用 qos（best_effort），RViz 端需要把订阅 QoS 改成 Best Effort
        // 才能收到，否则会报 RELIABILITY_QOS_POLICY 不兼容
        publisher = this->create_publisher
        <visualization_msgs::msg::MarkerArray>
        (
            "/sensor/ROR",
            qos
        );

        RCLCPP_INFO(this->get_logger(),"ROR node begin");
    }

    private:
    // ---- 订阅回调：收到一帧雷达数据后触发 ----
    // 只负责"分发"：打印一条限频日志（避免刷屏），然后把数据交给处理函数
    void ROR_receive_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // THROTTLE：同一行代码最多每 1000ms 打印一次，防止 10~30Hz 的雷达刷屏
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,
        "ROR node already receive data");

        // 真正干活的是这个函数
        ROR_data_publish(msg);
    }

// ---- 核心处理函数：ROR 过滤 + 发布 MarkerArray ----
void ROR_data_publish(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // ===== 算法参数（可按场景调整）=====
    const float R = 0.3f;   // 邻域半径（米）：某个点周围 0.3m 内的点算"邻居"
    const size_t K = 5;     // 最小邻居数：邻居少于 5 个就判定为离群点

    // ===== 第 1 步：把 ranges 转成 (x, y) 点集 =====
    // LaserScan 存的是"每个角度上的距离"，需要转成笛卡尔坐标才能算欧氏距离。
    // 用临时结构体 Pt 同时记录原始索引 idx，方便后面回填/标记。
    struct Pt { float x, y; size_t idx; };
    std::vector<Pt> pts;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float r = msg->ranges[i];

        // 过滤无效测距：inf（无回波/超量程）、nan、小于最小量程、大于最大量程
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;

        // 第 i 个点的角度 = 起始角 + i * 角增量
        float a = msg->angle_min + static_cast<float>(i) * msg->angle_increment;

        // 极坐标 (r, a) → 直角坐标 (x, y)，z 恒为 0（2D 雷达）
        pts.push_back({r * std::cos(a), r * std::sin(a), i});
    }

    // ===== 第 2 步：对每个有效点，统计半径 R 内的邻居数 =====
    // is_outlier 用原始 ranges 的长度，按原始索引标记，避免和 pts 的索引混淆。
    // O(n²) 朴素算法：n≈360 时约 13 万次距离计算，单帧几毫秒，够用。
    std::vector<bool> is_outlier(msg->ranges.size(), false);

    for (size_t i = 0; i < pts.size(); ++i) {
        size_t nb = 0;   // 邻居计数

        for (size_t j = 0; j < pts.size(); ++j) {
            if (i == j) continue;   // 自己不算自己的邻居

            // 用平方距离比较，避免开根号（sqrt 慢，且这里只比大小）
            float dx = pts[i].x - pts[j].x;
            float dy = pts[i].y - pts[j].y;
            if (dx*dx + dy*dy <= R*R) ++nb;
        }

        // 邻居数不足 K → 标记为离群点（用原始索引 idx 回填）
        if (nb < K) is_outlier[pts[i].idx] = true;
    }
            // 统计 ROR 的分类结果
// size_t outlier_count = 0;

// for (const auto &pt : pts) {
//     if (is_outlier[pt.idx]) {
//         ++outlier_count;
//     }
// }

// size_t inlier_count = pts.size() - outlier_count;

// RCLCPP_INFO(
//     this->get_logger(),
//     "ROR result: valid_points=%zu, inliers=%zu, outliers=%zu, R=%.2f, K=%zu",
//     pts.size(),
//     inlier_count,
//     outlier_count,
//     R,
//     K
// );

    // ===== 第 3 步：构造 MarkerArray 并发布 =====
    // 用两个 marker：内点绿色、离群点红色，RViz 里一眼能看出算法删了哪些点。

    visualization_msgs::msg::MarkerArray array;

    // 3.1 先放一个 DELETEALL，清掉上一帧的 marker，否则旧点会残留越积越多
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    // 3.2 定义两个 marker：inliers（内点，绿）和 outliers（离群点，红）
    visualization_msgs::msg::Marker inliers, outliers;

    // 复用雷达消息的 header（frame_id + 时间戳），RViz 靠它做坐标变换
    inliers.header = outliers.header = msg->header;

    // 同一命名空间 "ror_points"，用不同 id 区分两个 marker
    inliers.ns = outliers.ns = "ror_points";
    inliers.id = 0;
    outliers.id = 1;

    // 类型 SPHERE_LIST：每个点画成一个 3D 小球，比 POINTS 更直观
    inliers.type = outliers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    inliers.action = outliers.action = visualization_msgs::msg::Marker::ADD;

    // 球的尺寸 5cm
    inliers.scale.x = inliers.scale.y = inliers.scale.z = 0.05;
    outliers.scale = inliers.scale;

    // 颜色：内点绿、离群点红；alpha 必须设 1，否则完全透明看不见
    inliers.color.r = 0; inliers.color.g = 1; inliers.color.b = 0; inliers.color.a = 1;   // 绿
    outliers.color.r = 1; outliers.color.g = 0; outliers.color.b = 0; outliers.color.a = 1; // 红

    // 3.3 遍历点集，按"是否离群"塞进对应 marker 的 points 数组
    for (const auto &pt : pts) {
        geometry_msgs::msg::Point p;
        p.x = pt.x; p.y = pt.y; p.z = 0.0;

        if (is_outlier[pt.idx]) outliers.points.push_back(p);   // 离群 → 红
        else                    inliers.points.push_back(p);    // 内点 → 绿
    }

    // 3.4 把两个 marker 一起放进 MarkerArray，一次发布
    array.markers.push_back(inliers);
    array.markers.push_back(outliers);
    publisher->publish(array);
}

    // ---- 成员变量 ----
    // 订阅者：接收 /bcr_bot/scan
    rclcpp::Subscription
    <sensor_msgs::msg::LaserScan>::SharedPtr subscription;

    // 发布者：发送 /sensor/ROR（MarkerArray）
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
};

int main(int argc, char **argv)
{
    // 初始化 ROS 2 客户端库
    rclcpp::init(argc, argv);

    // 创建节点并进入事件循环：只要有消息到达就调用对应回调，直到 Ctrl+C
    rclcpp::spin(
        std::make_shared<ROR>()
    );

    // 关闭 ROS 2
    rclcpp::shutdown();

    return 0;
}