
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <functional>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MotionTestNode : public rclcpp::Node
{
public:
  MotionTestNode()
  : Node("run_test_node")
  {
    // 可通过 ROS 参数调整速度
    forward_speed_ = declare_parameter<double>("forward_speed", 0.08);
    turn_speed_ = declare_parameter<double>("turn_speed", 0.20);
    circle_speed_ = declare_parameter<double>("circle_speed", 0.08);
    circle_radius_ = declare_parameter<double>("circle_radius", 0.50);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&MotionTestNode::odom_callback, this,
                std::placeholders::_1));

    // 使用 wall timer，控制循环不依赖仿真时间是否前进
    timer_ = create_wall_timer(
      50ms, std::bind(&MotionTestNode::control_loop, this));

    last_odom_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(),
      "Motion test node started. Waiting for /odom ...");
  }

  ~MotionTestNode() override
  {
    publish_stop();
  }

private:
  enum class Phase
  {
    WAIT_ODOM,
    FORWARD_1,
    TURN_RIGHT,
    FORWARD_2,
    MOVE_TO_CIRCLE,
    ALIGN_TANGENT,
    CIRCLE,
    ERROR
  };

  static constexpr double PI = 3.14159265358979323846;
  static constexpr double DISTANCE = 0.50;
  static constexpr double TURN_ANGLE = PI / 2.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Phase phase_{Phase::WAIT_ODOM};
  bool have_odom_{false};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  // 起点 A、第一段终点 B、第二段终点 C
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};

  double point_b_x_{0.0};
  double point_b_y_{0.0};

  double center_x_{0.0};
  double center_y_{0.0};
  double circle_start_x_{0.0};
  double circle_start_y_{0.0};

  double target_yaw_{0.0};
  double segment_start_x_{0.0};
  double segment_start_y_{0.0};
  double segment_start_yaw_{0.0};

  double forward_speed_{0.08};
  double turn_speed_{0.20};
  double circle_speed_{0.08};
  double circle_radius_{0.50};

  std::chrono::steady_clock::time_point last_odom_time_;

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double clamp_value(double value, double low, double high)
  {
    return std::max(low, std::min(value, high));
  }

  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;

    // 从四元数计算平面机器人偏航角 yaw
    const double sin_yaw =
      2.0 * (q.w * q.z + q.x * q.y);
    const double cos_yaw =
      1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    yaw_ = std::atan2(sin_yaw, cos_yaw);

    have_odom_ = true;
    last_odom_time_ = std::chrono::steady_clock::now();
  }

  void publish_velocity(double linear, double angular)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  void publish_stop()
  {
    publish_velocity(0.0, 0.0);
  }

  void set_phase(Phase next, const std::string & message)
  {
    phase_ = next;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }

  double distance_to(double target_x, double target_y) const
  {
    return std::hypot(target_x - x_, target_y - y_);
  }

  // 按指定世界坐标方向前进 DISTANCE 米
  bool move_straight(double target_distance, double speed)
  {
    const double dx = x_ - segment_start_x_;
    const double dy = y_ - segment_start_y_;

    // 将位移投影到起步时的前进方向，避免侧向误差影响距离
    const double travelled =
      dx * std::cos(segment_start_yaw_) +
      dy * std::sin(segment_start_yaw_);

    if (travelled >= target_distance) {
      publish_stop();
      return true;
    }

    const double heading_error =
      normalize_angle(segment_start_yaw_ - yaw_);

    const double angular = clamp_value(
      1.5 * heading_error, -0.35, 0.35);

    publish_velocity(speed, angular);
    return false;
  }

  // 原地转向指定角度
  bool rotate_to(double desired_yaw)
  {
    const double error = normalize_angle(desired_yaw - yaw_);

    if (std::abs(error) < 0.025) {
      publish_stop();
      return true;
    }

    const double angular = clamp_value(
      1.5 * error, -turn_speed_, turn_speed_);

    publish_velocity(0.0, angular);
    return false;
  }

  // 朝目标点移动；到达后停止
  bool move_to_point(double target_x, double target_y)
  {
    const double dx = target_x - x_;
    const double dy = target_y - y_;
    const double distance = std::hypot(dx, dy);

    if (distance < 0.025) {
      publish_stop();
      return true;
    }

    const double desired_yaw = std::atan2(dy, dx);
    const double error = normalize_angle(desired_yaw - yaw_);

    // 朝向误差较大时先转向，避免大幅横向绕行
    if (std::abs(error) > 0.45) {
      publish_velocity(
        0.0,
        clamp_value(1.5 * error, -turn_speed_, turn_speed_));
    } else {
      publish_velocity(
        std::min(forward_speed_, 0.5 * distance),
        clamp_value(1.5 * error, -0.25, 0.25));
    }

    return false;
  }

  void control_loop()
  {
    if (!have_odom_) {
      publish_stop();
      return;
    }

    // 用真实墙钟检查里程计新鲜度，避免过期位姿下继续运动
    const auto odom_age =
      std::chrono::steady_clock::now() - last_odom_time_;

    if (odom_age > 1s) {
      publish_stop();

      if (phase_ != Phase::ERROR) {
        phase_ = Phase::ERROR;
        RCLCPP_ERROR(get_logger(),
          "/odom has been stale for over 1 second. Robot stopped.");
      }
      return;
    }

    switch (phase_) {
      case Phase::WAIT_ODOM:
      {
        start_x_ = x_;
        start_y_ = y_;
        start_yaw_ = yaw_;

        segment_start_x_ = x_;
        segment_start_y_ = y_;
        segment_start_yaw_ = yaw_;

        set_phase(Phase::FORWARD_1,
          "Phase 1: moving forward 0.5 m.");
        break;
      }

      case Phase::FORWARD_1:
      {
        if (move_straight(DISTANCE, forward_speed_)) {
          point_b_x_ = x_;
          point_b_y_ = y_;

          target_yaw_ = normalize_angle(start_yaw_ - TURN_ANGLE);

          set_phase(Phase::TURN_RIGHT,
            "Phase 2: turning right 90 degrees.");
        }
        break;
      }

      case Phase::TURN_RIGHT:
      {
        if (rotate_to(target_yaw_)) {
          segment_start_x_ = x_;
          segment_start_y_ = y_;
          segment_start_yaw_ = target_yaw_;

          set_phase(Phase::FORWARD_2,
            "Phase 3: moving forward another 0.5 m.");
        }
        break;
      }

      case Phase::FORWARD_2:
      {
        if (move_straight(DISTANCE, forward_speed_)) {
          // A 与 C 是两段直角路径的端点；
          // 直角三角形的底边 AC 中点作为圆心
          center_x_ = (start_x_ + x_) / 2.0;
          center_y_ = (start_y_ + y_) / 2.0;

          const double rx = x_ - center_x_;
          const double ry = y_ - center_y_;
          const double radius_now = std::hypot(rx, ry);

          if (radius_now < 1e-6) {
            publish_stop();
            phase_ = Phase::ERROR;
            RCLCPP_ERROR(get_logger(),
              "Cannot calculate circle start: radius is too small.");
            break;
          }

          // 从 C 点沿圆心向外的径向，到半径 circle_radius_
          circle_start_x_ =
            center_x_ + circle_radius_ * rx / radius_now;
          circle_start_y_ =
            center_y_ + circle_radius_ * ry / radius_now;

          RCLCPP_INFO(get_logger(),
            "Circle center: (%.3f, %.3f); target radius: %.2f m",
            center_x_, center_y_, circle_radius_);

          set_phase(Phase::MOVE_TO_CIRCLE,
            "Phase 4: moving onto the circle.");
        }
        break;
      }

      case Phase::MOVE_TO_CIRCLE:
      {
        if (move_to_point(circle_start_x_, circle_start_y_)) {
          // 顺时针切线方向 = 径向角度 - 90°
          const double radial_angle = std::atan2(
            circle_start_y_ - center_y_,
            circle_start_x_ - center_x_);

          target_yaw_ = normalize_angle(radial_angle - PI / 2.0);

          set_phase(Phase::ALIGN_TANGENT,
            "Phase 5: aligning with clockwise circle tangent.");
        }
        break;
      }

      case Phase::ALIGN_TANGENT:
      {
        if (rotate_to(target_yaw_)) {
          set_phase(Phase::CIRCLE,
            "Phase 6: circling clockwise continuously. Ctrl+C to stop.");
        }
        break;
      }

      case Phase::CIRCLE:
      {
        const double dx = x_ - center_x_;
        const double dy = y_ - center_y_;
        const double radius_now = std::hypot(dx, dy);

        if (radius_now < 1e-6) {
          publish_stop();
          phase_ = Phase::ERROR;
          RCLCPP_ERROR(get_logger(),
            "Robot is too close to circle center. Stopping.");
          break;
        }

        const double radial_angle = std::atan2(dy, dx);
        const double tangent_yaw =
          normalize_angle(radial_angle - PI / 2.0);

        const double radial_error = radius_now - circle_radius_;

        // 半径偏大时稍微向内修正，偏小时向外修正
        const double desired_yaw = normalize_angle(
          tangent_yaw - 1.2 * radial_error);

        const double heading_error =
          normalize_angle(desired_yaw - yaw_);

        const double angular = clamp_value(
          1.8 * heading_error, -0.40, 0.40);

        publish_velocity(circle_speed_, angular);
        break;
      }

      case Phase::ERROR:
      default:
      {
        publish_stop();
        break;
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionTestNode>());
  rclcpp::shutdown();
  return 0;
}
