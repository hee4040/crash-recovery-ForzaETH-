// #include <memory>
// #include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "std_msgs/msg/bool.hpp"

// class CollisionDetectorNode : public rclcpp::Node
// {
// public:
//   CollisionDetectorNode()
//   : Node("collision_detector_node")
//   {
//     // 구독자들
//     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//       "/scan", 10,
//       std::bind(&CollisionDetectorNode::scanCallback, this, std::placeholders::_1));

//     imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//       "/imu/data", 10,
//       std::bind(&CollisionDetectorNode::imuCallback, this, std::placeholders::_1));

//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/odom", 10,
//       std::bind(&CollisionDetectorNode::odomCallback, this, std::placeholders::_1));

//     // 충돌 여부 publish
//     collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("/collision_detected", 10);

//     RCLCPP_INFO(this->get_logger(), "CollisionDetector C++ node started");
//   }

// private:
//   void publishCollision(bool detected, const std::string & source)
//   {
//     if (!detected) {
//       return;
//     }
//     std_msgs::msg::Bool msg;
//     msg.data = true;
//     collision_pub_->publish(msg);
//     RCLCPP_WARN(this->get_logger(), "Collision detected from %s!", source.c_str());
//   }

//   void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//   {
//     // 전방 중심부 몇 개 빔만 체크 (예: 중앙 ±5 인덱스)
//     if (msg->ranges.empty()) return;

//     int center = static_cast<int>(msg->ranges.size() / 2);
//     int window = 5;
//     for (int i = -window; i <= window; ++i) {
//       int idx = center + i;
//       if (idx < 0 || idx >= static_cast<int>(msg->ranges.size())) continue;
//       float r = msg->ranges[idx];
//       if (std::isfinite(r) && r > msg->range_min && r < 0.3) {  // 0.3m 이내
//         publishCollision(true, "LiDAR");
//         return;
//       }
//     }
//   }

//   void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
//   {
//     // 매우 단순한 충돌 가속도 기준
//     double accel =
//       std::fabs(msg->linear_acceleration.x) +
//       std::fabs(msg->linear_acceleration.y) +
//       std::fabs(msg->linear_acceleration.z);

//     if (accel > 20.0) {  // 임계값은 나중에 조정
//       publishCollision(true, "IMU");
//     }
//   }

//   void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     double v = std::fabs(msg->twist.twist.linear.x);
//     static double last_v = 0.0;

//     bool hit = (last_v > 1.0 && v < 0.1);  // 속도가 갑자기 멈춘 경우
//     last_v = v;

//     if (hit) {
//       publishCollision(true, "Odom");
//     }
//   }

//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CollisionDetectorNode>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

class CollisionDetectorNode : public rclcpp::Node
{
public:
  CollisionDetectorNode()
  : Node("collision_detector_node")
  {
    // 구독자들
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&CollisionDetectorNode::scanCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&CollisionDetectorNode::imuCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&CollisionDetectorNode::odomCallback, this, std::placeholders::_1));

    // 충돌 여부 publish
    collision_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/collision_detected",
        rclcpp::QoS(1).transient_local()
    );

    RCLCPP_INFO(this->get_logger(), "CollisionDetector C++ node started");
  }

private:

  // 충돌 신호 publish 함수
  void publishCollision(bool detected, const std::string & source)
  {
    if (!detected)
      return;

    std_msgs::msg::Bool msg;
    msg.data = true;
    collision_pub_->publish(msg);

    RCLCPP_WARN(this->get_logger(), "Collision detected from %s!", source.c_str());
  }

  // LiDAR 콜백
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty()) return;

    int center = static_cast<int>(msg->ranges.size() / 2);
    int window = 5;

    for (int i = -window; i <= window; ++i) {
      int idx = center + i;
      if (idx < 0 || idx >= (int)msg->ranges.size()) continue;

      float r = msg->ranges[idx];
      if (std::isfinite(r) && r > msg->range_min && r < 0.3) {
        publishCollision(true, "LiDAR");
        return;
      }
    }
  }

  // IMU 콜백
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double accel =
      std::fabs(msg->linear_acceleration.x) +
      std::fabs(msg->linear_acceleration.y) +
      std::fabs(msg->linear_acceleration.z);

    if (accel > 20.0)
      publishCollision(true, "IMU");
  }

  // Odom 콜백
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double v = std::fabs(msg->twist.twist.linear.x);
    static double last_v = 0.0;

    bool hit = (last_v > 1.0 && v < 0.1);
    last_v = v;

    if (hit)
      publishCollision(true, "Odom");
  }

  // 멤버 변수들
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
};

// main 함수
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

