
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class MapToLaserTF : public rclcpp::Node
{
public:
    MapToLaserTF()
        : Node("map_to_laser")
    {
        broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "my_laser_frame";

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.2;

        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        broadcaster_->sendTransform(transform);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapToLaserTF>());
    rclcpp::shutdown();
    return 0;
}
