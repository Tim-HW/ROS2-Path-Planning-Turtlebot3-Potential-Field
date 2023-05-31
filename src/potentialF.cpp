#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>


using std::placeholders::_1;

class PotentialField : public rclcpp::Node
{
  public:
    PotentialField()
    : Node("potential_field_node")

    {
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      
      sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&PotentialField::odom_callback, this, _1));
      sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", default_qos, std::bind(&PotentialField::scan_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            
    }

  private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
    }
    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {

      RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],_msg->ranges[100]);

    //  float angle_min = msg->angle_min;
    //  float angle_max = msg->angle_max;
    //  auto scan = msg->ranges;

      //RCLCPP_INFO(this->get_logger(), "Scan : min = %f | max = %f",angle_min,angle_max);
    //  RCLCPP_INFO(this->get_logger(), "Scan : %f",scan);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    double x;
    double y;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialField>());
  rclcpp::shutdown();

  

  return 0;
}