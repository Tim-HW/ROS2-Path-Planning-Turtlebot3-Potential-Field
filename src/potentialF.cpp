#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <iostream>
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
      cmd_pub  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      att_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("attraction_vector",10);
            
    }


    void ComputeAttraction(float x_a, float y_a)
    {
      
      float distance =  sqrt(pow(x_a - x,2) + pow(y_a - y,2));
      x_a = x_a - x;
      y_a = y_a - y;

      F_attraction = (Q_minus * Q_attraction )/ 4 * 3.14 * pow(distance,2);
      V_attraction = {x_a,y_a};

      geometry_msgs::msg::PoseStamped attraction;

      std::string id_frame = "/odom";
      attraction.header.frame_id = id_frame;
      attraction.header.stamp = this->get_clock()->now();

      attraction.pose.position.x = 0 ;
      attraction.pose.position.y = 0 ;
      attraction.pose.position.z = 0 ;

      float angle = atan2(x_a,y_a);

      tf2::Quaternion q;
      q.setRPY(0,0,angle);

      attraction.pose.orientation = tf2::toMsg(q);

      att_pub->publish(attraction);

      RCLCPP_INFO(this->get_logger(), "f_attraction is :%f",F_attraction);
      RCLCPP_INFO(this->get_logger(), "v_attraction is : x = %f ; y = %f",x,y);


    }

  private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x     = msg->pose.pose.position.x;
        y     = msg->pose.pose.position.y;

        tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        theta = yaw;

        RCLCPP_INFO(this->get_logger(), "Odometry : x = %f | y = %f | theta = %f" , x , y, theta);

        ComputeAttraction(1,1);
    }
    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {

      float angle_min = _msg->angle_min;
      float angle_max = _msg->angle_max;
      float step      = _msg->angle_increment; 
      auto scan       = _msg->ranges;
      auto len        = int(float(scan.size()));

    //RCLCPP_INFO(this->get_logger(), "scan is %d long",sizeof(scan));

    //for(int i = 0; i < len;i++)
    //{
    //  RCLCPP_INFO(this->get_logger(), "scan : nbr = %d |  value= %f", i , scan[i]);
    //}

      

    //  float angle_min = msg->angle_min;
    //  float angle_max = msg->angle_max;
    //  auto scan = msg->ranges;

      //RCLCPP_INFO(this->get_logger(), "Scan : min = %f | max = %f",angle_min,angle_max);
    //  RCLCPP_INFO(this->get_logger(), "Scan : %f",scan);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  att_pub;
    
    double x;
    double y;
    double theta;

    float F_attraction;
    int Q_attraction = 100;
    std::vector<float> V_attraction ;

    int Q_minus = 1 ;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialField>());
  rclcpp::shutdown();

  return 0;
}