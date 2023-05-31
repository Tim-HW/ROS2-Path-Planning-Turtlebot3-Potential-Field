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
      // Create Subscriber to the odometry
      sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&PotentialField::odom_callback, this, _1));
      // Create Subscriber to LiDAR
      sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", default_qos, std::bind(&PotentialField::scan_callback, this, _1));
      // Create Publisher in command control
      cmd_pub  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      // Create Publihser of the attraction vector
      att_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("attraction_vector",10);
      
    }


    void ComputeAttraction(float x_a, float y_a)
    {
      
      // Compute distance between the attraction and the current position
      float distance =  sqrt(pow(x_a - x,2) + pow(y_a - y,2));
      // Compute the point to reach relative to the current position
      x_a = x_a - x;
      y_a = y_a - y;
      // Create the position of the force to simulate
      V_attraction = {x_a,y_a};
      // Create the Module of the force to simulate
      F_attraction = (Q_minus * Q_attraction )/ 4 * 3.14 * pow(distance,2);
      
      
      // Create the attraction vector to show in RVIZ
      geometry_msgs::msg::PoseStamped attraction;
      // Set the frame id
      std::string id_frame = "/odom";
      attraction.header.frame_id = id_frame;
      // Set the time stamp (current time)
      attraction.header.stamp = this->get_clock()->now();
      // set the position (it's always (0,0,0) as the reference frame is odom)
      attraction.pose.position.x = 0 ;
      attraction.pose.position.y = 0 ;
      attraction.pose.position.z = 0 ;
      // Compute the theta angle
      float angle = atan2(x_a,y_a);
      // Init variable of quaterion 
      tf2::Quaternion q;
      // Set the quaterion using euler angles
      q.setRPY(0,0,angle);
      // Convert the geometry::quaternion message into pose::quaterion message
      attraction.pose.orientation = tf2::toMsg(q);
      // Publish it
      att_pub->publish(attraction);

      //RCLCPP_INFO(this->get_logger(), "f_attraction is :%f",F_attraction);
      //RCLCPP_INFO(this->get_logger(), "v_attraction is : x = %f ; y = %f",x,y);


    }

  private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // set current x position
        x     = msg->pose.pose.position.x;
        // set current y position
        y     = msg->pose.pose.position.y;

        // Retrive the rotations using quaterions
        tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
        // Convert it into matrice of 3x3
        tf2::Matrix3x3 m(q);
        // Init angle variables
        double roll, pitch, yaw;
        // Transform quaterion into Euler
        m.getRPY(roll, pitch, yaw);
        // Define theta = yaw
        theta = yaw;

        //RCLCPP_INFO(this->get_logger(), "Odometry : x = %f | y = %f | theta = %f" , x , y, theta);

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
    // odom subsriber variable declaration
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    // scan subsriber variable declaration
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    // robot control publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    // attraction vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  att_pub;
    // Declare position
    double x;
    double y;
    double theta;

    // declare attraction vector
    float F_attraction;
  
    int Q_attraction = 100;
    std::vector<float> V_attraction ;

    int Q_minus = 1 ;
};

int main(int argc, char * argv[])
{
  // init node
  rclcpp::init(argc, argv);
  // init class
  rclcpp::spin(std::make_shared<PotentialField>());
  // shutdown once finished
  rclcpp::shutdown();
  // end
  return 0;
}