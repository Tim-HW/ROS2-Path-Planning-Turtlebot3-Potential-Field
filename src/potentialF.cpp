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

#define PI 3.14159265


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
      cmd_pub  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
      // Create Publihser of the attraction vector
      att_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("attraction_vector",1);
      // Create Publihser of the attraction vector
      rep_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("repulsion_vector",1);
      
    }

    void controller()
    {

      double x_final = V_attraction[0] + V_repulsion[0];
      double y_final = V_attraction[1] + V_repulsion[1];

      geometry_msgs::msg::Twist direction;

      double tolerance = 0.1 ;
      double angle = atan(y_final/x_final);
      theta = atan(y/x);
      if(angle < theta - tolerance)
      {
        direction.angular.z = -0.1;
        direction.linear.x  = 0;
        // v angle +
      }
      else if(angle > theta + tolerance)
      {
        // v angle -
        direction.angular.z = 0.1;
        direction.linear.x  = 0;
      }
      else
      {
        // v forward +
        direction.linear.x  = 0.1;
        direction.angular.z = 0;
      
      }

      cmd_pub->publish(direction);
            
    }

    geometry_msgs::msg::PoseStamped PublishVector(float x, float y)

    {

      // Create the attraction vector to show in RVIZ
      geometry_msgs::msg::PoseStamped vector;
      // Set the frame id
      std::string id_frame = "/odom";
      vector.header.frame_id = id_frame;
      // Set the time stamp (current time)
      vector.header.stamp = this->get_clock()->now();
      // set the position (it's always (0,0,0) as the reference frame is odom)
      vector.pose.position.x = 0 ;
      vector.pose.position.y = 0 ;
      vector.pose.position.z = 0 ;
      // Compute the theta angle
      float angle;
      if (x < 0)
      {
        angle = atan(y/x) + PI;
      }
      else
      {
        angle = atan(y/x);
      }
      
      // Init variable of quaterion 
      tf2::Quaternion q;
      // Set the quaterion using euler angles
      q.setRPY(0,0,angle);
      // Convert the geometry::quaternion message into pose::quaterion message
      vector.pose.orientation = tf2::toMsg(q);
      // Publish it

      return vector;

    }

    void ComputeAttraction(float x_a, float y_a, float x_odom, float y_odom)
    {
      
      // Compute distance between the attraction and the current position
      float distance =  sqrt(pow(x_a - x_odom,2) + pow(y_a - y_odom,2));
      // Compute the point to reach relative to the current position
      x_a = x_a - x_odom;
      y_a = y_a - y_odom;
            // Create the Module of the force to simulate
      F_attraction = (Q_repulsion * Q_attraction )/(4 * PI * pow(distance,2));
      // Create the position of the force to simulate
      V_attraction = {F_attraction * x_a , F_attraction * y_a};

      
      //RCLCPP_INFO(this->get_logger(), "x : %f | y : %f",x_a,y_a);
      //RCLCPP_INFO(this->get_logger(), "Force: %f",F_attraction);
      

      //RCLCPP_INFO(this->get_logger(), "angle attraction :%f°",atan(V_attraction[1]/V_attraction[0])*180/PI);
      //RCLCPP_INFO(this->get_logger(), "v_attraction is : x = %f ; y = %f",x,y);

      geometry_msgs::msg::PoseStamped attraction = PublishVector(V_attraction[0],V_attraction[1]);
      att_pub->publish(attraction);

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

        ComputeAttraction(3,0,x,y);

        //controller();
    }
    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {

      float angle_min = _msg->angle_min;
      //float angle_max = _msg->angle_max;
      float step      = _msg->angle_increment; 
      auto scan       = _msg->ranges;
      auto len        = int(float(scan.size()));

      int counter = 0;


      float x_r = 0;
      float y_r = 0;

      for(int i = 0 ; i < len ; i++)
      {
        // If the value of the scan is < 100m it's not tacking into account
        if(scan[i] < 100 and scan[i] > 0.1)
        {
          //RCLCPP_INFO(this->get_logger(), "Scan n: %d | value: %f",i,scan[i]);
          float Current_Q = (Q_attraction * Q_repulsion) / (4 * PI * pow(scan[i],2));
          // Projection of the vectors in the x , y coordinates
          x_r -= Current_Q * cos(angle_min+step*i);
          y_r -= Current_Q * sin(angle_min+step*i);
          
        }
        else
        {
          counter += 1;
        }
        
      }
      //RCLCPP_INFO(this->get_logger(), "Counter : %d  ",counter);
      if(counter == 360)
      {
        V_repulsion = {0.0001,0.000000000001};
      }
      else
      {
        
        //RCLCPP_INFO(this->get_logger(), "x: %f | y: %f",x_r,y_r);
      

        V_repulsion = {x_r, y_r};

        
      }

      RCLCPP_INFO(this->get_logger(), "\n angle repulsion : %f  °",atan(V_repulsion[1]/V_repulsion[0])*180/PI);


      // Create the vector for Rviz
      geometry_msgs::msg::PoseStamped repulsion = PublishVector(V_repulsion[0],V_repulsion[1]);
      // Publish the vector
      rep_pub->publish(repulsion);


    }
    // odom subsriber variable declaration
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom;
    // scan subsriber variable declaration
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  sub_scan;
    // robot control publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_pub;
    // attraction vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  att_pub;
    // repulsion vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  rep_pub;
    // Declare position

    double x;
    double y;
    double theta;

    // declare attraction vector
    float F_attraction;
  
    int Q_attraction = 50;
    std::vector<float> V_attraction ;

    int Q_repulsion = 1;
    std::vector<float> V_repulsion ;
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