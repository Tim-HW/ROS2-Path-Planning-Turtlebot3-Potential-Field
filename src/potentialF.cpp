#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>

#define PI 3.14159265

using std::placeholders::_1;

class PotentialField : public rclcpp::Node
{
  public:
    PotentialField(char* x_goal, char* y_goal)
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
      // Create Publihser of the Replusion vector
      rep_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("repulsion_vector",1);
      // Create Publihser of the Final vector
      fin_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("Final_Vector",1);

      RCLCPP_INFO(this->get_logger(), "x = %s and y = %s", x_goal,y_goal);

      goal_x = atof(x_goal);
      goal_y = atof(y_goal);


    }

    int controller()
    {
        // Initialize the command message
        geometry_msgs::msg::Twist direction;

        // Parameters
        const double goal_tolerance = 0.1;     // Meters
        const double tolerance = 0.2;         // Radians
        const double rotation_gain = 1.0;     // Gain for angular speed adjustment
        const double max_rotation_speed = 0.5; // Max angular speed (radians/sec)
        const double forward_speed = 0.1;     // Linear speed (meters/sec)

        // Calculate final vector components
        double x_final = V_attraction[0] + V_repulsion[0];
        double y_final = V_attraction[1] + V_repulsion[1];

        // Check if the robot arrived at the destination
        double distance = std::sqrt(x_final * x_final + y_final * y_final);
        if (distance < goal_tolerance) {
            direction.linear.x = 0.0;
            direction.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Target reached!");
            cmd_pub->publish(direction);
            return 0;
        }

        // Publish the final vector (for visualization/debugging)
        geometry_msgs::msg::PoseStamped finalvector = PublishVector(x_final, y_final);
        fin_pub->publish(finalvector);

        // Calculate target angle using atan2 (handles all quadrants correctly)
        double angle = atan2(y_final, x_final);

        // Calculate the shortest angular difference (delta)
        double delta = normalize_angle(angle - theta);

        RCLCPP_INFO(this->get_logger(), "Angle to goal: %f", delta);

        // Control logic
        if (delta < -tolerance) {
            // Rotate clockwise to reduce delta
            direction.angular.z = -std::clamp(delta * rotation_gain, -max_rotation_speed, max_rotation_speed);
            direction.linear.x = 0.0;
        } 
        else if (delta > tolerance) {
            // Rotate counterclockwise to reduce delta
            direction.angular.z = std::clamp(delta * rotation_gain, -max_rotation_speed, max_rotation_speed);
            direction.linear.x = 0.0;
        } 
        else {
            // Move forward when aligned within tolerance
            direction.linear.x = forward_speed;
            direction.angular.z = 0.0;
        }

        // Publish the command
        cmd_pub->publish(direction);

        return 0;
    }

    // Helper function to normalize angle to [-PI, PI]
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
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
      vector.pose.position.x = x_odom ;
      vector.pose.position.y = y_odom ;
      vector.pose.position.z = 0 ;
      // Compute the theta angle
      float angle;
      if (x < 0)
      {
        angle = PI + atan(y/x);
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

    void ComputeAttraction(float x_goal, float y_goal)
    {
        // Log the goal position
        RCLCPP_INFO(this->get_logger(), "GOAL | x: %f | y: %f", x_goal, y_goal);

        // Compute the relative position of the goal from the current position
        float dx = x_goal - x_odom;
        float dy = y_goal - y_odom;

        // Compute the distance between the current position and the goal
        float distance = std::sqrt(dx * dx + dy * dy);

        // Handle edge case: Avoid division by zero or extremely large forces
        if (distance < 1e-3) {
            RCLCPP_WARN(this->get_logger(), "Goal is too close to the current position; force set to zero.");
            V_attraction = {0.0f, 0.0f};
            return;
        }

        // Define the strength of the attraction
        const int Q_attraction = 100;

        // Compute the magnitude of the attractive force
        float F_attraction = Q_attraction / (4 * M_PI * distance * distance);

        // Compute the attraction vector
        V_attraction = {F_attraction * dx / distance, F_attraction * dy / distance};

        // Log the computed attraction force
        RCLCPP_INFO(this->get_logger(), "Attraction Force | x: %f | y: %f", V_attraction[0], V_attraction[1]);

        // Visualize the attraction vector by publishing it
        geometry_msgs::msg::PoseStamped attraction = PublishVector(V_attraction[0], V_attraction[1]);
        att_pub->publish(attraction);
    }

    

  private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract and set the current position from the Odometry message
        x_odom = msg->pose.pose.position.x;
        y_odom = msg->pose.pose.position.y;

        // Retrieve orientation as a quaternion
        const auto& orientation = msg->pose.pose.orientation;
        tf2::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        );

        // Convert quaternion to Euler angles (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Set theta (yaw) as the robot's current heading
        theta = yaw;

        // Log current odometry data (uncomment for debugging)
        // RCLCPP_INFO(this->get_logger(), "Odometry | x: %f | y: %f | theta: %f", x_odom, y_odom, theta);

        // Compute the attraction vector based on the updated odometry
        ComputeAttraction(goal_x, goal_y);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        // Extract scan parameters
        float angle_min = _msg->angle_min;
        float angle_increment = _msg->angle_increment;
        const auto& scan = _msg->ranges;
        size_t num_readings = scan.size();

        // Initialize repulsion vector components
        float x_r = 0.0f;
        float y_r = 0.0f;
        int valid_points = 0;

        // Define constants
        const float MIN_DISTANCE = 0.1f;  // Minimum valid distance
        const float MAX_DISTANCE = 100.0f; // Maximum valid distance
        const int Q_repulsion = 1;

        // Process each laser scan reading
        for (size_t i = 0; i < num_readings; ++i)
        {
            float distance = scan[i];

            // Consider only valid readings within the specified range
            if (distance > MIN_DISTANCE && distance < MAX_DISTANCE)
            {
                float repulsion_force = Q_repulsion / (4 * M_PI * distance * distance);
                float angle = angle_min + theta + angle_increment * i;

                // Calculate projection of the repulsion force onto x and y axes
                x_r -= repulsion_force * cos(angle);
                y_r -= repulsion_force * sin(angle);

                ++valid_points;
            }
        }

        // Handle cases with no valid scan points
        if (valid_points == 0)
        {
            // Assign a negligible repulsion vector to avoid divide-by-zero errors later
            V_repulsion = {0.0001f, 0.0000001f};
        }
        else
        {
            // Update the repulsion vector
            V_repulsion = {x_r, y_r};
        }

        // Log the repulsion vector for debugging (uncomment if needed)
        // RCLCPP_INFO(this->get_logger(), "Repulsion Vector | x: %f, y: %f", V_repulsion[0], V_repulsion[1]);

        // Visualize the repulsion vector in RViz
        geometry_msgs::msg::PoseStamped repulsion = PublishVector(V_repulsion[0], V_repulsion[1]);
        rep_pub->publish(repulsion);

        // Call the controller function to process the updated repulsion vector
        controller();
    }

    // odom subsriber variable declaration
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_odom;
    // scan subsriber variable declaration
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   sub_scan;
    // robot control publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_pub;
    // attraction vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  att_pub;
    // repulsion vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  rep_pub;
    // repulsion vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  fin_pub;
    // Declare position


    // x position of odometry
    double x_odom;
    // y position of odoemtry
    double y_odom;
    // Angle of the robot
    double theta;
    // Attraction vector
    std::vector<float> V_attraction ;
    // Replusion vector
    std::vector<float> V_repulsion ;
    //
    float goal_x = 0;
    float goal_y = 0;

};

int main(int argc, char * argv[])
{  
  // init node
  rclcpp::init(argc, argv);
  // init class
  auto node = std::make_shared<PotentialField>(argv[1],argv[2]);
  rclcpp::spin(node);
  // shutdown once finished
  rclcpp::shutdown();
  // end
  return 0;
}