/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"


namespace microstrain {

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("listener")
  {
    // Setup the subscription (lambda for simplicity, but this could also be a member function)
    sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
      [this](sensor_msgs::msg::Imu::UniquePtr imu)
      {
        RCLCPP_INFO(get_logger(), "Quaternion Orientation:    [%f, %f, %f, %f]", imu->orientation.x, imu->orientation.y, imu->orientation.z,
                imu->orientation.w);
        RCLCPP_INFO(get_logger(), "Angular Velocity:          [%f, %f, %f]", imu->angular_velocity.x, imu->angular_velocity.y,
                imu->angular_velocity.z);
        RCLCPP_INFO(get_logger(), "Linear Acceleration:       [%f, %f, %f]", imu->linear_acceleration.x, imu->linear_acceleration.y,
                imu->linear_acceleration.z);

        // add code here to handle incoming IMU data
      }
    );
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

};  // namespace microstrain

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<microstrain::Listener>());
  rclcpp::shutdown();
  return 0;
}
