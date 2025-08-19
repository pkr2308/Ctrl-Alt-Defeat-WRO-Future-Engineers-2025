#pragma once

#include <ICommunication.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/float32_multi_array.h>

class hw_rev_2_ROSCommunication : public ICommunication{

public:
  hw_rev_2_ROSCommunication(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand update(VehicleData data, VehicleCommand cmd) override;  

private:
  VehicleConfig _config;
  ILogger* _logger;
  rcl_node_t _rosNode;
  rclc_executor_t _rosExecutor;
  rcl_publisher_t _rosOrientationPublisher;
  rcl_publisher_t _rosLidarPublisher;
  rcl_subscription_t _rosSteeringSubscriber;
  rcl_subscription_t _rosThrottleSubscriber;  
  rcl_timer_t _rosTimer;
  std_msgs__msg__Int16 _rosThrottleMessage;
  std_msgs__msg__UInt16 _rosSteeringMessage;
  std_msgs__msg__Float32MultiArray _rosLidarMessage;
  std_msgs__msg__Float32MultiArray _rosOrientationMessage;

};