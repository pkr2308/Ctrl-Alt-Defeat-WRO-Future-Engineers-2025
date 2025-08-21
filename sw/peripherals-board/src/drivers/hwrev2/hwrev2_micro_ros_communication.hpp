/**
 * @author DIY Labs
 * @note Commented out due to linker issues with Micro ROS.
*/

/*
#pragma once

#include <ICommunication.hpp>
#include <vehiclecommand.hpp>
#include <vehicledata.hpp>
#include <config.hpp>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

class hw_rev_2_ROSCommunication : public ICommunication{

public:
  hw_rev_2_ROSCommunication(VehicleConfig cfg);
  void init(ILogger *logger) override;
  VehicleCommand update(VehicleData data, VehicleCommand cmd) override;  
  void publishData();

private:
  VehicleConfig _config;
  VehicleCommand _command;
  VehicleData _data;
  ILogger* _logger;
  rclc_support_t _rosSupport;
  rcl_allocator_t _rosAllocator;
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

  const char *_rosNodeName = "RP2040";
  const char *_rosNodeNamespace = "";
  const char * const _rosOrientationTopicName = "/robot/periph_ori";
  const char * const _rosLidarTopicName = "/robot/periph_lidar";
  const char * const _rosSteeringTopicName = "/robot/periph_steering_cmd";
  const char * const _rosThrottleTopicName = "/robot/periph_throttle_cmd";

  static void _rosTimerCallback(rcl_timer_t *timer, int64_t lastCallTime);
  static void _rosSteeringCallback(const void *msgin, void *context);
  static void _rosThrottleCallback(const void *msgin, void *context);

};
*/