/*
#include <hwrev2_micro_ros_communication.hpp>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

static hw_rev_2_ROSCommunication *_classptr = NULL;

hw_rev_2_ROSCommunication::hw_rev_2_ROSCommunication(VehicleConfig cfg){

  _config = cfg;

}

void hw_rev_2_ROSCommunication::init(ILogger *logger){

  _classptr = this;
  _logger = logger;

  _rosAllocator = rcl_get_default_allocator();
  rclc_support_init(&_rosSupport, 0, NULL, &_rosAllocator);
  rclc_node_init_default(&_rosNode, _rosNodeName, _rosNodeNamespace, &_rosSupport);

  rclc_publisher_init_default(
    &_rosOrientationPublisher,
    &_rosNode,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    _rosOrientationTopicName
  );

  rclc_publisher_init_default(
    &_rosLidarPublisher,
    &_rosNode,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    _rosLidarTopicName
  );

  rclc_subscription_init_default(
    &_rosSteeringSubscriber,
    &_rosNode,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    _rosSteeringTopicName
  );  

  rclc_subscription_init_default(
    &_rosThrottleSubscriber,
    &_rosNode,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    _rosThrottleTopicName
  );  

  rclc_timer_init_default(
    &_rosTimer,
    &_rosSupport,
    RCL_MS_TO_NS(100),
    _rosTimerCallback

  );

  _rosExecutor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&_rosExecutor, &_rosSupport.context, 5, &_rosAllocator);
  rclc_executor_add_subscription_with_context(&_rosExecutor, &_rosSteeringSubscriber, &_rosSteeringMessage, _rosSteeringCallback, reinterpret_cast<void*>(this), ON_NEW_DATA);
  rclc_executor_add_subscription_with_context(&_rosExecutor, &_rosThrottleSubscriber, &_rosThrottleMessage, _rosThrottleCallback, reinterpret_cast<void*>(this), ON_NEW_DATA);
  rclc_executor_add_timer(&_rosExecutor, &_rosTimer);
  
  _logger->sendMessage("hw_rev_2_ROSCommunication::init", _logger->INFO, "ROS communication driver initialised");

}

VehicleCommand hw_rev_2_ROSCommunication::update(VehicleData data, VehicleCommand cmd){

  _data = data;
  rclc_executor_spin_some(&_rosExecutor, RCL_MS_TO_NS(100));
  return _command;
  
}

void hw_rev_2_ROSCommunication::_rosTimerCallback(rcl_timer_t *timer, int64_t lastCallTime){

  _classptr->publishData();

}

void hw_rev_2_ROSCommunication::_rosThrottleCallback(const void *msgin, void *context){

  const std_msgs__msg__Int16 *received_msg = (const std_msgs__msg__Int16 *)msgin;  
  hw_rev_2_ROSCommunication* contextClass = reinterpret_cast<hw_rev_2_ROSCommunication*>(context);
  contextClass->_command.targetSpeed = received_msg->data;

}

void hw_rev_2_ROSCommunication::publishData(){

  _logger->sendMessage("hw_rev_2_ROSCommunication::publishData", _logger->INFO, "publishData called");

}
  */