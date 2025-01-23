#include "tof_pub.h"

void TofPub::setup(rcl_node_t node) {

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, Tof),
      "/tof/distances"));
}// figure out what to do with this part of the code ^

void TofPub::publish(float32 left, float32 right, float32 front, float32 back) {
  // Assign sensor data to the message
  msg.left = left;
  msg.right = right;
  msg.front = front;
  msg.back = back;

  // Add timestamp (same as battery example)
  msg.header.stamp.sec = NS_TO_S(rmw_uros_epoch_nanos());
  msg.header.stamp.nanosec = NS_REMAINDER(rmw_uros_epoch_nanos());

  // Publish the message
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}
