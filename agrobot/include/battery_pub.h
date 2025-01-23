#ifndef BATTERY_PUB
#define BATTERY_PUB

#include "publisher.h"
#include <Wire.h>
#include <agrobot_interfaces/msg/battery_status.h>

/**
 * @author Nelson Durrant
 * @date September 2024
 *
 * Publisher for battery sensor data.
 */
class BatteryPub : Publisher {

public:
  /**
   * This function sets up the battery publisher.
   *
   * @param node the micro-ROS node
   */
  void setup(rcl_node_t node);

  /**
   * This function publishes the battery data to the micro-ROS agent.
   *
   * @param voltage the battery voltage (V)
   * @param current the battery current (A)
   */
  void publish(float voltage, float current);
  using Publisher::destroy;

private:
  agrobot_interfaces__msg__BatteryStatus msg;
};

#endif // BATTERY_PUB 