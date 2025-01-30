#ifndef TOF_PUB
#define TOF_PUB

#include "publisher.h"
#include <Wire.h>
#include <agrobot_interfaces/msg/to_f_data.h> //fix this


/**
 * @author Brighton Anderson
 * @date January 2025
 *
 * Publisher for battery sensor data.
 */
class TofPub : Publisher {

public:
  /**
   * This function sets up the tof publisher.
   *
   * @param node the micro-ROS node
   */
  void setup(rcl_node_t node);

  /**
   * This function publishes the tof data to the micro-ROS agent.
   *
   * @param left the distance from left wall (mm)
   * @param right the distance from right wall (mm)
   * @param front the distance from front wall (mm)
   * @param back the distance from back wall (mm)
   */
  void publish(float32 left, float32 right, float32 front, float32 back);
  using Publisher::destroy;

private:
  agrobot_interfaces__msg__Tof msg;
};

#endif // TOF_PUB
