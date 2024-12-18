#ifndef TOF_PUB
#define TOF_PUB

#include "publisher.h"
#include <agrobot_interfaces/msg/TOF.msg>



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
   * @param left_dist the distance from left wall (mm)
   * @param right_dist the distance from right wall (mm)
   * @param front_dist the distance from front wall (mm)
   * @param back_dist the distance from back wall (mm)
   */
  void publish(int left_dist, int right_dist, int front_dist, int back_dist);
  using Publisher::destroy;

private:
  agrobot_interfaces__msg__Tof msg;
};

#endif // TOF_PUB