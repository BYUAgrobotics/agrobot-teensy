#ifndef PUBLISHER
#define PUBLISHER

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define NS_S 1000000000
#define NS_TO_S(ns) (ns / NS_S)
#define NS_REMAINDER(ns) (ns % NS_S)

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }

#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

/**
 * @author Nelson Durrant
 * @date September 2024
 * @brief Base class for publishers
 */
class Publisher {

public:
  /**
   * @brief This function sets up the publisher. It must be implemented by the derived
   * class.
   * 
   * @param node the micro-ROS node
   */
  virtual void setup(rcl_node_t node) = 0;

  /**
   * @brief This function destroys the publisher.
   * 
   * @param node the micro-ROS node
   */
  void destroy(rcl_node_t node) {
    RCCHECK(rcl_publisher_fini(&publisher, &node));
  }

protected:
  rcl_publisher_t publisher;

  void error_loop() {
    while (1) {
      delay(100);
    }
  }
};

#endif // PUBLISHER