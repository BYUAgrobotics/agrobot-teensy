/**
 * @file main.cpp
 * @author Nelson Durrant
 * @date September 2024
 *
 * This node is the micro-ROS node for the agrobot. It controls the actuators
 * and LEDs and reads the sensor data (TOF sensors, etc). The node communicates
 * with the Raspberry Pi over micro-ROS.
 *
 * Subscribes:
 * - TODO: Add other subscribers
 * - LED/command (agrobot_interfaces/msg/led_command.h)
 *
 * Publishes:
 * - battery_status (agrobot_interfaces/msg/BatteryStatus)
 * - TODO: Add other publishers
 *
 *
 * IMPORTANT! For an example of a larger micro-ROS project that follows this
 * approach, see: https://github.com/BYU-FRoSt-Lab/cougars-teensy.git
 */

#include "battery_pub.h"

#include <SoftwareSerial.h>
#include <agrobot_interfaces/msg/led_command.h>
// #include <frost_interfaces/msg/u_command.h>

#define ENABLE_ACTUATORS
#define ENABLE_TOF_SENSORS
#define ENABLE_LEDS
#define ENABLE_BATTERY
// #define ENABLE_BT_DEBUG

#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

// micro-ROS config values
#define BAUD_RATE 6000000
#define CALLBACK_TOTAL 1
#define SYNC_TIMEOUT 1000

// hardware pin values
#define BT_MC_RX 34
#define BT_MC_TX 35
#define VOLT_PIN 18
#define CURRENT_PIN 17
#define LED_PIN 13 // Built-in Teensy LED
#define LARGE_LED 30  // WHITE
#define SMALL_LED 31  // GREEN
#define BAD_LED 32    // RED

// sensor baud rates
#define BT_DEBUG_RATE 9600

// sensor update rates
#define BATTERY_MS 1000 // arbitrary

// time of last received command (used as a fail safe)
unsigned long last_received = 0;

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// message objects
agrobot_interfaces__msg__LEDCommand led_command_msg;

// subscriber objects
rcl_subscription_t led_command_sub;

// publisher objects
BatteryPub battery_pub;

// sensor objects
SoftwareSerial BTSerial(BT_MC_RX, BT_MC_TX);

// states for state machine in loop function
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} static state;

void error_loop() {
  while (1) {
    delay(100);

#ifdef ENABLE_BT_DEBUG
    BTSerial.println("[ERROR] In error loop");
#endif // ENABLE_BT_DEBUG
  }
}

/**
 * Callback function for the "kinematics/command" subscriber. This function
 is
 * called whenever a new control command is received from the micro-ROS
 agent.
 * The function updates the actuator positions based on the received command.
 *
 * @param command_msgin The received frost_interfaces/msg/UCommand message
 */

 void led_command_callback(const void * command_msgin) {
    const agrobot_interfaces__msg__LEDCommand * led_command_msg = (const agrobot_interfaces__msg__LEDCommand *)command_msgin;
    int8_t led_value = led_command_msg->command;
    Serial.printf("Received LED command: %d\n", led_value);

     // First, turn off all LEDs
    digitalWrite(LARGE_LED, HIGH);
    digitalWrite(SMALL_LED, HIGH);
    digitalWrite(BAD_LED, HIGH);

   // Turn on the LED corresponding to the command
    if (led_value == 1) {
        // Small LED -- GREEN
      digitalWrite(SMALL_LED, HIGH);
    } else if (led_value == 2) {
        // Large LED -- WHITE
      digitalWrite(LARGE_LED, HIGH);
    } else if (led_value == 3) {
        // BAD LED -- RED
      digitalWrite(BAD_LED, HIGH);
    }
    
    // Add code to control the LED based on the received command
}
// void command_sub_callback(const void *command_msgin) {

//   last_received = millis();

//   const frost_interfaces__msg__UCommand *command_msg =
//       (const frost_interfaces__msg__UCommand *)command_msgin;

// #ifdef ENABLE_SERVOS
//   myServo1.write(command_msg->fin[0] + DEFAULT_SERVO); // top fin
//   myServo2.write(command_msg->fin[1] + DEFAULT_SERVO); // right fin, from
//   front myServo3.write(command_msg->fin[2] + DEFAULT_SERVO); // left fin,
//   from front
// #endif // ENABLE_SERVOS

// #ifdef ENABLE_THRUSTER
//   int converted = map(command_msg->thruster, THRUSTER_IN_LOW, THRUSTER_IN_HIGH, THRUSTER_OUT_LOW, THRUSTER_OUT_HIGH);
//   myThruster.writeMicroseconds(converted);
// #endif // ENABLE_THRUSTER

// #ifdef ENABLE_BT_DEBUG
//   BTSerial.println("[INFO] Command Received");
// #endif // ENABLE_BT_DEBUG
// }

/**
 * Creates micro-ROS entities. This function initializes the micro-ROS
 * entities (node, publishers, subscribers, and executor) and synchronizes the
 * timestamps with the Raspberry Pi.
 *
 * @return true if the entities were created successfully, false otherwise
 */
bool create_entities() {

  // the allocator object wraps the dynamic memory allocation and deallocation
  // methods used in micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // synchronize timestamps with the Raspberry Pi
  // after sync, timing should be able to be accessed with "rmw_uros_epoch"
  // functions
  RCCHECK(rmw_uros_sync_session(SYNC_TIMEOUT));

#ifdef ENABLE_BT_DEBUG
  if (!rmw_uros_epoch_synchronized()) {
    BTSerial.println("[ERROR] Could not synchronize timestamps with agent");
  } else {
    BTSerial.println("[INFO] Timestamps synchronized with agent");
  }
#endif // ENABLE_BT_DEBUG

  // create publishers
  battery_pub.setup(node);

  // create subscribers
    RCCHECK(rclc_subscription_init_default(
        &led_command_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, LEDCommand),
        "/led/command"));

  // create executor
    RCSOFTCHECK(rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL, &allocator));

  // add callbacks to executor
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &led_command_sub, &led_command_msg, &led_command_callback, ON_NEW_DATA));

#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Micro-ROS entities created successfully");
#endif // ENABLE_BT_DEBUG

  return true;
}

/**
 * Destroys micro-ROS entities. This function destroys the micro-ROS
 * entities (node, publishers, subscribers, and executor).
 */
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // destroy publishers
  battery_pub.destroy(node);

  // destroy everything else
  //   if (rcl_subscription_fini(&led_command_sub, &node) != RCL_RET_OK) {
  // #ifdef ENABLE_BT_DEBUG
  //     BTSerial.println("[WARN] Failed to destroy led_command_sub");
  // #endif // ENABLE_BT_DEBUG
  //   }
  //   rclc_executor_fini(&executor);
  if (rcl_node_fini(&node) != RCL_RET_OK) {
#ifdef ENABLE_BT_DEBUG
    BTSerial.println("[WARN] Failed to destroy node");
#endif // ENABLE_BT_DEBUG
  }
  rclc_support_fini(&support);

#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Micro-ROS entities destroyed successfully");
#endif // ENABLE_BT_DEBUG
}

/**
 * Sets up the micro-ROS serial transports. This function sets up the
 * micro-ROS serial transports for communication with the Raspberry Pi.
 */
void setup() {

  Serial.begin(BAUD_RATE);
  set_microros_serial_transports(Serial);

  // set up the indicator light
  pinMode(LED_PIN, OUTPUT);
  pinMode(LARGE_LED, OUTPUT);
  pinMode(SMALL_LED, OUTPUT);
  pinMode(BAD_LED, OUTPUT);
  // Turn all LEDs off by default
  digitalWrite(LARGE_LED, HIGH);
  digitalWrite(SMALL_LED, HIGH);
  digitalWrite(BAD_LED, HIGH);

#ifdef ENABLE_BT_DEBUG
  BTSerial.begin(BT_DEBUG_RATE);
#endif // ENABLE_BT DEBUG

#ifdef ENABLE_ACTUATORS
  // TODO: Add actuator setup code here
#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Actuators enabled");
#endif // ENABLE_BT_DEBUG
#endif // ENABLE_ACTUATORS

#ifdef ENABLE_BATTERY
  pinMode(CURRENT_PIN, INPUT);
  pinMode(VOLT_PIN, INPUT);

#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Battery Sensor enabled");
#endif // ENABLE_BT_DEBUG
#endif // ENABLE_BATTERY

  state = WAITING_AGENT;
}

/**
 * Reads the battery sensor data. This function reads the battery sensor
 * data (voltage and current) and publishes it to the micro-ROS agent.
 */
void read_battery() {

  // we did some testing to determine the below params, but
  // it's possible they are not completely accurate
  float voltage = (analogRead(VOLT_PIN) * 0.03437) + 0.68;
  float current = (analogRead(CURRENT_PIN) * 0.122) - 11.95;

  // publish the battery data
  battery_pub.publish(voltage, current);
}

/**
 * This function is the main loop for the micro-ROS node. It manages the
 * connection and disconnection of the micro-ROS agent, actuator positions,
 * and sensor data collection.
 */
void loop() {

  // blink the indicator light
  if (millis() % 1000 < 250) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LARGE_LED, LOW);
    digitalWrite(SMALL_LED, LOW);
    digitalWrite(BAD_LED, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  // fail safe for agent disconnect
  if (millis() - last_received > 5000) {

#ifdef ENABLE_ACTUATORS
    // TODO: Add actuator stop code here
#endif // ENABLE_ACTUATORS

#ifdef ENABLE_BT_DEBUG
    BTSerial.println("[INFO] No command received in timeout, stopping actuators");
#endif // ENABLE_BT_DEBUG
  }

  // state machine to manage connecting and disconnecting the micro-ROS agent
  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    };
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED) {

      //////////////////////////////////////////////////////////
      // EXECUTES WHEN THE AGENT IS CONNECTED
      //////////////////////////////////////////////////////////

#ifdef ENABLE_BATTERY
      EXECUTE_EVERY_N_MS(BATTERY_MS, read_battery());
#endif // ENABLE_BATTERY

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      //////////////////////////////////////////////////////////
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }
}
