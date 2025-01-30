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
#include "tof_pub.h" //TODO: make sure this works
#include "DFRobot_TMF8x01.h"
#include <SoftwareSerial.h>
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

// sensor baud rates
#define BT_DEBUG_RATE 9600

// sensor update rates
#define BATTERY_MS 1000 // arbitrary
#define TOF_MS 500     // arbitrary

// time of last received command (used as a fail safe)
unsigned long last_received = 0;

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rclc_executor_t executor;

// message objects
// frost_interfaces__msg__UCommand command_msg;

// subscriber objects
// rcl_subscription_t command_sub;

// publisher objects
BatteryPub battery_pub;

// TOF publisher object
TofPub tof_pub;

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

// /**
//  * Callback function for the "kinematics/command" subscriber. This function
//  is
//  * called whenever a new control command is received from the micro-ROS
//  agent.
//  * The function updates the actuator positions based on the received command.
//  *
//  * @param command_msgin The received frost_interfaces/msg/UCommand message
//  */
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
  //   RCCHECK(rclc_subscription_init_default(
  //       &command_sub, &node,
  //       ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, UCommand),
  //       NAMESPACE "/kinematics/command"));

  // create executor
  //   RCSOFTCHECK(rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL, &allocator));

  // add callbacks to executor
  //   RCSOFTCHECK(rclc_executor_add_subscription(&executor, &command_sub, &command_msg, &command_sub_callback, ON_NEW_DATA));

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
  //   if (rcl_subscription_fini(&command_sub, &node) != RCL_RET_OK) {
  // #ifdef ENABLE_BT_DEBUG
  //     BTSerial.println("[WARN] Failed to destroy command_sub");
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

#ifdef ENABLE_TOF_SENSORS
#define EN1       2                      // EN pin for left TMF8801
#define EN2       3                      // EN pin for right TMF8801
#define EN3       4                      // EN pin for front TMF8801
#define EN4       5                      // EN pin for back TMF8801
#define INT      -1                      // INT pin is floating, not used in this demo

// Create two sensor objects, one for each TMF8801
DFRobot_TMF8801 tofLeft(/*enPin =*/EN1, /*intPin=*/INT);
DFRobot_TMF8801 tofRight(/*enPin =*/EN2, /*intPin=*/INT);
DFRobot_TMF8801 tofFront(/*enPin =*/EN3, /*intPin=*/INT);
DFRobot_TMF8801 tofBack(/*enPin =*/EN4, /*intPin=*/INT);

// Calibration data
uint8_t caliDataBuf[14] = {0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};

// Serial.begin(115200);
// while(!Serial) {
//   // Wait for serial port to connect. Needed for native USB port only
// }

pinMode(EN1, OUTPUT);
pinMode(EN2, OUTPUT);
pinMode(EN3, OUTPUT);
pinMode(EN4, OUTPUT);

// Initially set all EN pins LOW
digitalWrite(EN1, LOW);
digitalWrite(EN2, LOW);
digitalWrite(EN3, LOW);
digitalWrite(EN4, LOW);
delay(50); // Give some time for sensors to reset

// Initialize first sensor (left)
digitalWrite(EN1, HIGH);  // Enable left sensor
delay(50);                // Wait for the sensor to power up

BTSerial.print("Initializing left TMF8801 sensor...");
while(tofLeft.begin() != 0) {
  BTSerial.println("failed.");
  delay(1000);
}
BTSerial.println("done.");

BTSerial.println(tofLeft.getUniqueID(), HEX); // check unique id before resetting address

// Set I2C address for first sensor
bool addr1 = tofLeft.setI2CAddress(1);  // Set to address 1
if (addr1){
  BTSerial.println("failed to set address 1");
  delay(10);
}
else{BTSerial.println("successfully set address 1!");}

// Print first sensor info
BTSerial.println("Left Sensor:");
BTSerial.print("Software Version: ");
BTSerial.println(tofLeft.getSoftwareVersion());
BTSerial.print("Unique ID: ");
BTSerial.println(tofLeft.getUniqueID(), HEX);
BTSerial.print("Model: ");
BTSerial.println(tofLeft.getSensorModel());

// Initialize second sensor (right)
digitalWrite(EN2, HIGH);  // Enable right sensor
delay(50);                // Wait for the sensor to power up

BTSerial.print("Initializing right TMF8801 sensor...");
while(tofRight.begin() != 0) {
  BTSerial.println("failed.");
  delay(1000);
}
BTSerial.println("done.");

// Set I2C address for second sensor
tofRight.setI2CAddress(2);  // Set to address 2

// Print second sensor info
BTSerial.println("Right Sensor:");
BTSerial.print("Software Version: ");
BTSerial.println(tofRight.getSoftwareVersion());
BTSerial.print("Unique ID: ");
BTSerial.println(tofRight.getUniqueID(), HEX);
BTSerial.print("Model: ");
BTSerial.println(tofRight.getSensorModel());

// Initialize third sensor (front)
digitalWrite(EN3, HIGH);  // Enable front sensor
delay(50);                // Wait for the sensor to power up

BTSerial.print("Initializing front TMF8801 sensor...");
while(tofFront.begin() != 0) {
  BTSerial.println("failed.");
  delay(1000);
}
BTSerial.println("done.");

// Set I2C address for third sensor
tofFront.setI2CAddress(3);  // Set to address 3

// Print second sensor info
BTSerial.println("Front Sensor:");
BTSerial.print("Software Version: ");
BTSerial.println(tofFront.getSoftwareVersion());
BTSerial.print("Unique ID: ");
BTSerial.println(tofFront.getUniqueID(), HEX);
BTSerial.print("Model: ");
BTSerial.println(tofFront.getSensorModel());

// Initialize fourth sensor (back)
digitalWrite(EN4, HIGH);  // Enable back sensor
delay(50);                // Wait for the sensor to power up

BTSerial.print("Initializing back TMF8801 sensor...");
while(tofBack.begin() != 0) {
  BTSerial.println("failed.");
  delay(1000);
}
BTSerial.println("done.");

// Set I2C address for fourth sensor
tofBack.setI2CAddress(4);  // Set to address 3

// Print Fourth sensor info
BTSerial.println("Back Sensor:");
BTSerial.print("Software Version: ");
BTSerial.println(tofBack.getSoftwareVersion());
BTSerial.print("Unique ID: ");
BTSerial.println(tofBack.getUniqueID(), HEX);
BTSerial.print("Model: ");
BTSerial.println(tofBack.getSensorModel());

// Set calibration data for both sensors
tofLeft.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));
tofRight.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));
tofFront.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));
tofBack.setCalibrationData(caliDataBuf, sizeof(caliDataBuf));

// Start measurements on both sensors
tofLeft.startMeasurement(/*cailbMode =*/tofLeft.eModeCalib);
tofRight.startMeasurement(/*cailbMode =*/tofRight.eModeCalib);
tofFront.startMeasurement(/*cailbMode =*/tofFront.eModeCalib);
tofBack.startMeasurement(/*cailbMode =*/tofBack.eModeCalib);


#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Battery Sensor enabled");
#endif // ENABLE_BT_DEBUG
#endif // ENABLE_BATTERY
#endif // ENABLE_TOF_SENSORS
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

void read_tof_sensor() {
  if (tofLeft.isDataReady()) {
    float32 left_distance = tofLeft.getDistance_mm();
  }

  if (tofRight.isDataReady()) {
    float32 right_distance = tofRight.getDistance_mm();
  }

  if (tofFront.isDataReady()) {
    float32 front_distance = tofFront.getDistance_mm();
  }

  if (tofBack.isDataReady()) {
    float32 back_distance = tofBack.getDistance_mm();
  }

  // publish the TOF sensor data [ADD back_distance WHEN able to power all 4 sensors]
  tof_pub.publish(left_distance, right_distance, front_distance, back_distance);
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

//loop that runs when microros agent is connected
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED) {
      
      //////////////////////////////////////////////////////////
      // EXECUTES WHEN THE AGENT IS CONNECTED
      //////////////////////////////////////////////////////////

#ifdef ENABLE_BATTERY
      EXECUTE_EVERY_N_MS(BATTERY_MS, read_battery());
#endif // ENABLE_BATTERY

#ifdef ENABLE_TOF_SENSORS
      EXECUTE_EVERY_N_MS(TOF_MS, read_tof_sensor());  //How to run if this has higher baud rate? Also what MS time?
#endif // ENABLE_TOF_SENSORS

      // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

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
