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
#include "tof_pub.h"
#include "DFRobot_TMF8x01.h"
#include <SoftwareSerial.h>
// #include <frost_interfaces/msg/u_command.h>

#define ENABLE_ACTUATORS
#define ENABLE_TOF_SENSORS
#define ENABLE_LEDS
#define ENABLE_BATTERY
#define ENABLE_BT_DEBUG

#define EN1       2                      // EN pin for left TMF8801
#define EN2       3                      // EN pin for right TMF8801
#define EN3       4                      // EN pin for front TMF8801
#define EN4       5                      // EN pin for back TMF8801
#define INT      -1                      // INT pin is floating, not used in this demo

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

// TOF Calibration data
uint8_t caliDataBuf[14] = {0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// publisher objects
BatteryPub battery_pub;

// TOF publisher object
TofPub tof_pub;

// sensor objects
SoftwareSerial BTSerial(BT_MC_RX, BT_MC_TX);
DFRobot_TMF8801 tofLeft(/*enPin =*/EN1, /*intPin=*/INT);
DFRobot_TMF8801 tofRight(/*enPin =*/EN2, /*intPin=*/INT);
DFRobot_TMF8801 tofFront(/*enPin =*/EN3, /*intPin=*/INT);
DFRobot_TMF8801 tofBack(/*enPin =*/EN4, /*intPin=*/INT);

// global values for ToF sensor
float left_distance = -1;
float right_distance = -1;
float front_distance = -1;
float back_distance = -1;

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
  tof_pub.setup(node);

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
  tof_pub.destroy(node);

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

#ifdef ENABLE_BATTERY
  pinMode(CURRENT_PIN, INPUT);
  pinMode(VOLT_PIN, INPUT);

#ifdef ENABLE_BT_DEBUG
  BTSerial.println("[INFO] Battery Sensor enabled");
#endif // ENABLE_BT_DEBUG
#endif // ENABLE_BATTERY

#ifdef ENABLE_TOF_SENSORS // TODO: Add ifdefs for BTSerial below
  
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
  BTSerial.println("[INFO] TOF sensors enabled");
#endif // ENABLE_BT_DEBUG
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

  // Update the sensors as fast as they're available
  if (tofLeft.isDataReady()) {
    left_distance = tofLeft.getDistance_mm();
  }

  if (tofRight.isDataReady()) {
    right_distance = tofRight.getDistance_mm();
  }

  if (tofFront.isDataReady()) {
    front_distance = tofFront.getDistance_mm();
  }

  if (tofBack.isDataReady()) {
    back_distance = tofBack.getDistance_mm();
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
