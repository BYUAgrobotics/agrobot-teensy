#include "DFRobot_TMF8x01.h"
#include "Arduino.h"

#define EN1       2                      // EN pin for left TMF8801
#define EN2       3                      // EN pin for right TMF8801
#define EN3       4                      // EN pin for front TMF8801
// #define EN4       5                      // EN pin for back TMF8801
#define INT      -1                      // INT pin is floating, not used in this demo

// Create two sensor objects, one for each TMF8801
DFRobot_TMF8801 tofLeft(/*enPin =*/EN1, /*intPin=*/INT);
DFRobot_TMF8801 tofRight(/*enPin =*/EN2, /*intPin=*/INT);
DFRobot_TMF8801 tofFront(/*enPin =*/EN3, /*intPin=*/INT);
// DFRobot_TMF8801 tofBack(/*enPin =*/EN4, /*intPin=*/INT);

// Calibration data
uint8_t caliDataBuf[14] = {0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};

void setup() {
  Serial.begin(115200);
  while(!Serial) {
    // Wait for serial port to connect. Needed for native USB port only
  }

  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  // pinMode(EN4, OUTPUT);
  
  // Initially set all EN pins LOW
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(EN3, LOW);
  // digitalWrite(EN4, LOW);
  delay(50); // Give some time for sensors to reset

  // Initialize first sensor (left)
  digitalWrite(EN1, HIGH);  // Enable left sensor
  delay(50);                // Wait for the sensor to power up

  Serial.print("Initializing left TMF8801 sensor...");
  while(tofLeft.begin() != 0) {
    Serial.println("failed.");
    delay(1000);
  }
  Serial.println("done.");

  Serial.println(tofLeft.getUniqueID(), HEX); // check unique id before resetting address
  
  // Set I2C address for first sensor
  bool addr1 = tofLeft.setI2CAddress(1);  // Set to address 1
  if (addr1){
    Serial.println("failed to set addy 1");
    delay(10);
  }
  else{Serial.println("successfully set address 1!");}

  // Print first sensor info
  Serial.println("Left Sensor:");
  Serial.print("Software Version: ");
  Serial.println(tofLeft.getSoftwareVersion());
  Serial.print("Unique ID: ");
  Serial.println(tofLeft.getUniqueID(), HEX);
  Serial.print("Model: ");
  Serial.println(tofLeft.getSensorModel());

  // Initialize second sensor (right)
  digitalWrite(EN2, HIGH);  // Enable right sensor
  delay(50);                // Wait for the sensor to power up

  Serial.print("Initializing right TMF8801 sensor...");
  while(tofRight.begin() != 0) {
    Serial.println("failed.");
    delay(1000);
  }
  Serial.println("done.");
  
  // Set I2C address for second sensor
  tofRight.setI2CAddress(2);  // Set to address 2

  // Print second sensor info
  Serial.println("Right Sensor:");
  Serial.print("Software Version: ");
  Serial.println(tofRight.getSoftwareVersion());
  Serial.print("Unique ID: ");
  Serial.println(tofRight.getUniqueID(), HEX);
  Serial.print("Model: ");
  Serial.println(tofRight.getSensorModel());

  // Initialize third sensor (front)
  digitalWrite(EN3, HIGH);  // Enable front sensor
  delay(50);                // Wait for the sensor to power up

  Serial.print("Initializing front TMF8801 sensor...");
  while(tofFront.begin() != 0) {
    Serial.println("failed.");
    delay(1000);
  }
  Serial.println("done.");
  
  // Set I2C address for third sensor
  tofFront.setI2CAddress(3);  // Set to address 3

  // Print second sensor info
  Serial.println("Front Sensor:");
  Serial.print("Software Version: ");
  Serial.println(tofFront.getSoftwareVersion());
  Serial.print("Unique ID: ");
  Serial.println(tofFront.getUniqueID(), HEX);
  Serial.print("Model: ");
  Serial.println(tofFront.getSensorModel());

  // Initialize fourth sensor (back)
  digitalWrite(EN4, HIGH);  // Enable back sensor
  delay(50);                // Wait for the sensor to power up

  Serial.print("Initializing back TMF8801 sensor...");
  while(tofBack.begin() != 0) {
    Serial.println("failed.");
    delay(1000);
  }
  Serial.println("done.");
  
  // Set I2C address for fourth sensor
  tofBack.setI2CAddress(4);  // Set to address 3

  // Print Fourth sensor info
  Serial.println("Back Sensor:");
  Serial.print("Software Version: ");
  Serial.println(tofBack.getSoftwareVersion());
  Serial.print("Unique ID: ");
  Serial.println(tofBack.getUniqueID(), HEX);
  Serial.print("Model: ");
  Serial.println(tofBack.getSensorModel());



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
}

// void loop() {

//   if (tofLeft.isDataReady()) {
//     Serial.print("Left Distance = ");
//     Serial.print(tofLeft.getDistance_mm());
//     Serial.print(" mm    ");
//   }


//   if (tofRight.isDataReady()) {
//     Serial.print("Right Distance = ");
//     Serial.print(tofRight.getDistance_mm());
//     Serial.println(" mm");
//   }

//   if (tofFront.isDataReady()) {
//     Serial.print("Front Distance = ");
//     Serial.print(tofFront.getDistance_mm());
//     Serial.print(" mm    ");
//   }

//   // if (tofBack.isDataReady()) {
//   //   Serial.print("Back Distance = ");
//   //   Serial.print(tofBack.getDistance_mm());
//   //   Serial.println(" mm");
//   // }

//   delay(50); // Adjust delay to control the measurement rate
// }

// TODO: figure out why teensy can't power 4 sensors at the same time, then uncomment all the back sensor code