This four load cells balance board raw data is OK. Please create a new code for calculation for COP control a car left, right, forward, and back. it need to calibrate four control and center before operation/*
 * Four Load Cell HX711 - COP Calculation & HC05 Bluetooth Car Control
 * Arduino Uno Configuration
 * 
 * This code reads 4 load cells via HX711 modules, calculates Center of Pressure (COP),
 * and controls a remote car via HC05 Bluetooth based on pressure distribution.
 * 
 * REQUIRED LIBRARIES:
 * - HX711 Library by Rob Tillaart (https://github.com/RobTillaart/HX711)
 *   Install via: Sketch > Include Library > Manage Libraries > search "HX711" by Rob Tillaart
 */

#include "HX711.h"
#include <SoftwareSerial.h>

// ==================== PIN DEFINITIONS ====================
// All HX711 modules share the same CLK pin
const int HX711_CLK_PIN = 3;  // SHARED by all 4 HX711 modules

// Each Load Cell has its own DT (Data) pin
// Load Cell 1 (Front-Left)
const int HX711_DT_PIN1 = 2;

// Load Cell 2 (Front-Right)
const int HX711_DT_PIN2 = 4;

// Load Cell 3 (Back-Left)
const int HX711_DT_PIN3 = 6;

// Load Cell 4 (Back-Right)
const int HX711_DT_PIN4 = 8;

// HC05 Bluetooth Module Pins
// RX pin of HC05 -> TX pin of Arduino (Pin 10)
// TX pin of HC05 -> RX pin of Arduino (Pin 11)
const int BT_RX_PIN = 10;
const int BT_TX_PIN = 11;

// ==================== HX711 INSTANCES ====================
HX711 scale1;  // Front-Left
HX711 scale2;  // Front-Right
HX711 scale3;  // Back-Left
HX711 scale4;  // Back-Right

// ==================== BLUETOOTH SERIAL ====================
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);  // RX, TX

// ==================== CONFIGURATION ====================
// Calibration factors for each load cell
// These values need to be calibrated based on your specific setup
// See calibration function below
float calibration_factor1 = 420.0;   // Front-Left
float calibration_factor2 = 420.0;   // Front-Right
float calibration_factor3 = 420.0;   // Back-Left
float calibration_factor4 = 420.0;   // Back-Right

// Load cell positions (in cm, relative to center point)
// Adjust these based on your actual foot switch/load cell layout
float x1 = -5.0, y1 = 5.0;   // Front-Left (negative x, positive y)
float x2 = 5.0, y2 = 5.0;    // Front-Right (positive x, positive y)
float x3 = -5.0, y3 = -5.0;  // Back-Left (negative x, negative y)
float x4 = 5.0, y4 = -5.0;   // Back-Right (positive x, negative y)

// COP threshold values (in kg)
float total_weight_threshold = 0.5;  // Minimum weight to register input
float cop_threshold = 5.0;            // Distance threshold for direction change (cm)

// ==================== VARIABLES ====================
float weight1, weight2, weight3, weight4;
float total_weight;
float cop_x, cop_y;
float cop_x_filtered, cop_y_filtered;
String current_command = "";
unsigned long last_command_time = 0;
unsigned long command_interval = 100;  // Send command every 100ms

// 5-point moving average arrays for COP smoothing
const int MOVING_AVG_SIZE = 5;
float cop_x_history[MOVING_AVG_SIZE] = {0};
float cop_y_history[MOVING_AVG_SIZE] = {0};
int history_index = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);      // Serial monitor for debugging
  bluetooth.begin(9600);   // HC05 Bluetooth module (default baud rate)
  
  delay(1000);
  
  Serial.println("Initializing Load Cells...");
  
  // Initialize all HX711 modules with pin assignments
  scale1.begin(HX711_DT_PIN1, HX711_CLK_PIN);
  scale2.begin(HX711_DT_PIN2, HX711_CLK_PIN);
  scale3.begin(HX711_DT_PIN3, HX711_CLK_PIN);
  scale4.begin(HX711_DT_PIN4, HX711_CLK_PIN);
  
  delay(100);
  
  // Set calibration factors
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  
  delay(100);
  
  // Tare all scales (zero them out)
  scale1.tare();
  scale2.tare();
  scale3.tare();
  scale4.tare();
  
  Serial.println("Load Cells Ready!");
  Serial.println("HC05 Bluetooth Connected");
  Serial.println("");
  Serial.println("Position Layout:");
  Serial.println("  Front-Left(FL)   Front-Right(FR)");
  Serial.println("  Back-Left(BL)    Back-Right(BR)");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Read all load cells 
  weight1 = scale1.get_units(10);  // Front-Left
  weight2 = scale2.get_units(10);  // Front-Right
  weight3 = scale3.get_units(10);  // Back-Left
  weight4 = scale4.get_units(10);  // Back-Right
  
 Serial.print(weight1);
 Serial.print("  "); 
  Serial.print(weight2);
 Serial.print("  ");  
 Serial.print(weight3);
 Serial.print("  ");  
 Serial.println(weight4);
 
  delay(50);  // 20Hz sampling rate
}

// ==================== COP CALCULATION & MOVEMENT CONTROL ====================
// Apply 5-point moving average filter to smooth COP data
void applyMovingAverage(float new_x, float new_y) {
  // Store new values in circular buffer
  cop_x_history[history_index] = new_x;
  cop_y_history[history_index] = new_y;
  
  // Move to next position in circular buffer
  history_index = (history_index + 1) % MOVING_AVG_SIZE;
  
  // Calculate average of all stored values
  float sum_x = 0, sum_y = 0;
  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    sum_x += cop_x_history[i];
    sum_y += cop_y_history[i];
  }
  
  // Store filtered values
  cop_x_filtered = sum_x / MOVING_AVG_SIZE;
  cop_y_filtered = sum_y / MOVING_AVG_SIZE;
}

String determineMoveCommand() {
  // No significant weight detected
  if (total_weight < total_weight_threshold) {
    return "STOP";
  }
  
  // Determine primary direction based on FILTERED COP position
  // Y-axis: positive = forward/front, negative = backward/back
  // X-axis: positive = right, negative = left
  
  String command = "STOP";
  
  // Forward/Front movement (positive Y)
  if (cop_y_filtered > cop_threshold) {
    command = "FRONT";
  }
  // Backward/Back movement (negative Y)
  else if (cop_y_filtered < -cop_threshold) {
    command = "BACK";
  }
  // Right movement (positive X)
  else if (cop_x_filtered > cop_threshold) {
    command = "RIGHT";
  }
  // Left movement (negative X)
  else if (cop_x_filtered < -cop_threshold) {
    command = "LEFT";
  }
  
  return command;
}

// ==================== BLUETOOTH COMMUNICATION ====================
void sendBluetoothCommand(String cmd) {
  // Send command to HC05 Bluetooth module
  // Format: Single character command (can be modified for your car controller)
  
  char bt_command;
  
  if (cmd == "FRONT") {
    bt_command = 'F';
  } else if (cmd == "BACK") {
    bt_command = 'B';
  } else if (cmd == "LEFT") {
    bt_command = 'L';
  } else if (cmd == "RIGHT") {
    bt_command = 'R';
  } else {  // STOP
    bt_command = 'S';
  }
  
  //bluetooth.print(bt_command);
  
  //Serial.print("BT Command Sent: ");
  //Serial.println(cmd);
}

// ==================== DEBUG INFORMATION ====================
void printDebugInfo() {
  static unsigned long last_print = 0;
  
  // Print every 500ms to avoid serial clutter
  if (millis() - last_print > 500) {
    last_print = millis();
    /*
    Serial.print("Weights(kg): FL=");
    Serial.print(weight1, 2);
    Serial.print(" FR=");
    Serial.print(weight2, 2);
    Serial.print(" BL=");
    Serial.print(weight3, 2);
    Serial.print(" BR=");
    Serial.print(weight4, 2);
    
    Serial.print(" Total=");
    Serial.println(total_weight, 2);
    
    Serial.print("COP_RAW(cm): X=");
    Serial.print(cop_x, 2);
    Serial.print(" Y=");
    Serial.println(cop_y, 2);
    
    Serial.print("COP_FILTERED(cm): X=");
    */
    Serial.print(cop_x_filtered, 2);
    Serial.print(" ");
    //Serial.print(" Y=");
    Serial.println(cop_y_filtered, 2);
    
    //Serial.print("Command: ");
    //Serial.println(current_command);
    //Serial.println("---");
  }
}

// ==================== CALIBRATION FUNCTION ====================
// Call this function once to calibrate your load cells
// Instructions:
// 1. Make sure nothing is on the load cells
// 2. Call calibrateSingleCell() for each cell
// 3. Place known weight on the cell (e.g., 1kg)
// 4. Use the formula to calculate calibration factor
void calibrateSingleCell() {
  Serial.println("\n=== CALIBRATION MODE ===");
  Serial.println("Make sure the load cell is EMPTY and tared.");
  Serial.println("Place a KNOWN weight on the load cell.");
  Serial.println("Replace 1.0 with your actual weight in kg");
  Serial.println("The calibration factor will be printed.\n");
  
  // Example: calibrate first scale with 1kg weight
  float raw_reading = scale1.get_units(10);
  float known_weight = 1.0;  // kg
  float calc_factor = raw_reading / known_weight;
  
  Serial.print("Raw reading: ");
  Serial.println(raw_reading);
  Serial.print("Known weight: ");
  Serial.println(known_weight);
  Serial.print("Calculated calibration factor: ");
  Serial.println(calc_factor);
}

// ==================== NOTES ====================
/*
 * HARDWARE CONNECTIONS:
 * 
 * *** IMPORTANT: All 4 HX711 modules SHARE the same CLK pin (Pin 3) ***
 * 
 * HX711 #1 (Front-Left):
 *   - DT (Data)   -> Arduino Pin 2
 *   - CLK (Clock) -> Arduino Pin 3 (SHARED with all modules)
 *   - GND         -> Arduino GND
 *   - VCC         -> Arduino 5V
 * 
 * HX711 #2 (Front-Right):
 *   - DT (Data)   -> Arduino Pin 4
 *   - CLK (Clock) -> Arduino Pin 3 (SAME as module #1)
 *   - GND         -> Arduino GND
 *   - VCC         -> Arduino 5V
 * 
 * HX711 #3 (Back-Left):
 *   - DT (Data)   -> Arduino Pin 6
 *   - CLK (Clock) -> Arduino Pin 3 (SAME as modules #1 & #2)
 *   - GND         -> Arduino GND
 *   - VCC         -> Arduino 5V
 * 
 * HX711 #4 (Back-Right):
 *   - DT (Data)   -> Arduino Pin 8
 *   - CLK (Clock) -> Arduino Pin 3 (SAME as all other modules)
 *   - GND         -> Arduino GND
 *   - VCC         -> Arduino 5V
 * 
 * SUMMARY: CLK Pin 3 connects to ALL 4 HX711 modules (in parallel)
 *          DT Pins: 2, 4, 6, 8 (one per module)
 *          Total Arduino pins used: 5 (1 CLK + 4 DT)
 * 
 * HC05 Bluetooth Module:
 *   - VCC -> Arduino 5V (with voltage divider if needed)
 *   - GND -> Arduino GND
 *   - TX  -> Arduino Pin 10 (through voltage divider: 5V -> 3.3V)
 *   - RX  -> Arduino Pin 11 (direct connection)
 * 
 * *** IMPORTANT: HC05 operates at 3.3V, use a voltage divider for TX pin! ***
 * Voltage Divider for TX pin: 
 *   HC05 TX (5V) -> 1k Resistor -> Arduino Pin 10 -> 2k Resistor to GND
 * 
 * CALIBRATION:
 * 1. Place load cell on a flat surface
 * 2. With no weight: Arduino will auto-tare in setup()
 * 3. Place known weight (e.g., 1kg) and note the raw reading
 * 4. Calibration factor = raw reading / known weight
 * 5. Update calibration_factor1-4 variables with calculated values
 * 
 * COP CALCULATION:
 * COP (Center of Pressure) is calculated as weighted average of positions:
 * COP_X = (W1*X1 + W2*X2 + W3*X3 + W4*X4) / Total_Weight
 * COP_Y = (W1*Y1 + W2*Y2 + W3*Y3 + W4*Y4) / Total_Weight
 * 
 * Movement mapping:
 * - COP_Y > threshold: FRONT (move forward)
 * - COP_Y < -threshold: BACK (move backward)
 * - COP_X > threshold: RIGHT (turn right)
 * - COP_X < -threshold: LEFT (turn left)
 * - Otherwise: STOP
 * 
 * BLUETOOTH PROTOCOL:
 * Single character commands sent to remote car:
 * 'F' = FRONT, 'B' = BACK, 'L' = LEFT, 'R' = RIGHT, 'S' = STOP
 * Modify sendBluetoothCommand() if your car uses different protocol
 */
