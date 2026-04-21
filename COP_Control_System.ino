/*
 * Four Load Cell HX711 - COP Control System with 5-Position Calibration
 * Arduino Uno + HX711 Modules + HC05 Bluetooth
 * 
 * CALIBRATION POSITIONS (60kg user):
 * 1. CENTER - Equal weight distribution
 * 2. LEFT - Lean left
 * 3. RIGHT - Lean right
 * 4. FORWARD - Lean forward
 * 5. BACKWARD - Lean backward
 */

#include "HX711.h"
#include <SoftwareSerial.h>

// ==================== PIN DEFINITIONS ====================
const int HX711_CLK_PIN = 3;  // SHARED by all 4 HX711 modules
const int HX711_DT_PIN1 = 2;   // Front-Left
const int HX711_DT_PIN2 = 4;   // Front-Right
const int HX711_DT_PIN3 = 6;   // Back-Left
const int HX711_DT_PIN4 = 8;   // Back-Right
const int BT_RX_PIN = 10;
const int BT_TX_PIN = 11;

// ==================== HX711 INSTANCES ====================
HX711 scale1, scale2, scale3, scale4;
SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

// ==================== CALIBRATION DATA STRUCTURE ====================
struct CalibrationPoint {
  float w1, w2, w3, w4;
  float total_weight;
  float cop_x, cop_y;
};

struct CalibrationData {
  CalibrationPoint center, left, right, forward, backward;
  bool is_valid;
};

CalibrationData calib_data;

// ==================== CONFIGURATION ====================
float calibration_factor1 = 420.0, calibration_factor2 = 420.0;
float calibration_factor3 = 420.0, calibration_factor4 = 420.0;
float x1 = -5.0, y1 = 5.0, x2 = 5.0, y2 = 5.0;
float x3 = -5.0, y3 = -5.0, x4 = 5.0, y4 = -5.0;
const float USER_WEIGHT = 60.0;
const int CALIB_TIME = 3000;

// ==================== VARIABLES ====================
float weight1, weight2, weight3, weight4;
float total_weight, cop_x, cop_y, cop_x_filtered, cop_y_filtered;
String current_command = "";
const int MOVING_AVG_SIZE = 5;
float cop_x_history[MOVING_AVG_SIZE] = {0};
float cop_y_history[MOVING_AVG_SIZE] = {0};
int history_index = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);
  delay(1000);
  
  Serial.println("\n\n========================================");
  Serial.println("  HX711 COP BALANCE BOARD");
  Serial.println("  5-Position Calibration - 60KG USER");
  Serial.println("========================================\n");
  
  Serial.println("Initializing Load Cells (HX711 - SHARED CLK)...");
  
  scale1.begin(HX711_DT_PIN1, HX711_CLK_PIN);
  scale2.begin(HX711_DT_PIN2, HX711_CLK_PIN);
  scale3.begin(HX711_DT_PIN3, HX711_CLK_PIN);
  scale4.begin(HX711_DT_PIN4, HX711_CLK_PIN);
  
  delay(100);
  
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  
  delay(100);
  
  scale1.tare();
  scale2.tare();
  scale3.tare();
  scale4.tare();
  
  Serial.println("Load Cells Ready!");
  Serial.println("HC05 Bluetooth Connected\n");
  Serial.println("Position Layout:");
  Serial.println("  Front-Left(FL)   Front-Right(FR)");
  Serial.println("  Back-Left(BL)    Back-Right(BR)\n");
  
  Serial.println("*** CALIBRATION SEQUENCE STARTING ***\n");
  performCalibration();
  
  Serial.println("\n✓ CALIBRATION COMPLETE!");
  Serial.println("Starting control loop...\n");
  delay(2000);
}

// ==================== CALIBRATION ====================
void performCalibration() {
  Serial.println("[1/5] CENTER POSITION");
  Serial.println("Stand with equal weight on all four corners.");
  Serial.println("Expected: ~15kg on each cell (60kg ÷ 4)");
  calibratePosition(&calib_data.center, "CENTER");
  delay(1500);
  
  Serial.println("\n[2/5] LEFT POSITION");
  Serial.println("Lean to the LEFT side.");
  Serial.println("Expected: Left ~25kg, Right ~10kg");
  calibratePosition(&calib_data.left, "LEFT");
  delay(1500);
  
  Serial.println("\n[3/5] RIGHT POSITION");
  Serial.println("Lean to the RIGHT side.");
  Serial.println("Expected: Right ~25kg, Left ~10kg");
  calibratePosition(&calib_data.right, "RIGHT");
  delay(1500);
  
  Serial.println("\n[4/5] FORWARD POSITION");
  Serial.println("Lean FORWARD (toward front of board).");
  Serial.println("Expected: Front ~25kg, Back ~10kg");
  calibratePosition(&calib_data.forward, "FORWARD");
  delay(1500);
  
  Serial.println("\n[5/5] BACKWARD POSITION");
  Serial.println("Lean BACKWARD (toward back of board).");
  Serial.println("Expected: Back ~25kg, Front ~10kg");
  calibratePosition(&calib_data.backward, "BACKWARD");
  
  calib_data.is_valid = true;
  printCalibrationSummary();
}

void calibratePosition(CalibrationPoint *pos, String position_name) {
  Serial.println("Reading for 3 seconds...");
  
  float sum_w1 = 0, sum_w2 = 0, sum_w3 = 0, sum_w4 = 0;
  float sum_cop_x = 0, sum_cop_y = 0;
  int samples = 0;
  
  unsigned long start_time = millis();
  
  while (millis() - start_time < CALIB_TIME) {
    weight1 = scale1.get_units(10);
    weight2 = scale2.get_units(10);
    weight3 = scale3.get_units(10);
    weight4 = scale4.get_units(10);
    total_weight = weight1 + weight2 + weight3 + weight4;
    
    if (total_weight > 1.0) {
      cop_x = (weight1 * x1 + weight2 * x2 + weight3 * x3 + weight4 * x4) / total_weight;
      cop_y = (weight1 * y1 + weight2 * y2 + weight3 * y3 + weight4 * y4) / total_weight;
    }
    
    sum_w1 += weight1;
    sum_w2 += weight2;
    sum_w3 += weight3;
    sum_w4 += weight4;
    sum_cop_x += cop_x;
    sum_cop_y += cop_y;
    samples++;
    
    Serial.print(".");
    delay(100);
  }
  
  pos->w1 = sum_w1 / samples;
  pos->w2 = sum_w2 / samples;
  pos->w3 = sum_w3 / samples;
  pos->w4 = sum_w4 / samples;
  pos->total_weight = pos->w1 + pos->w2 + pos->w3 + pos->w4;
  pos->cop_x = sum_cop_x / samples;
  pos->cop_y = sum_cop_y / samples;
  
  Serial.println();
  Serial.print("FL="); Serial.print(pos->w1, 2); Serial.print("kg ");
  Serial.print("FR="); Serial.print(pos->w2, 2); Serial.print("kg ");
  Serial.print("BL="); Serial.print(pos->w3, 2); Serial.print("kg ");
  Serial.print("BR="); Serial.print(pos->w4, 2); Serial.println("kg");
  Serial.print("Total="); Serial.print(pos->total_weight, 2); Serial.print("kg | ");
  Serial.print("COP: ("); Serial.print(pos->cop_x, 2); Serial.print(", "); 
  Serial.print(pos->cop_y, 2); Serial.println(")");
}

void printCalibrationSummary() {
  Serial.println("\n========================================");
  Serial.println("  CALIBRATION SUMMARY - 60KG USER");
  Serial.println("========================================\n");
  
  Serial.print("CENTER:   Total="); Serial.print(calib_data.center.total_weight, 2); Serial.println("kg");
  Serial.print("LEFT:     Total="); Serial.print(calib_data.left.total_weight, 2); Serial.print("kg | COP_X="); Serial.println(calib_data.left.cop_x, 2);
  Serial.print("RIGHT:    Total="); Serial.print(calib_data.right.total_weight, 2); Serial.print("kg | COP_X="); Serial.println(calib_data.right.cop_x, 2);
  Serial.print("FORWARD:  Total="); Serial.print(calib_data.forward.total_weight, 2); Serial.print("kg | COP_Y="); Serial.println(calib_data.forward.cop_y, 2);
  Serial.print("BACKWARD: Total="); Serial.print(calib_data.backward.total_weight, 2); Serial.print("kg | COP_Y="); Serial.println(calib_data.backward.cop_y, 2);
  
  float x_range = calib_data.right.cop_x - calib_data.left.cop_x;
  float y_range = calib_data.forward.cop_y - calib_data.backward.cop_y;
  
  Serial.println("\nDynamic Control Thresholds (30% of range):");
  Serial.print("X-axis (L/R): ±"); Serial.print(x_range * 0.3, 2); Serial.println(" cm");
  Serial.print("Y-axis (F/B): ±"); Serial.print(y_range * 0.3, 2); Serial.println(" cm");
  Serial.println("========================================\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  if (!calib_data.is_valid) {
    delay(100);
    return;
  }
  
  weight1 = scale1.get_units(10);
  weight2 = scale2.get_units(10);
  weight3 = scale3.get_units(10);
  weight4 = scale4.get_units(10);
  total_weight = weight1 + weight2 + weight3 + weight4;
  
  Serial.print(weight1);
  Serial.print("  ");
  Serial.print(weight2);
  Serial.print("  ");
  Serial.print(weight3);
  Serial.print("  ");
  Serial.println(weight4);
  
  if (total_weight > 1.0) {
    cop_x = (weight1 * x1 + weight2 * x2 + weight3 * x3 + weight4 * x4) / total_weight;
    cop_y = (weight1 * y1 + weight2 * y2 + weight3 * y3 + weight4 * y4) / total_weight;
  } else {
    cop_x = calib_data.center.cop_x;
    cop_y = calib_data.center.cop_y;
  }
  
  applyMovingAverage(cop_x, cop_y);
  current_command = determineMoveCommand();
  sendBluetoothCommand(current_command);
  
  delay(50);
}

// ==================== FUNCTIONS ====================
void applyMovingAverage(float new_x, float new_y) {
  cop_x_history[history_index] = new_x;
  cop_y_history[history_index] = new_y;
  history_index = (history_index + 1) % MOVING_AVG_SIZE;
  
  float sum_x = 0, sum_y = 0;
  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    sum_x += cop_x_history[i];
    sum_y += cop_y_history[i];
  }
  
  cop_x_filtered = sum_x / MOVING_AVG_SIZE;
  cop_y_filtered = sum_y / MOVING_AVG_SIZE;
}

String determineMoveCommand() {
  if (total_weight < 5.0) return "STOP";
  
  float x_threshold = (calib_data.right.cop_x - calib_data.left.cop_x) * 0.3;
  float y_threshold = (calib_data.forward.cop_y - calib_data.backward.cop_y) * 0.3;
  
  float x_offset = cop_x_filtered - calib_data.center.cop_x;
  float y_offset = cop_y_filtered - calib_data.center.cop_y;
  
  if (y_offset > y_threshold) return "FRONT";
  else if (y_offset < -y_threshold) return "BACK";
  else if (x_offset > x_threshold) return "RIGHT";
  else if (x_offset < -x_threshold) return "LEFT";
  
  return "STOP";
}

void sendBluetoothCommand(String cmd) {
  static String last_cmd = "";
  
  if (cmd != last_cmd) {
    char bt_command = 'S';
    if (cmd == "FRONT") bt_command = 'F';
    else if (cmd == "BACK") bt_command = 'B';
    else if (cmd == "LEFT") bt_command = 'L';
    else if (cmd == "RIGHT") bt_command = 'R';
    
    bluetooth.write(bt_command);
    Serial.print("BT: ");
    Serial.println(cmd);
    
    last_cmd = cmd;
  }
}

/*
 * HARDWARE: CLK Pin 3 -> All HX711 | DT Pins: 2,4,6,8
 * HC05: RX=10, TX=11 | Expected: CENTER=15kg ea, LEFT/RIGHT=25/10kg, FORWARD/BACKWARD=25/10kg
 */
