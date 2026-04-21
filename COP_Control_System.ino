# COP Control System

This file contains the implementation for the COP Control System, which calculates the center of pressure (COP) from four load cells and controls a remote car via HC05 Bluetooth module.

## COP Calculation and Calibration

#define LOAD_CELL_1  A0
#define LOAD_CELL_2  A1
#define LOAD_CELL_3  A2
#define LOAD_CELL_4  A3

float load_cells[4];
float cop_x, cop_y;

void setup() {
    Serial.begin(9600);
    // Initialize load cells here
}

void loop() {
    readLoadCells();
    calculateCOP();
    sendBluetoothCommands();
    delay(100); // Adjust the delay as necessary
}

void readLoadCells() {
    load_cells[0] = analogRead(LOAD_CELL_1);
    load_cells[1] = analogRead(LOAD_CELL_2);
    load_cells[2] = analogRead(LOAD_CELL_3);
    load_cells[3] = analogRead(LOAD_CELL_4);
}

void calculateCOP() {
    float totalForce = load_cells[0] + load_cells[1] + load_cells[2] + load_cells[3];
    cop_x = (load_cells[0] * 0 + load_cells[1] * 1 + load_cells[2] * 2 + load_cells[3] * 3) / totalForce;
    cop_y = (load_cells[0] * 0 + load_cells[1] * 0 + load_cells[2] * 1 + load_cells[3] * 1) / totalForce;
}

void sendBluetoothCommands() {
    if (cop_x > 1) {
        Serial.println("FRONT");
    } else if (cop_x < -1) {
        Serial.println("BACK");
    }
    if (cop_y > 1) {
        Serial.println("RIGHT");
    } else if (cop_y < -1) {
        Serial.println("LEFT");
    } else {
        Serial.println("STOP");
    }
}