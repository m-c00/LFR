#include <Arduino.h> // Include the Arduino core library
#include "MyRoboSensors.h"
#include "MyRoboControls.h"


// Define pin assignments
const int INAL = 9;
const int INBL = 8;
const int INCR = 7;
const int INDR = 6;
const int ENAL = 10;
const int ENBR = 5;
const int TFLR = 11;
const int EF = 4;
const int EL = 2;
const int ER = 3;

// Maximum distance for sonar sensors
#define MAX_DIS 200

// Other variables
unsigned int disFront = 0;
unsigned int disLeft = 0;
unsigned int disRight = 0;

MyRoboSensors sensors;
MyRoboControls controls;

void setup() {
    // Set motor driver pins as outputs
    pinMode(INAL, OUTPUT);
    pinMode(INBL, OUTPUT);
    pinMode(INCR, OUTPUT);
    pinMode(INDR, OUTPUT);
    pinMode(ENAL, OUTPUT);
    pinMode(ENBR, OUTPUT);

    // Your other setup code here
}

void loop() {
    // Your loop code here
    // Use sensors and controls functions from your library
    sensors.ReadSonar();
    
    // if (sensors.refobs == 1) {
        controls.bridge_obstackle_control();
    // }
    
    // if (sensors.refline == 1) {
        controls.line_robot_control();
    // } else if (sensors.refwall == 1) {
        controls.wall_robot_control();
    // }
}
