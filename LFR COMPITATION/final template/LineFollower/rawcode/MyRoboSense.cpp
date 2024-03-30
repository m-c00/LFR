#include "MyRoboSense.h"`

#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// boolean onoff = 0;
int val, cnt = 0;
float sum = 0;
// int v[3];
const uint16_t threshold = 500; // adjustable - can take values between 0 and 1000
int  position;
int oldsensorValues[7];


// Define your functions here
void MyRoboSense::IRAnalysis() {
    // Implementation of IRAnalysis function
}

void MyRoboSense::Find_reference() {
    // Implementation of Find_reference function
}

void MyRoboSense::ReadSonar() {
    // Implementation of ReadSonar function
}

void MyRoboSense::toggle() {
    // Implementation of toggle function
}

void MyRoboSense::WIRAnalysis() {
    // Implementation of WIRAnalysis function
}
