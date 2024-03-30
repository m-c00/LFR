#ifndef MyRoboSensors_h
#define MyRoboSensors_h

#include <Arduino.h>
#include <QTRSensors.h>
#include <NewPing.h>

class MyRoboSensors {
public:
    MyRoboSensors();

    // Sensor-related functions
    void IRAnalysis();
    void Find_reference();
    void ReadSonar();
    void toggle();

    // Add other sensor-related methods as needed
private:
    // Define private sensor-related variables and functions
};

#endif
