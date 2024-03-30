#ifndef MyRoboSense_h
#define MyRoboSense_h

#include <Arduino.h> // Include necessary Arduino headers
// #include <QTRSensors.h>
class MyRoboSense {
public:
    // Declare function prototypes
    static void IRAnalysis();
    static void Find_reference();
    static void ReadSonar();
    static void toggle();
    static void WIRAnalysis();
};

#endif
