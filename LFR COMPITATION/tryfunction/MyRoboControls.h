#ifndef MyRoboControls_h
#define MyRoboControls_h

#include <Arduino.h>
#include <QTRSensors.h>
#include <NewPing.h>

class MyRoboControls {
public:
    MyRoboControls();

    // Control-related functions
    void PID(int error, float Kp, float Ki, float Kd);
    void speedcontrol(int mota, int motb);
    void bridge_obstackle_control();
    void line_robot_control();
    void wall_robot_control();

    // Add other control-related methods as needed
private:
    // Define private control-related variables and functions
};

#endif
