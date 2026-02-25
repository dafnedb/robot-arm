#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

struct JoyPosition
{
    int x;
    int y;
};

class Joystick
{
private:
    int pinX;
    int pinY;
    int maxSpeed;
    int centerX;
    int centerY;
    int deadzone;
    float exponent;

    int processAxis(int value, int center);

public:
    Joystick(int pX, int pY, int dz = 30, int speed = 200, float exp = 2.0);

    void calibrate();
    JoyPosition readPosition();
};

#endif