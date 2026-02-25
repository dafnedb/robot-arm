#include "Joystick.h"
#include <math.h>

Joystick::Joystick(int pX, int pY, int dz, int speed, float exp)
    : pinX(pX), pinY(pY), deadzone(dz), maxSpeed(speed), exponent(exp),
      centerX(2048), centerY(2048)
{
    pinMode(pinX, INPUT);
    pinMode(pinY, INPUT);
}

void Joystick::calibrate()
{
    centerX = analogRead(pinX);
    centerY = analogRead(pinY);
}

JoyPosition Joystick::readPosition()
{
    JoyPosition pos;
    pos.x = processAxis(analogRead(pinX), centerX);
    pos.y = processAxis(analogRead(pinY), centerY);
    return pos;
}

int Joystick::processAxis(int value, int center)
{
    value = constrain(value, 0, 4095);

    if (abs(value - center) < deadzone)
        return 0;

    int lineal;

    if (value < center)
        lineal = map(value, 0, center - deadzone, -maxSpeed, 0);
    else
        lineal = map(value, center + deadzone, 4095, 0, maxSpeed);

    lineal = constrain(lineal, -maxSpeed, maxSpeed);

    if (maxSpeed == 0)
        return 0;

    float normalized = (float)lineal / maxSpeed;
    float curved = pow(abs(normalized), exponent);
    if (normalized < 0)
        curved = -curved;

    return (int)(curved * maxSpeed);
}