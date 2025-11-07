#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

class Sensors{
    private:
        uint8_t  _trigpin, _echopin;

    public:
        void ultrasonic(uint8_t trigpin, uint8_t echopin);

        long readDistance();

};

#endif