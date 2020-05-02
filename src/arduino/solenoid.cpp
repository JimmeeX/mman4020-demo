#include "solenoid.h"
#include "Arduino.h"

Solenoid::Solenoid(int pin_) {
    pin = pin_;
};

void Solenoid::set(bool value) {
    if (value) {
        openValve();
    }
    else {
        closeValve();
    }
};

void Solenoid::openValve() {
    digitalWrite(pin, LOW);
};

void Solenoid::closeValve() {
    digitalWrite(pin, HIGH);
};
