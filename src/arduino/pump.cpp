#include "pump.h"
#include "Arduino.h"

Pump::Pump(int pin_) {
    pin = pin_;
};

void Pump::set(bool value) {
    if (value) {
        turnOn();
    }
    else {
        turnOff();
    }
};

void Pump::turnOn() {
    // 400ml/min
    digitalWrite(pin, LOW);
}

void Pump::turnOff() {
    digitalWrite(pin, HIGH);
}
