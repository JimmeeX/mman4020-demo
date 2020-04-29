#ifndef PUMP_H
#define PUMP_H

class Pump {
    private:
        int pin;
    public:
        Pump(int pin_);
        int getMotor();
        void set(bool value);
        void turnOn();
        void turnOff();
        // void purge();
};

#endif
