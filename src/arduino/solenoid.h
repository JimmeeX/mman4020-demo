#ifndef SOLENOID_H
#define SOLENOID_H

class Solenoid {
    private:
        int pin;
    public:
        Solenoid(int pin_);
        void set(bool value);
        void openValve();
        void closeValve();
};

#endif
