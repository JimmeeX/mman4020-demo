// Functions to deal with Peristaltic Pump

class Pump {
  private:
    const int motor;
    
  public:
    int getMotor();
    void turnOn();
    void turnOff();
}

int getMotor() {
  return motor;
}

void turnOn() {
  digitalWrite(getMotor(), HIGH);
}

void turnOff() {
  digitalWrite(getMotor(), LOW);
}
