// Functions to deal with Peristaltic Pump

class Pump {
  private:
    const int motor;
    
  public:
    int getMotor();
    void turnOn();
    void turnOff();
    void purge();
}

int getMotor() {
  return motor;
}

void turnOn() {
  // 400ml/min
  digitalWrite(getMotor(), HIGH);
}

void turnOff() {
  digitalWrite(getMotor(), LOW);
}
