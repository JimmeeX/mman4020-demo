class Pump {
  private:
    const int motor; // Arduino pin connection
    
  public:
    int getMotor(); // Retrieves pin connection
    void turnOn(); // Turns on the motor
    void turnOff(); // Turns off the motor
    void purge(); // Execute purge cycle
}

int getMotor() {
  return motor;
}

void turnOn() {
  // 400ml/min
  digitalWrite(getMotor(), LOW);
}

void turnOff() {
  digitalWrite(getMotor(), HIGH);
}
