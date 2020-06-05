class FlowSensor {
  private:
    const int pin; // Arduino connection
    double flowRate; // Calculated flow rate
    volatile int pulses; // Interrupt variable
    
  public:
    int getPin(); // Retrieve pin connection
    void setPin(int pin); // Change assigned pin
    void resetCount(); // Resets flow pin/min
    void pulse(); // Regularly tracks inflow
    void calculateFlow(); // Calculates of flow
}

int getFlowPin() {
  return flowPin;
}

int setFlowPin(int flowPin) {
  this->flowPin = flowPin;
}

void resetCount() {
  pulses = 0;
}

void flowPulse() {
  pulses++;
}

void calculateFlow() {
  flowRate = pulses * 2.25; // Multiply pulses by 2.25mL (verify in datasheet)
  flowRate *= 60; // Convert sec->min so flowRate is mL/min
  flowRate /= 1000; // flowRate is in L/min
  Serial.println(flowRate);
}

void setup() {
  setFlowPin(2); // Select pin
  pinMode(flowPin, INPUT); // Data send from sensor to this pin
  attachInterrupt(0, flowPulse, RISING); // Activate interrupt when sensor reads value
}

void loop() {
  resetCount();
  calculateFlow();
  
}
