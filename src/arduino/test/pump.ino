//// Functions to deal with Peristaltic Pump
//
//class Pump {
//  private:
//    const int motor;
//    
//  public:
//    int getMotor();
//    void set(bool value);
//    void turnOn();
//    void turnOff();
//    void purge();
//};
//
//int getMotor() {
//  return motor;
//}
//
//void set(bool value) {
//  if (value) {
//    digitalWrite(getMotor(), HIGH);
//  }
//  else {
//    digitalWrite(getMotor(), LOW);
//  }
//}
//
//void turnOn() {
//  // 400ml/min
//  digitalWrite(getMotor(), HIGH);
//}
//
//void turnOff() {
//  digitalWrite(getMotor(), LOW);
//}
