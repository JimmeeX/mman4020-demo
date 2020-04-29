// Functions to deal with Solenoid Valves (Note there are 7 instances)
/*
 * Solenoid is connected to the relay board, so it corresponds with the module number
 * 7 Solenoids = 7 Relay outputs
 * - RELAY_ENABLE_1
 * - RELAY_ENABLE_2
 * - RELAY_ENABLE_3
 * - RELAY_ENABLE_4
 * - RELAY_ENABLE_5
 * - RELAY_ENABLE_6
 * - RELAY_ENABLE_7
*/

//class Solenoid {
//  public:
//    void setValve(int id, bool value);
//    void openValve(int id);
//    void closeValve(int id);
//};
//
//void setValve(int id, bool value) {
//  if (value) {
//    digitalWrite(id, HIGH);
//  }
//  else {
//    digitalWrite(id, LOW);
//  }
//}
//
//void openValve(int id) {
//  // Actuate the solenoid to OPEN
//  digitalWrite(id, HIGH);
//}
//
//void closeValve(int id) {
//  // Go back to default state (closed)
//  digitalWrite(id, LOW); 
//}
