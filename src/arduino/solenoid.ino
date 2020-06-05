/**************************
Valve IDs range from 1 - 7 
**************************/
class Solenoid {
  public:
    void openValve(int id);
    void closeValve(int id);
}

void openValve(int id) {
  // Actuate the solenoid to OPEN
  digitalWrite(id, LOW);
}

void closeValve(int id) {
  // Go back to default state (closed)
  digitalWrite(id, HIGH); 
}
