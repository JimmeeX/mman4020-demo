// Arduino.ino Process Method:
// 1. Specify desired jar
// 2. Open specified valve
// 3. Pump in water
// 4. Purge to prepare for next smaple

/* --------------------------------
    ARDUINO->VALVE PIN CONNECTIONS
   -------------------------------- */
const int v1 = 1; // Valve 1 = Pin 1
const int v2 = 2; // Valve 2 = Pin 2
const int v3 = 3; // Valve 3 = Pin 3
const int v4 = 4; // Valve 4 = Pin 4
const int v5 = 5; // Valve 5 = Pin 5
const int v6 = 6; // Valve 6 = Pin 6
const int v7 = 7; // Valve 7 = Pin 7
const int pump = 10;

/* --------------
    VALVE STATES
   -------------- */
valve1 = false;
valve2 = false;
valve3 = false;
valve4 = false;
valve5 = false;
valve6 = false;
valve7 = false;

/* -----------------
    PRINT VARIABLES
   ----------------- */
int valveID;
String command;

void setup() {
  // Establish connection to Serial Monitor at Port 9600
  Serial.begin(9600);
  Serial.flush(); //flush all previous received and transmitted data
  
  // Register Valve Pins as OUTPUT modes (possibly the state can be mapped as boolean?)
  pinMode(v1, OUTPUT);
  pinMode(v2, OUTPUT);
  pinMode(v3, OUTPUT);
  pinMode(v4, OUTPUT);
  pinMode(v5, OUTPUT);
  pinMode(v6, OUTPUT);
  pinMode(v7, OUTPUT); 

  valveID = 0;
}

/* -----------------
    MAIN LOOP
   ----------------- */
void loop() {
  // 1) Identify which valves we want to open
  Serial.print("Toggle Valve (enter a number from 1-7):");
  if (Serial.available() > 0) {
    valveID = Serial.readStringUntil('\n');

    // Debug: print to test serial input is working
    Serial.print("Receieved: ");
    Serial.println(valveID);
  }

  if(valveID) {
    // Check it's a valid valve ID
    if(valveID >= 1 && valveID <= 7) {
      
      // Wait for command to start filling the jar
      Serial.print("Start filling the jar (y/n)?");
      if(Serial.available() > 0) {
        command = Serial.readStringUntil('\n');
        Serial.print("Command Receieved: ");
        Serial.println(command);
      }
  
      if(command.equals("y")) {
        // Start to fill the jar
        fillJar(valveID);
        purge();
      } else {
        Serial.println("Operation cancelled.");
      }
    }
  }
}

void getID() {
  
}

/* -----------------
    VALVE FUNCTIONS
   ----------------- */
void openValve(int id) {
  Serial.print("Opening valve ");
  Serial.print(id);
  // Relay active LOW
  digitalWrite(id, LOW);
}

void closeValve(int id) {
  digitalWrite(id, HIGH);
  Serial.print("Valve is now closed.");
}

toggleValve(id) {
  switch(id) {
    case 1:
      valve1 = !valve1;
      break
    case 2:
      valve1 = !valve1;
      break
    case 3:
      valve1 = !valve1;
      break
    case 4:
      valve1 = !valve1;
      break
    case 5:
      valve1 = !valve1;
      break
    case 6:
      valve1 = !valve1;
      break
    case 7:
      valve1 = !valve1;
      break
    default:
      break;
  }
}

void fillJar(int id) {
  // Open valve
  openValve(id);
  toggleValve(id); // Toggle valve state
        
  Serial.println(" for 10 seconds...");

  pumpWater(); 

  // Close valve (should be about full)
  closeValve(id);
}


/* ----------------
    PUMP FUNCTIONS
   ---------------- */
void pumpOn() {
  digitalWrite(pump, LOW);
}

void pumpOff() {
  digitalWrite(pump, HIGH);
}

void pumpWater() {
  pumpOn(); // Turn pump on
  
  Serial.println("Pumping in water...");
  delay(10000); // Keep it on for ~10s
  
  pumpOff(); // Turn pump off
}

void purge() {
  // Purge until arrived at next location?
  pumpOn(); // Turn pump on
  
  Serial.println("Purging containment for next sample.");
  delay(20000); // Keep it on for ~10s
  
  pumpOff(); // Turn pump off
}
