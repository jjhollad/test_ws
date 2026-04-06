/*
 * 4-Channel Relay Controller for Arduino DUE
 * Communicates with ROS2 via Serial
 * 
 * Commands:
 * RELAY1_ON,RELAY2_ON,RELAY3_ON,RELAY4_ON - Turn on relays
 * RELAY1_OFF,RELAY2_OFF,RELAY3_OFF,RELAY4_OFF - Turn off relays
 * STATUS - Get relay status
 */

// Relay control pins
const int RELAY1_PIN = 2;
const int RELAY2_PIN = 3;
const int RELAY3_PIN = 4;
const int RELAY4_PIN = 5;

// Relay states
bool relay1_state = false;   // Start with relays OFF
bool relay2_state = false;
bool relay3_state = false;
bool relay4_state = false;

// Serial communication
String inputString = "";
boolean stringComplete = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize relay pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  
  // Turn off all relays initially (inverted logic: HIGH = OFF)
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  digitalWrite(RELAY3_PIN, HIGH);
  digitalWrite(RELAY4_PIN, HIGH);
  
  // Reserve space for input string
  inputString.reserve(100);
  
  // Send ready message
  Serial.println("RELAY_CONTROLLER_READY");
}

void loop() {
  // Process serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Small delay
  delay(10);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  // Handle concatenated commands by splitting them
  if (command.indexOf("RELAY") != -1 && command.length() > 15) {
    // Likely concatenated commands, try to split them
    String remaining = command;
    while (remaining.length() > 0) {
      int nextRelay = remaining.indexOf("RELAY", 1);
      String singleCommand;
      
      if (nextRelay > 0) {
        singleCommand = remaining.substring(0, nextRelay);
        remaining = remaining.substring(nextRelay);
      } else {
        singleCommand = remaining;
        remaining = "";
      }
      
      if (singleCommand.length() > 0) {
        processSingleCommand(singleCommand);
      }
    }
    return;
  }
  
  processSingleCommand(command);
}

void processSingleCommand(String command) {
  if (command == "RELAY1_ON") {
    setRelay(1, true);
  }
  else if (command == "RELAY1_OFF") {
    setRelay(1, false);
  }
  else if (command == "RELAY2_ON") {
    setRelay(2, true);
  }
  else if (command == "RELAY2_OFF") {
    setRelay(2, false);
  }
  else if (command == "RELAY3_ON") {
    setRelay(3, true);
  }
  else if (command == "RELAY3_OFF") {
    setRelay(3, false);
  }
  else if (command == "RELAY4_ON") {
    setRelay(4, true);
  }
  else if (command == "RELAY4_OFF") {
    setRelay(4, false);
  }
  else if (command == "STATUS") {
    sendStatus();
  }
  else if (command == "ALL_ON") {
    setRelay(1, true);
    setRelay(2, true);
    setRelay(3, true);
    setRelay(4, true);
  }
  else if (command == "ALL_OFF") {
    setRelay(1, false);
    setRelay(2, false);
    setRelay(3, false);
    setRelay(4, false);
  }
  else if (command.length() > 0) {
    Serial.println("ERROR: Unknown command: " + command);
  }
}

void setRelay(int relayNum, bool state) {
  int pin;
  bool* relayState;
  
  switch (relayNum) {
    case 1:
      pin = RELAY1_PIN;
      relayState = &relay1_state;
      break;
    case 2:
      pin = RELAY2_PIN;
      relayState = &relay2_state;
      break;
    case 3:
      pin = RELAY3_PIN;
      relayState = &relay3_state;
      break;
    case 4:
      pin = RELAY4_PIN;
      relayState = &relay4_state;
      break;
    default:
      Serial.println("ERROR: Invalid relay number: " + String(relayNum));
      return;
  }
  
  // Set relay state (inverted logic: LOW = ON, HIGH = OFF)
  *relayState = state;
  digitalWrite(pin, state ? LOW : HIGH);
  
  // Send confirmation
  Serial.println("RELAY" + String(relayNum) + ":" + (state ? "ON" : "OFF"));
}

void sendStatus() {
  Serial.println("STATUS:" + 
                 String(relay1_state ? "1" : "0") + "," +
                 String(relay2_state ? "1" : "0") + "," +
                 String(relay3_state ? "1" : "0") + "," +
                 String(relay4_state ? "1" : "0"));
}

void sendError(String error) {
  Serial.println("ERROR: " + error);
}
