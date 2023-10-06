const int ledPin = 2; // Define the LED pin
char receivedChar;    // Variable to store the received character
boolean newData = false; // Flag to indicate whether new data has been received

void setup() {
  pinMode(ledPin, OUTPUT); // Set the LED pin as an output
  Serial.begin(9600);     // Initialize the serial communication
}

void loop() {
  // Check if new data has been received
  if (Serial.available()) {
    // Blink the LED for 0.5 seconds
    digitalWrite(ledPin, HIGH); // Turn on the LED
    delay(100);                 // Wait for 500 milliseconds (0.5 seconds)
    digitalWrite(ledPin, LOW);  // Turn off the LED
  }
}

// Serial data reception interrupt
void serialEvent() {
  while (Serial.available()) {
    receivedChar = Serial.read(); // Read the incoming character
    newData = true;              // Set the newData flag to true
  }
}
