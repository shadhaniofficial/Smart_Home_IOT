
#include <WiFi.h>
#include <Ultrasonic.h>

#define TRIGGER_PIN 2  // GPIO pin connected to ultrasonic sensor's trigger pin
#define ECHO_PIN 4     // GPIO pin connected to ultrasonic sensor's echo pin
#define Rpin 5    // GPIO pin connected to relay module's control pin

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN); // Define ultrasonic sensor object

const char* ssid = "shadhani";
const char* password = "123456789";

WiFiServer server(80);

const int mq6Pin = 34; // Analog pin connected to MQ6 AO pin (can be any ADC pin)

// Define pins for relay and motor control
int relayPin1 = 15;
const int motorPin1 = 19; // Digital pin connected to IN1 on the L298
const int motorPin2 = 18; // Digital pin connected to IN2 on the L298
int motor3 = 21;
int motor4 = 22;

// Define the threshold value for the gas concentration (determined through observation)
const int threshold = 3900; // Adjusted threshold value based on observed readings

// Timing variables
unsigned long previousMillis = 0;
const long interval = 10000; // 10 seconds

void setup() {
  Serial.begin(115200);
  delay(10);

  // Set static IP address
  IPAddress ip(192, 168, 137, 221);  // Static IP address
  IPAddress gateway(192, 168, 137, 1);  // Gateway IP address
  IPAddress subnet(255, 255, 255, 0);  // Subnet mask
  IPAddress dns(192, 168, 137, 1);  // DNS server
  
  WiFi.config(ip, gateway, subnet, dns);

  // Connect to Wi-Fi
  Serial.println();
  Serial.println("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(Rpin, OUTPUT); // Set relay pin as output

  // Set relay pins as outputs
  pinMode(relayPin1, OUTPUT);

  // Set motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

  // Initialize relays to OFF state
  digitalWrite(relayPin1, HIGH);

  // Initialize motors to OFF state
  // digitalWrite(motor1, HIGH);
  // digitalWrite(motor2, HIGH);
  digitalWrite(motor3, HIGH);
  digitalWrite(motor4, HIGH);

  // Start server
  server.begin();
}

void APP(){
    // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Wait until the client sends some data
  while (!client.available()) {
    delay(1);
  }

  // Read the first line of the request
  String request = client.readStringUntil('\r');
  client.flush();

  // Check if the request contains "RELAYON"
  if (request.indexOf("RELAYON") != -1) {
    digitalWrite(relayPin1, LOW);
  }

  // Check if the request contains "RELAYOFF"
  if (request.indexOf("RELAYOFF") != -1) {
    digitalWrite(relayPin1, HIGH);
  }

  // Check if the request contains "RELAYOFF4"
  if (request.indexOf("RELAYON4") != -1) {
    digitalWrite(motor3, LOW); // Motor3 OFF
  } 
  if (request.indexOf("RELAYOFF4") != -1){
    digitalWrite(motor3, HIGH); // Motor3 ON
  }

  // Send HTTP response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<head><title>ESP32 Web Relay Control</title></head>");
  client.println("<body>");
  client.println("<h1>Relay Control</h1>");
  client.println("<p>Relay 1 is currently: " + String(digitalRead(relayPin1) == HIGH ? "ON" : "OFF") + "</p>");
  client.println("<h1>Motor Control</h1>");
  client.println("<p>Motor 3 is currently: " + String(digitalRead(motor3) == HIGH ? "ON" : "OFF") + "</p>");
  client.println("<p>Motor 4 is currently: " + String(digitalRead(motor4) == HIGH ? "ON" : "OFF") + "</p>");
  client.println("</body>");
  client.println("</html>");

  // Close the connection
  delay(1);
  client.stop();
}

void Light(){
  long distance = ultrasonic.read(); // Read distance from sensor

  // Check if distance is less than a threshold (indicating an obstacle)
  if (distance < 10) { // Adjust this threshold as needed
    digitalWrite(Rpin, LOW); // Turn on relay
  } else {
    digitalWrite(Rpin, HIGH); // Turn off relay
  }

  delay(100); // Delay between distance readings

}

void MQ(){
  // Read the analog value from the MQ6 sensor
  int sensorValue = analogRead(mq6Pin);
  
  // Print the sensor value for debugging
  // Serial.print("Sensor Value: ");
  // Serial.println(sensorValue);
  
  // Check if the sensor value exceeds the threshold
  if (sensorValue > threshold) {
    // Get the current time
    unsigned long currentMillis = millis();
    
    // Check if 10 seconds have passed
    if (currentMillis - previousMillis >= interval) {
      // Save the last time the fan was turned on or off
      previousMillis = currentMillis;
      
      Serial.print("Sensor Value: ");
      Serial.println(sensorValue);

      // Turn on the motor (cooler fan) for 10 seconds
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      Serial.println("cooler fan turned on.");
      delay(10000); // Wait for 10 seconds
      
      // Turn off the motor (cooler fan) for 10 seconds
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      Serial.println("Cooler fan turned off.");
      delay(5000); // Wait for 5 seconds
    }
  } else {
    // Ensure the motor is off if the threshold is not exceeded
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    // Serial.println("Threshold not exceeded, cooler fan remains off.");
  }
  
  // Wait for a short time before reading again
  delay(500);
}

void loop() {
  APP();
  Light();
  MQ();
}
