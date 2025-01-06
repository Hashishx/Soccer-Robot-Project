#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* ssid = "Ehab";  // Replace with your Wi-Fi SSID
const char* password = "boody2002";  // Replace with your Wi-Fi password

// UDP setup
WiFiUDP udp;
const int udpPort = 4210;

// Infrared sensor pin and Servo pin
const int irPin = A0;  // Replace with your actual infrared sensor pin
const int servoPin = 2; // Replace with your actual servo pin
Servo myServo;

// Variables to manage infrared sensor and servo position
bool infrared_enabled = false;  // Infrared sensor is off by default
int servo_position = 0;  // Default servo position

// Motor control pins (example)
const int motor1_pwm = 5;  // Motor 1 speed control (PWM)
const int motor1_in1 = 4;  // Motor 1 input 1
const int motor1_in2 = 0;  // Motor 1 input 2

const int motor2_pwm = 14;  // Motor 2 speed control (PWM)
const int motor2_in1 = 12;  // Motor 2 input 1
const int motor2_in2 = 13;  // Motor 2 input 2

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Wait for Wi-Fi to connect
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  udp.begin(udpPort);
  myServo.attach(servoPin);

  pinMode(irPin, INPUT);  // Set infrared sensor pin as input
  pinMode(motor1_in1, OUTPUT);
  pinMode(motor1_in2, OUTPUT);
  pinMode(motor2_in1, OUTPUT);
  pinMode(motor2_in2, OUTPUT);
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
}

void loop() {
  char packetBuffer[512];
  int packetSize = udp.parsePacket();

  if (packetSize) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
    }

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, packetBuffer);
    if (error) {
      Serial.println("Failed to parse JSON");
      return;
    }

    // Handle infrared enabled/disabled commands
    if (doc.containsKey("infrared_enabled")) {
      infrared_enabled = doc["infrared_enabled"];
      Serial.print("Infrared Enabled: ");
      Serial.println(infrared_enabled ? "Yes" : "No");
    }

    // Handle servo position commands
    if (doc.containsKey("action") && String(doc["action"]) == "servo" && doc.containsKey("position")) {
      int position = doc["position"];
      myServo.write(position);
      servo_position = position;
      Serial.print("Servo Position: ");
      Serial.println(servo_position);
    }

    // Handle motor commands (example)
    if (doc.containsKey("action") && String(doc["action"]) == "move") {
      String move = doc["direction"];
      if (move == "forward") {
        moveForward();
        Serial.println("Moving forward");
      } else if (move == "backward") {
        moveBackward();
        Serial.println("Moving backward");
      } else if (move == "left") {
        turnLeft();
        Serial.println("Moving left");
      } else if (move == "right") {
        turnRight();
        Serial.println("Moving right");
      } else {
        stopMotors();
        Serial.println("Stop");
      }
    }
  }

  // Check for infrared sensor and handle object detection if needed
  if (infrared_enabled) {
    int sensorValue = analogRead(irPin);
    if (sensorValue > 500) {  // Threshold for detecting object
      Serial.println("Object detected!");
      // Move servo to 180 degrees when object detected
      controlServo(180);
    } else {
      Serial.println("No object detected.");
      // Move servo back to 90 degrees
      controlServo(90);
    }
  } else {
    // When infrared is off, don't perform any reading
    Serial.println("Infrared is off.");
  }

  delay(100);  // Small delay to prevent flooding of the loop
}

void moveForward() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, HIGH);
  digitalWrite(motor2_in1, HIGH);
  digitalWrite(motor2_in2, LOW);
  analogWrite(motor1_pwm, 200);
  analogWrite(motor2_pwm, 200);
}

void moveBackward() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, HIGH);
  analogWrite(motor1_pwm, 200);
  analogWrite(motor2_pwm, 200);
}

void turnLeft() {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, HIGH);
  digitalWrite(motor2_in2, LOW);
  analogWrite(motor1_pwm, 200);
  analogWrite(motor2_pwm, 200);
}

void turnRight() {
  digitalWrite(motor1_in1, LOW);
  digitalWrite(motor1_in2, HIGH);
  digitalWrite(motor2_in1, LOW);
  digitalWrite(motor2_in2, HIGH);
  analogWrite(motor1_pwm, 200);
  analogWrite(motor2_pwm, 200);
}

void stopMotors() {
  analogWrite(motor1_pwm, 0);
  analogWrite(motor2_pwm, 0);
}

void controlServo(int position) {
  myServo.write(position);
  servo_position = position;
}