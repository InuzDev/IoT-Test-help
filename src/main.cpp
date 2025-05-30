#include <Arduino.h>

// Basic Robot Code - Motors + Ultrasonic Sensor
// L298N Motor Driver + HC-SR04 Ultrasonic Sensor

// L298N Motor Driver Pins
#define ENA 5 // Enable A - PWM pin for left motor speed
#define IN1 3 // Input 1 - Left motor direction
#define IN2 4 // Input 2 - Left motor direction
#define ENB 8 // Enable B - PWM pin for right motor speed
#define IN3 6 // Input 3 - Right motor direction
#define IN4 7 // Input 4 - Right motor direction

// HC-SR04 Ultrasonic Sensor Pins
#define TRIG_PIN 9
#define ECHO_PIN 10

// Motor speed constants
#define MOTOR_SPEED 200 // PWM value (0-255)

// Function definition to avoid "this function wasn't defined here"
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
int getDistance();

void setup()
{
  // Initialize Serial communication
  Serial.begin(9600);

  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Stop motors initially
  stopMotors();

  Serial.println("🤖 Basic Robot Initialized");
  Serial.println("Motors: L298N | Sensor: HC-SR04");
  delay(1000);
}

void loop()
{
  // Read distance from ultrasonic sensor
  int distance = getDistance();

  // Print distance reading
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Simple behavior: move forward if clear, stop if obstacle
  if (distance > 20)
  {
    moveForward();
    Serial.println("Moving forward");
  }
  else
  {
    stopMotors();
    Serial.println("Obstacle detected - stopping");
  }

  delay(500); // Wait half second before next reading
}

// === MOTOR CONTROL FUNCTIONS ===

void moveForward()
{
  // Set motor speed
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);

  // Left motor forward: IN1=HIGH, IN2=LOW
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor forward: IN3=HIGH, IN4=LOW
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward()
{
  // Set motor speed
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);

  // Left motor backward: IN1=LOW, IN2=HIGH
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right motor backward: IN3=LOW, IN4=HIGH
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft()
{
  // Set motor speed
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);

  // Left motor backward: IN1=LOW, IN2=HIGH
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right motor forward: IN3=HIGH, IN4=LOW
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight()
{
  // Set motor speed
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);

  // Left motor forward: IN1=HIGH, IN2=LOW
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor backward: IN3=LOW, IN4=HIGH
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors()
{
  // Turn off motor speed
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Set all motor pins to LOW
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// === ULTRASONIC SENSOR FUNCTION ===

int getDistance()
{
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in centimeters
  // Speed of sound = 343 meters/second = 0.0343 cm/microsecond
  // Distance = (time * speed) / 2 (divided by 2 because sound travels to object and back)
  int distance = (duration * 0.0343) / 2;

  // Return distance (limit to reasonable range)
  if (distance < 2 || distance > 400)
  {
    return 400; // Return max distance if out of range
  }

  return distance;
}