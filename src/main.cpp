#include <L298NX2.h>
#include <DHT.h>
#include <Servo.h>

// ====== Constants ====== //
#define CANNON_SERVO_PIN 13
#define DHTPIN A2
#define DHTTYPE DHT11
#define PUMP_EN 9   // Water pump
#define PUMP_IN1 10 // Control pin 1
#define PUMP_IN2 11 // control pin 2
#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 6
#define IN3 7
#define IN4 8
#define TRIG_PIN 14
#define ECHO_PIN 15
#define FLAME_SENSOR_PIN 12

const int DISTANCIA_SEGURA = 30;
const int DISTANCIA_ESQUIVAR = 15;
const int VELOCIDAD_NORMAL = 200;
const int VELOCIDAD_GIRO = 150;
const int VELOCIDAD_LENTA = 120;
const int TIMEOUT_ULTRASONICO = 25000;

const long SENSOR_INTERVAL = 100;
const long MOVE_INTERVAL = 50;
const int MAX_LECTURAS_INVALIDAS = 3;

bool fireByTemperature = false;
Servo cannonServo;
int cannonAngle = 90; // Cannon angle default

// ====== Temp Sensor ===== //
DHT dht(DHTPIN, DHTTYPE);
const float FIRE_TEMP_THRESHOLD = 400.0;

// ====== Enums ====== //
enum RobotState
{
  MOVING_FORWARD,
  AVOIDING_OBSTACLE,
  TURNING_LEFT,
  TURNING_RIGHT,
  BACKING_UP
};

// ====== Global Variables ====== //
L298NX2 motors(ENA, IN1, IN2, ENB, IN3, IN4);
RobotState currentState = MOVING_FORWARD;
unsigned long stateStartTime = 0;
unsigned long previousMoveMillis = 0;
unsigned long previousSensorMillis = 0;

int lecturas[5] = {100, 100, 100, 100, 100};
int indice_lectura = 0;
int distancia_anterior = 100;
int lecturas_invalidas = 0;

// ====== Function Declarations ====== //
void activatePump(int speed);
void deactivatePump();
void aimCannon(int angle);
void sweepCannon();
int medirDistancia();
int obtenerDistanciaFiltrada();
void moverAutonomo(int distancia, unsigned long currentTime);
void changeState(RobotState newState, unsigned long currentTime);
String getStateString(RobotState state);

// ====== Setup ====== //
void setup()
{
  dht.begin();
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);

  // Water pump
  pinMode(PUMP_EN, OUTPUT);
  pinMode(PUMP_IN1, OUTPUT);
  pinMode(PUMP_IN2, OUTPUT);

  // Ensure pump is off at start
  digitalWrite(PUMP_IN1, LOW);
  digitalWrite(PUMP_IN2, LOW);
  analogWrite(PUMP_EN, 0);

  cannonServo.attach(CANNON_SERVO_PIN);
  cannonServo.write(cannonAngle);

  motors.setSpeed(VELOCIDAD_NORMAL);
  motors.stop();

  Serial.println("🤖 Robot Evitador de Obstáculos Iniciado");
  delay(1000);
  stateStartTime = millis();
}

// ====== Main Loop ====== //
void loop()
{
  unsigned long currentMillis = millis();
  static int distancia_actual = 100;

  // Flame detection
  bool flameDetected = digitalRead(FLAME_SENSOR_PIN) == LOW;

  // Temperature
  float temperature = dht.readTemperature();
  if (isnan(temperature))
  {
    Serial.println("⚠ Error reading temperature");
    return;
  }

  Serial.print("🌡 Temp: ");
  Serial.print(temperature);
  Serial.println(" °C");

  fireByTemperature = (temperature >= FIRE_TEMP_THRESHOLD);
  bool fireDetected = flameDetected || fireByTemperature;

  if (currentMillis - previousSensorMillis >= SENSOR_INTERVAL)
  {
    previousSensorMillis = currentMillis;
    distancia_actual = obtenerDistanciaFiltrada();

    Serial.print("📏 Distancia: ");
    Serial.print(distancia_actual);
    Serial.print("cm | Estado: ");
    Serial.println(getStateString(currentState));
  }

  if (fireDetected)
  {
    Serial.println("🔥 Fire detected!");

    if (distancia_actual <= 20)
    {
      // Stop and extinguish
      motors.stop();
      sweepCannon();
      activatePump(200);
      delay(3000);
      deactivatePump();
    }
    else
    {
      // Approach slowly
      motors.setSpeed(VELOCIDAD_LENTA);
      motors.backward();
    }
  }
  else
  {
    // No fire: normal obstacle-avoiding logic
    if (currentMillis - previousMoveMillis >= MOVE_INTERVAL)
    {
      previousMoveMillis = currentMillis;
      moverAutonomo(distancia_actual, currentMillis);
    }
  }
}

void aimCannon(int angle)
{
  cannonAngle = constrain(angle, 0, 180);
  cannonServo.write(cannonAngle);
}

void sweepCannon()
{
  for (int pos = 60; pos <= 120; pos += 5)
  {
    aimCannon(pos);
    delay(100);
  }
  for (int pos = 120; pos >= 60; pos -= 5)
  {
    aimCannon(pos);
    delay(100);
  }
}

void activatePump(int speed = 255)
{
  digitalWrite(PUMP_IN1, HIGH);
  digitalWrite(PUMP_IN2, LOW);
  analogWrite(PUMP_EN, speed); // Max = 255
  Serial.println("💧 Pump Activated");
}

void deactivatePump()
{
  digitalWrite(PUMP_IN1, LOW);
  digitalWrite(PUMP_IN2, LOW);
  analogWrite(PUMP_EN, 0);
  Serial.println("🛑 Pump Deactivated");
}

// ====== Movimiento autónomo ====== //
void moverAutonomo(int distancia, unsigned long currentTime)
{
  unsigned long stateTime = currentTime - stateStartTime;

  switch (currentState)
  {
  case MOVING_FORWARD:
    if (distancia > DISTANCIA_SEGURA)
    {
      motors.setSpeed(VELOCIDAD_NORMAL);
      motors.backward(); // ← This now actually moves forward
    }
    else if (distancia > DISTANCIA_ESQUIVAR)
    {
      motors.setSpeed(VELOCIDAD_LENTA);
      motors.backward(); // ← Slow forward
    }
    else
    {
      Serial.println("🚫 Obstáculo detectado - iniciando maniobra");
      changeState(BACKING_UP, currentTime);
    }
    break;

  case BACKING_UP:
    motors.setSpeed(VELOCIDAD_GIRO);
    motors.forward(); // ← This now actually moves backward
    if (stateTime >= 600)
    {
      if (random(2) == 0)
      {
        Serial.println("↺ Girando izquierda");
        changeState(TURNING_LEFT, currentTime);
      }
      else
      {
        Serial.println("↻ Girando derecha");
        changeState(TURNING_RIGHT, currentTime);
      }
    }
    break;

  case TURNING_LEFT:
    motors.setSpeedA(VELOCIDAD_GIRO);
    motors.setSpeedB(VELOCIDAD_GIRO);
    motors.backwardA();
    motors.forwardB();
    if (stateTime >= 800)
    {
      Serial.println("✓ Giro completado - reanudando marcha");
      changeState(MOVING_FORWARD, currentTime);
    }
    break;

  case TURNING_RIGHT:
    motors.setSpeedA(VELOCIDAD_GIRO);
    motors.setSpeedB(VELOCIDAD_GIRO);
    motors.backwardA();
    motors.forwardB();
    if (stateTime >= 800)
    {
      Serial.println("✓ Giro completado - reanudando marcha");
      changeState(MOVING_FORWARD, currentTime);
    }
    break;

  case AVOIDING_OBSTACLE:
    changeState(MOVING_FORWARD, currentTime);
    break;
  }
}

// ====== Cambiar estado ====== //
void changeState(RobotState newState, unsigned long currentTime)
{
  currentState = newState;
  stateStartTime = currentTime;
  if (newState == MOVING_FORWARD)
  {
    motors.stop();
    delay(100);
  }
}

// ====== Mostrar estado en texto ====== //
String getStateString(RobotState state)
{
  switch (state)
  {
  case MOVING_FORWARD:
    return "Avanzando";
  case BACKING_UP:
    return "Retrocediendo";
  case TURNING_LEFT:
    return "Girando Izq";
  case TURNING_RIGHT:
    return "Girando Der";
  case AVOIDING_OBSTACLE:
    return "Esquivando";
  default:
    return "Desconocido";
  }
}

// ====== Medir distancia ====== //
int medirDistancia()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_ULTRASONICO);
  if (duration == 0)
    return -1;

  int distance = (duration * 0.0343) / 2;

  // Ignore impossible readings
  if (distance < 2 || distance > 400)
    return -1;

  return distance;
}

// ====== Obtener distancia filtrada ====== //
int obtenerDistanciaFiltrada()
{
  int lectura_actual = medirDistancia();

  if (lectura_actual > 0)
  {
    lecturas[indice_lectura] = lectura_actual;
    indice_lectura = (indice_lectura + 1) % 5;

    long suma = 0;
    for (int i = 0; i < 5; i++)
      suma += lecturas[i];
    int promedio = suma / 5;

    int diferencia = abs(promedio - distancia_anterior);
    if (diferencia > 50 && distancia_anterior != 100)
      return distancia_anterior;

    distancia_anterior = promedio;
    lecturas_invalidas = 0;
    return promedio;
  }
  else
  {
    lecturas_invalidas++;
    if (lecturas_invalidas >= MAX_LECTURAS_INVALIDAS)
      return DISTANCIA_SEGURA + 10;
    return distancia_anterior;
  }
}