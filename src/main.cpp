#include <DHT.h>
#include <Servo.h>

// === Pin definitions ===
#define CANNON_SERVO_PIN 13
#define DHTPIN A2
#define DHTTYPE DHT11
#define PUMP_EN 9
#define PUMP_IN1 10
#define PUMP_IN2 11
#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 6
#define IN3 7
#define IN4 8
#define TRIG_PIN 2
#define ECHO_PIN 12
#define FLAME_SENSOR_PIN A0

// === Constants ===
const int DISTANCIA_SEGURA = 30;
const int DISTANCIA_ESQUIVAR = 5;
const int VELOCIDAD_NORMAL = 200;
const int VELOCIDAD_GIRO = 150;
const int VELOCIDAD_LENTA = 120;
const int TIMEOUT_ULTRASONICO = 25000;
const long SENSOR_INTERVAL = 100;
const long MOVE_INTERVAL = 50;
const int MAX_LECTURAS_INVALIDAS = 3;

// === Globals ===
bool fireByTemperature = false;
Servo cannonServo;
int cannonAngle = 90;

DHT dht(DHTPIN, DHTTYPE);

// === Setup ===
void setup()
{
  Serial.begin(9600);

  pinMode(PUMP_EN, OUTPUT);
  pinMode(PUMP_IN1, OUTPUT);
  pinMode(PUMP_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FLAME_SENSOR_PIN, INPUT);

  dht.begin();
  cannonServo.attach(CANNON_SERVO_PIN);
  cannonServo.write(cannonAngle);
}

// === Helper Functions ===
long medirDistancia()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duracion = pulseIn(ECHO_PIN, HIGH, TIMEOUT_ULTRASONICO);
  long distancia = duracion * 0.034 / 2;
  return distancia;
}

void moverAdelante(int velocidad)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velocidad);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, velocidad);
}

void moverAtras(int velocidad)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, velocidad);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, velocidad);
}

void girarIzquierda()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, VELOCIDAD_GIRO);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, VELOCIDAD_GIRO);
}

void girarDerecha()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, VELOCIDAD_GIRO);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, VELOCIDAD_GIRO);
}

void detener()
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void dispararAgua()
{
  Serial.println("Disparando agua...");
  digitalWrite(PUMP_IN1, HIGH);
  digitalWrite(PUMP_IN2, LOW);
  digitalWrite(PUMP_EN, HIGH);
  delay(12000);
  digitalWrite(PUMP_EN, LOW);
}

// === Main Loop ===
void loop()
{
  float temperatura = dht.readTemperature();
  int llamaDetectada = analogRead(FLAME_SENSOR_PIN);
  long distancia = medirDistancia();

  Serial.print("Llama valor: ");
  Serial.println(llamaDetectada);

  if (!isnan(temperatura))
  {
    Serial.print("Temp: ");
    Serial.println(temperatura);
    if (temperatura >= 40)
    {
      fireByTemperature = true;
    }
  }

  if (llamaDetectada < 700)
  {
    detener();
    cannonServo.write(90); // Centrar cañón
    delay(500);
    dispararAgua();
    fireByTemperature = false;
    delay(2000);
  }
  else if (distancia < DISTANCIA_SEGURA)
  {
    Serial.println("Obstáculo detectado, esquivando...");
    detener();
    delay(500);
    moverAtras(VELOCIDAD_LENTA);
    delay(1000);
    girarDerecha();
    delay(700);
    detener();
  }
  else
  {
    moverAdelante(VELOCIDAD_NORMAL);
  }

  delay(200);
}
